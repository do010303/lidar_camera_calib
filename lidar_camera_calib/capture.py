"""Interactive board capture. ROI selection requires human verification."""
import time
from collections import deque
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .geometry import load_camera, scan_points, select_segment


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


class Capture(Node):
    def __init__(self, node_name='lidar_camera_calib_capture'):
        super().__init__(node_name)
        defaults = dict(image_topic='/camera/image_raw', scan_topic='/scan',
            camera_yaml=str(Path(get_package_share_directory('lidar_camera_calib'))/'config/camera.yaml'),
            output_dir=str(Path.cwd()/'samples'), show_preview=True,
            angle_min_deg=-20., angle_max_deg=20., min_range_m=.15, max_range_m=1.5,
            min_board_points=6, max_pair_dt_s=.10, max_age_s=1.,
            cluster_gap_m=.04, max_line_rmse_m=.008, max_reprojection_px=1.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg, self.K, self.D = load_camera(self.p('camera_yaml'))
        self.board_size = (self.cfg['board_cols'], self.cfg['board_rows'])
        self.objp = np.zeros((np.prod(self.board_size), 3), float)
        self.objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)*self.cfg['square_size_m']
        self.images, self.scans = deque(maxlen=30), deque(maxlen=60)
        self.bridge = CvBridge()
        self.output = Path(self.p('output_dir')).expanduser()
        self.output.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, self.p('image_topic'), lambda m:self.images.append((m,time.monotonic())), qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p('scan_topic'), lambda m:self.scans.append((m,time.monotonic())), qos_profile_sensor_data)
        self.create_service(Trigger, '~/capture', self.capture_callback)
        self.status = 'Waiting for image and scan; C=capture, Q=quit'
        self.create_timer(.15, self.preview)

    def p(self, key):
        return self.get_parameter(key).value

    def prepare(self):
        now = time.monotonic()
        images = [(m,t) for m,t in self.images if now-t < self.p('max_age_s')]
        scans = [(m,t) for m,t in self.scans if now-t < self.p('max_age_s')]
        if not images or not scans:
            raise ValueError('No fresh image/scan')
        # Use newest image and nearest buffered scan, never an old best pair.
        imsg = images[-1][0]
        smsg = min(scans, key=lambda s:abs(stamp_seconds(s[0])-stamp_seconds(imsg)))[0]
        dt = abs(stamp_seconds(imsg)-stamp_seconds(smsg))
        if dt > self.p('max_pair_dt_s'):
            raise ValueError(f'Timestamp mismatch {dt:.3f}s')
        frame = self.bridge.imgmsg_to_cv2(imsg, 'bgr8')
        if frame.shape[1::-1] != (self.cfg['image_width'],self.cfg['image_height']):
            raise ValueError('Image resolution differs from intrinsic')
        points, ids, angles, ranges = scan_points(smsg)
        amin, amax = np.deg2rad([self.p('angle_min_deg'),self.p('angle_max_deg')])
        gate = ((angles >= amin)&(angles <= amax)) if amin <= amax else ((angles >= amin)|(angles <= amax))
        gate &= (ranges >= self.p('min_range_m')) & (ranges <= self.p('max_range_m'))
        selected = np.empty((0,3)); selected_ids = np.array([],int)
        problems = []
        line_error = np.nan
        try:
            selected, selected_ids, line_error = select_segment(points, ids, gate,
                self.p('min_board_points'),self.p('cluster_gap_m'),self.p('max_line_rmse_m'))
        except ValueError as e:
            problems.append(str(e))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, self.board_size)
        rvec = tvec = None
        reproj = np.nan
        if ok:
            corners = cv2.cornerSubPix(gray,corners,(7,7),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.001))
            ok, rvec, tvec = cv2.solvePnP(self.objp,corners,self.K,self.D)
            if ok:
                projected = cv2.projectPoints(self.objp,rvec,tvec,self.K,self.D)[0]
                reproj = float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=2))))
                R = cv2.Rodrigues(rvec)[0]
                if np.min((R@self.objp.T+tvec)[2]) <= 0 or reproj > self.p('max_reprojection_px'):
                    problems.append(f'Invalid board pose / reprojection {reproj:.2f}px')
            else:
                problems.append('solvePnP failed')
            cv2.drawChessboardCorners(frame,self.board_size,corners,True)
        else:
            problems.append('Checkerboard not detected')
        return dict(frame=frame,imsg=imsg,smsg=smsg,dt=dt,points=points,gate=gate,
                    selected=selected,selected_ids=selected_ids,corners=corners,rvec=rvec,tvec=tvec,
                    reproj=reproj,line_error=line_error,problems=problems)

    def save(self, d):
        if d['problems']:
            raise ValueError('; '.join(d['problems']))
        if not d['imsg'].header.frame_id or not d['smsg'].header.frame_id:
            raise ValueError('Sensor frame_id must be nonempty')
        name = f'sample_{time.time_ns()}'
        R = cv2.Rodrigues(d['rvec'])[0]; normal = R[:,2]
        np.savez_compressed(self.output/(name+'.npz'),points_l=d['selected'],normal_c=normal,
            offset_c=-float(normal@d['tvec'].ravel()),corners=d['corners'].reshape(-1,2),
            board_rvec=d['rvec'],board_tvec=d['tvec'],stamp_image=stamp_seconds(d['imsg']),
            stamp_scan=stamp_seconds(d['smsg']),source_frame=d['smsg'].header.frame_id,
            target_frame=d['imsg'].header.frame_id,K=self.K,D=self.D,
            board_size=self.board_size,square_size_m=self.cfg['square_size_m'],
            image_size=[self.cfg['image_width'],self.cfg['image_height']],
            line_rmse_m=d['line_error'],reprojection_rmse_px=d['reproj'],scan_indices=d['selected_ids'],
            all_points_l=d['points'],roi_mask=d['gate'],
            roi_parameters=np.array([self.p(k) for k in ('angle_min_deg','angle_max_deg','min_range_m','max_range_m')]))
        cv2.imwrite(str(self.output/(name+'.png')),d['frame'])
        self.status = f'Saved {name}: {len(d["selected"])} points'
        self.get_logger().info(self.status)

    def capture_callback(self, request, response):
        try:
            self.save(self.prepare()); response.success=True; response.message=self.status
        except Exception as e:
            response.success=False; response.message=str(e)
        return response

    def preview(self):
        if not self.p('show_preview'):
            return
        try:
            d = self.prepare()
            image = d['frame'].copy()
            message = '; '.join(d['problems']) or f'READY: {len(d["selected"])} points; verify board ROI; C=save'
            cv2.putText(image,message,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,255),1)
            cv2.imshow('Checkerboard',image)
            canvas = np.full((650,650,3),255,np.uint8)
            scale = 300/max(.5,self.p('max_range_m'))
            for p, gated in zip(d['points'],d['gate']):
                xy = (int(325-p[1]*scale),int(610-p[0]*scale))
                if 0 <= xy[0] < 650 and 0 <= xy[1] < 650:
                    cv2.circle(canvas,xy,2,(0,180,255) if gated else (170,170,170),-1)
            for p in d['selected']:
                cv2.circle(canvas,(int(325-p[1]*scale),int(610-p[0]*scale)),4,(0,150,0),-1)
            cv2.putText(canvas,'Top view: up=laser +X, left=+Y; green=selected',(8,25),0,.45,(0,0,0),1)
            cv2.putText(canvas,self.status[-85:],(8,48),0,.4,(0,0,0),1)
            cv2.imshow('LiDAR ROI',canvas)
        except ValueError as e:
            self.status=str(e)
            self.get_logger().warning(self.status, throttle_duration_sec=5.)
        key = cv2.waitKey(1)&255
        if key == ord('c'):
            try: self.save(self.prepare())
            except ValueError as e:
                self.status=str(e); self.get_logger().warning(self.status)
        elif key == ord('q'):
            rclpy.shutdown()


def main():
    rclpy.init(); node=None
    try:
        node=Capture(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
