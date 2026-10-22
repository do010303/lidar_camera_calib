"""Distortion-aware RAW overlay; selected board ROI is highlighted in green."""
from pathlib import Path
import cv2
import numpy as np
import yaml
import rclpy
from .capture import Capture
from .geometry import project


def draw_validation(data, R, t, K, D):
    """Render one paired sample, with no optimization or file writes."""
    frame = data['frame'].copy()
    h, w = frame.shape[:2]
    uv, _ = project(data['points'], R, t, K, D, w, h)
    for pixel in uv:
        cv2.circle(frame, tuple(np.rint(pixel).astype(int)), 2, (140,140,140), -1)
    selected = data['selected']
    chosen_uv, _ = project(selected, R, t, K, D, w, h)
    for pixel in chosen_uv:
        cv2.circle(frame, tuple(np.rint(pixel).astype(int)), 5, (0,255,0), -1)
        cv2.circle(frame, tuple(np.rint(pixel).astype(int)), 6, (0,0,0), 1)
    stats = None
    if not data['problems'] and len(selected):
        normal = cv2.Rodrigues(data['rvec'])[0][:,2]
        offset = -float(normal @ data['tvec'].ravel())
        camera_points = (R @ selected.T).T+t
        errors = camera_points @ normal+offset
        stats = dict(rmse_mm=float(np.sqrt(np.mean(errors**2))*1000),
                     bias_mm=float(np.mean(errors)*1000),
                     max_mm=float(np.max(np.abs(errors))*1000))
    lines = [f"dt={data['dt']:.3f}s | GREEN=selected ROI; GRAY=other scan | Q=quit",
             f"Selected: {len(selected)} | in image: {len(chosen_uv)} | Verify ROI belongs to board"]
    if stats:
        lines.append(f"Point-plane RMSE={stats['rmse_mm']:.1f} mm | bias={stats['bias_mm']:.1f} mm | max={stats['max_mm']:.1f} mm")
    else:
        lines.append('RMSE unavailable: '+'; '.join(data['problems']))
    for i,line in enumerate(lines):
        y=25+27*i
        cv2.putText(frame,line,(10,y),0,.55,(0,0,0),3)
        cv2.putText(frame,line,(10,y),0,.55,(0,255,255),1)
    return frame, stats


class Validate(Capture):
    def __init__(self):
        super().__init__('lidar_camera_calib_validate')
        self.declare_parameter('extrinsic_yaml',str(Path.cwd()/'extrinsic.yaml'))
        with Path(self.p('extrinsic_yaml')).expanduser().open() as f:
            self.extrinsic=yaml.safe_load(f)
        T=np.asarray(self.extrinsic['transform_4x4'],float)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1):
            raise ValueError('Invalid rigid transform')
        self.R,self.t=T[:3,:3],T[:3,3]
        self.declare_parameter('compare_extrinsic_yaml','')
        self.comparison=None
        if self.p('compare_extrinsic_yaml'):
            with Path(self.p('compare_extrinsic_yaml')).expanduser().open() as f:
                other=yaml.safe_load(f)
            C=np.asarray(other['transform_4x4'],float)
            if C.shape!=(4,4) or not np.isfinite(C).all() or not np.allclose(C[3],[0,0,0,1]) or not np.allclose(C[:3,:3].T@C[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(C[:3,:3]),1):
                raise ValueError('Invalid comparison rigid transform')
            if any(other[k]!=self.extrinsic[k] for k in ('source_frame','target_frame')):
                raise ValueError('Comparison transform frame mismatch')
            self.comparison=C

    def capture_callback(self, request, response):
        response.success=False
        response.message='Validation mode: capture disabled'
        return response

    def preview(self):
        comparison_frame=None
        try:
            data=self.prepare()
            if data['smsg'].header.frame_id!=self.extrinsic['source_frame'] or data['imsg'].header.frame_id!=self.extrinsic['target_frame']:
                raise ValueError('FRAME MISMATCH: no overlay or RMSE')
            frame, stats=draw_validation(data,self.R,self.t,self.K,self.D)
            status = (f"Selected {len(data['selected'])}; RMSE {stats['rmse_mm']:.1f} mm"
                      if stats else '; '.join(data['problems']))
            if self.comparison is not None:
                comparison_frame, other_stats=draw_validation(data,self.comparison[:3,:3],self.comparison[:3,3],self.K,self.D)
                for im,label in ((frame,'A: '+Path(self.p('extrinsic_yaml')).name),
                                 (comparison_frame,'B: '+Path(self.p('compare_extrinsic_yaml')).name)):
                    cv2.putText(im,label,(10,110),0,.55,(0,0,0),3)
                    cv2.putText(im,label,(10,110),0,.55,(255,255,255),1)
                if stats and other_stats:
                    status=f"Same pair/ROI: A RMSE={stats['rmse_mm']:.1f} mm; B RMSE={other_stats['rmse_mm']:.1f} mm"
            self.get_logger().info(status, throttle_duration_sec=3.)
        except (ValueError,cv2.error) as e:
            # Replace old image so stale successful overlays cannot masquerade as live data.
            frame=np.zeros((180,1000,3),np.uint8)
            cv2.putText(frame,str(e)[:115],(10,45),0,.55,(0,100,255),1)
            comparison_frame=frame.copy() if self.comparison is not None else None
            self.get_logger().warning(str(e),throttle_duration_sec=5.)
        if self.p('show_preview'):
            if comparison_frame is not None:
                # One panel ensures both views correspond to the exact same buffered pair.
                panel=np.hstack((frame,comparison_frame))
                scale=min(1.,1600/panel.shape[1])
                panel=cv2.resize(panel,None,fx=scale,fy=scale)
                cv2.imshow('LiDAR camera validation',panel)
            else:
                cv2.imshow('LiDAR camera validation',frame)
            if cv2.waitKey(1)&255==ord('q'):
                rclpy.shutdown()


def main():
    rclpy.init(); node=None
    try:
        node=Validate(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
