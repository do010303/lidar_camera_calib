"""Live overlay of a 2D LiDAR scan on the camera image, coloured by range (blue=near, red=far)."""
from pathlib import Path
import time
from collections import deque
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .geometry import load_camera, scan_points, project
from .sysstats import SysStats


def load_extrinsic(path):
    with Path(path).expanduser().open() as f:
        data = yaml.safe_load(f)
    T = np.asarray(data['transform_4x4'], float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1)):
        raise ValueError('Invalid rigid transform')
    return data, T[:3, :3], T[:3, 3]


def scaled_camera(K, scale):
    """Intrinsics for an image resized by `scale` (pixel centres at +0.5, as cv2.resize does); D is unchanged."""
    Ks = np.array(K, float)
    Ks[:2] *= scale
    Ks[:2, 2] += 0.5*scale-0.5
    return Ks


def put_lines(frame, lines, x, scale):
    """White text with black outline, bottom-left, one line per entry (same look as CoVip's panels)."""
    font, fs = cv2.FONT_HERSHEY_SIMPLEX, 0.4*scale
    widest = max(cv2.getTextSize(t, font, fs, 1)[0][0] for t in lines)
    fs *= min(1., (frame.shape[1]-2*x-4)/max(widest, 1))   # shrink the block so long lines fit the image
    lh = int(round(34*fs))
    y0 = frame.shape[0]-int(4*scale)-lh*(len(lines)-1)
    for i, text in enumerate(lines):
        org = (x, y0+lh*i)
        cv2.putText(frame, text, org, font, fs, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, org, font, fs, (255, 255, 255), 1, cv2.LINE_AA)


def range_colors(ranges, near, far):
    """BGR jet colour per range; near -> blue, far -> red."""
    v = np.clip((np.asarray(ranges) - near) / max(far - near, 1e-6), 0, 1)
    gray = np.rint(v * 255).astype(np.uint8).reshape(-1, 1)
    return cv2.applyColorMap(gray, cv2.COLORMAP_JET).reshape(-1, 3)


def draw_scan(frame, points, ids, R, t, K, D, near=0.2, far=4.0, height_m=0.0,
              gap_m=0.15, link_px=60, radius=3, alpha=0.85, label_gap_px=80):
    """Draw scan points (and the polyline joining neighbours) on a copy of frame.

    points: Nx3 in the LiDAR frame (z=0); ids: scan indices, used to break the line at
    missing returns. height_m > 0 also draws a vertical stripe of that height above each
    point (LiDAR z is assumed up), so the 2D scan reads as a wall.
    label_gap_px > 0 writes the range (m) next to points, at least that many px apart horizontally.
    Returns (image, number of points inside the image).
    """
    h, w = frame.shape[:2]
    uv, src = project(points, R, t, K, D, w, h)
    if not len(src):
        return frame.copy(), 0
    colors = range_colors(np.linalg.norm(points[src, :2], axis=1), near, far)
    layer = frame.copy()
    if height_m > 0:
        top, top_src = project(points + [0, 0, height_m], R, t, K, D, w, h)
        top_by_src = dict(zip(top_src.tolist(), top))
        for pixel, i, c in zip(uv, src, colors):
            if i in top_by_src:
                cv2.line(layer, tuple(np.rint(pixel).astype(int)),
                         tuple(np.rint(top_by_src[i]).astype(int)), c.tolist(), 1, cv2.LINE_AA)
    for k in range(len(src) - 1):
        a, b = src[k], src[k + 1]
        if (ids[b] - ids[a] == 1 and np.linalg.norm(points[b] - points[a]) < gap_m
                and np.linalg.norm(uv[k + 1] - uv[k]) < link_px):
            cv2.line(layer, tuple(np.rint(uv[k]).astype(int)), tuple(np.rint(uv[k + 1]).astype(int)),
                     colors[k].tolist(), 2, cv2.LINE_AA)
    for pixel, c in zip(uv, colors):
        cv2.circle(layer, tuple(np.rint(pixel).astype(int)), radius, c.tolist(), -1, cv2.LINE_AA)
    out = cv2.addWeighted(layer, alpha, frame, 1 - alpha, 0)
    if label_gap_px > 0:
        last_u = -1e9
        for k in np.argsort(uv[:, 0]):
            if uv[k, 0] - last_u < label_gap_px:
                continue
            last_u = uv[k, 0]
            text = f'{np.linalg.norm(points[src[k], :2]):.2f}m'
            org = (int(uv[k, 0]) - 20, int(uv[k, 1]) - 10)
            cv2.putText(out, text, org, 0, .5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, text, org, 0, .5, (255, 255, 255), 1, cv2.LINE_AA)
    return out, len(src)


def add_colorbar(image, near, far):
    bar = range_colors(np.linspace(near, far, 200), near, far)
    h = image.shape[0]
    x0, y0 = image.shape[1] - 30, h // 2 - 100
    image[y0:y0 + 200, x0:x0 + 15] = bar[::-1].reshape(200, 1, 3)
    for text, y in ((f'{far:.1f}m', y0 - 6), (f'{near:.1f}m', y0 + 216)):
        cv2.putText(image, text, (x0 - 18, y), 0, .45, (0, 0, 0), 3)
        cv2.putText(image, text, (x0 - 18, y), 0, .45, (255, 255, 255), 1)


class Overlay(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay')
        share = Path(get_package_share_directory('lidar_camera_calib'))
        defaults = dict(image_topic='/camera/image_raw', scan_topic='/scan',
                        camera_yaml=str(share/'config/camera.yaml'),
                        extrinsic_yaml=str(Path.cwd()/'extrinsic.yaml'),
                        near_m=0.2, far_m=4.0, height_m=0.0, label_gap_px=80, max_pair_dt_s=0.10, max_age_s=1.0,
                        show_preview=True, publish_image=True, stream_width=640,
                        publish_topic='/lidar_camera_calib/overlay')
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.cfg, self.K, self.D = load_camera(self.p('camera_yaml'))
        self.extrinsic, self.R, self.t = load_extrinsic(self.p('extrinsic_yaml'))
        self.bridge = CvBridge()
        self.images, self.scans = deque(maxlen=30), deque(maxlen=60)
        self.last_stamp = None
        self.fps, self.last_tick = 0., None
        self.scan_times = deque(maxlen=20)
        self.stats = SysStats(); self.stats.start()
        self.create_subscription(Image, self.p('image_topic'),
                                 lambda m: self.images.append((m, time.monotonic())), qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p('scan_topic'),
                                 self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, self.p('publish_topic'), 1) if self.p('publish_image') else None
        self.create_timer(1 / 30., self.tick)

    def on_scan(self, msg):
        now = time.monotonic()
        self.scans.append((msg, now)); self.scan_times.append(now)

    def scan_hz(self):
        t = self.scan_times
        return (len(t)-1)/(t[-1]-t[0]) if len(t) > 1 and t[-1] > t[0] else 0.

    def p(self, key):
        return self.get_parameter(key).value

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def tick(self):
        now = time.monotonic()
        fresh = [(m, t) for m, t in self.images if now - t < self.p('max_age_s')]
        images = [m for m, _ in fresh]
        scans = [m for m, t in self.scans if now - t < self.p('max_age_s')]
        if not images or not scans:
            return
        imsg, received = fresh[-1]
        if self.stamp(imsg) == self.last_stamp:
            return   # timer runs faster than the camera; redrawing the same frame wastes CPU
        self.last_stamp = self.stamp(imsg)
        smsg = min(scans, key=lambda s: abs(self.stamp(s) - self.stamp(imsg)))
        dt = abs(self.stamp(imsg) - self.stamp(smsg))
        if dt > self.p('max_pair_dt_s'):
            self.get_logger().warning(f'Timestamp mismatch {dt:.3f}s', throttle_duration_sec=5.)
            return
        frame = self.bridge.imgmsg_to_cv2(imsg, 'bgr8')
        K = self.K
        width = int(self.p('stream_width'))
        scale = 1.
        if 0 < width < frame.shape[1]:
            scale = width/frame.shape[1]
            frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            K = scaled_camera(self.K, scale)
        points, ids, _, _ = scan_points(smsg)
        out, n = draw_scan(frame, points, ids, self.R, self.t, K, self.D,
                           self.p('near_m'), self.p('far_m'), self.p('height_m'),
                           label_gap_px=max(self.p('label_gap_px')*scale, 70 if self.p('label_gap_px') else 0))
        add_colorbar(out, self.p('near_m'), self.p('far_m'))
        t_now = time.monotonic()
        if self.last_tick is not None:
            self.fps = .9*self.fps+.1/max(t_now-self.last_tick, 1e-6) if self.fps else 1/max(t_now-self.last_tick, 1e-6)
        self.last_tick = t_now
        st = self.stats
        f = lambda v, fmt: '--' if v is None else fmt.format(v)
        s_ = out.shape[1]/640
        put_lines(out, [f'{n}/{len(points)} scan points in image | scan-image dt {dt*1000:.0f} ms | LiDAR {self.scan_hz():.1f} Hz',
                        f"{self.fps:.1f} FPS | delay {(t_now-received)*1000:.0f} ms | CPU {f(st.cpu_sys, '{:.0f}')}% | "
                        f"{f(st.temp, '{:.0f}')} C | node {f(st.cpu_node, '{:.0f}')}% of 1 core, {f(st.rss_mb, '{:.0f}')} MB"],
                  int(4*s_), s_)
        if self.pub is not None:
            omsg = self.bridge.cv2_to_imgmsg(out, 'bgr8')
            omsg.header = imsg.header
            self.pub.publish(omsg)
        if self.p('show_preview'):
            cv2.imshow('LiDAR on camera', out)
            if cv2.waitKey(1) & 255 == ord('q'):
                rclpy.shutdown()


def main():
    rclpy.init(); node = None
    try:
        node = Overlay(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
