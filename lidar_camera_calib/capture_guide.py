"""Capture with live guidance: tells you when a board pose is good to save and what pose is still missing."""
import sys
from pathlib import Path
import cv2
import numpy as np
import rclpy
from .capture import Capture
from .geometry import candidate_segments, scan_points

# Coverage bins over already-saved poses: (label, value extractor, lo, hi)
TILT = .30            # |normal x or y| beyond this (~17 deg) counts as a tilted pose; calibration needs strong tilts
DIST_BINS = ((.35, .6, 'near 0.35-0.6 m'), (.6, .9, 'mid 0.6-0.9 m'), (.9, 1.4, 'far 0.9-1.4 m'))
TILT_BINS = (('x', -1, 'tilt board left (right edge toward camera)'), ('x', 1, 'tilt board right'),
             ('y', -1, 'tilt board top toward camera'), ('y', 1, 'tilt board top away'))


def pose_descriptor(normal, tvec):
    return np.array([normal[0], normal[1], float(np.ravel(tvec)[2])])


def load_saved(output_dir):
    out = []
    for path in sorted(Path(output_dir).glob('sample_*.npz')):
        with np.load(path, allow_pickle=False) as d:
            out.append(pose_descriptor(d['normal_c'], d['board_tvec']))
    return out


def coverage(saved):
    """Return a list of missing pose categories given descriptors of saved samples."""
    missing = []
    for lo, hi, label in DIST_BINS:
        if not any(lo <= s[2] < hi for s in saved):
            missing.append(label)
    for axis, sign, label in TILT_BINS:
        i = 0 if axis == 'x' else 1
        if not any(sign*s[i] > TILT for s in saved):
            missing.append(label)
    return missing


def pick_board_segment(clusters, expected, p):
    """Choose the cluster that is the board among hands/bodies/background, using the board pose from the camera.

    expected: (forward, left) of the board centre in the LiDAR frame, assuming camera ~ above LiDAR.
    Straight clusters of plausible length near the expected spot qualify; the biggest wins unless a rival
    has >=70% of its points (then ambiguous). Returns (cluster or None, message).
    """
    ok = []
    for c in clusters:
        q = c['points']; m = q.mean(axis=0)
        length = float(np.linalg.norm(q[-1]-q[0]))
        if (c['straight'] and p['seg_min'] <= length <= p['seg_max']
                and abs(m[1]-expected[1]) <= p['lateral_tol'] and abs(m[0]-expected[0]) <= p['depth_tol']):
            ok.append(c)
    if not ok:
        return None, (f'no LiDAR segment matches the board (expected ~{expected[0]:.2f} m ahead, '
                      f'{expected[1]:+.2f} m left); hold board so the scan plane cuts it')
    ok.sort(key=lambda c: -len(c['points']))
    if len(ok) > 1 and len(ok[1]['points']) >= .7*len(ok[0]['points']):
        return None, f'{len(ok)} similar candidates near the board; move hands/body out of the scan plane'
    return ok[0], ''


def assess(data, saved, moving_px, p):
    """Judge whether the current pair should be captured.

    Returns (level, reasons, hints): level is 'good', 'similar' or 'bad'.
    p: dict with min_points, min_dist, max_dist, seg_min, seg_max, lateral_tol, depth_tol,
    still_px, novelty_normal, novelty_dist.
    """
    reasons = list(data['problems'])
    sel = data['selected']
    if not reasons:
        if len(sel) < p['min_points']:
            reasons.append(f'only {len(sel)} LiDAR points (<{p["min_points"]}); move board closer')
        R = cv2.Rodrigues(data['rvec'])[0]
        tvec = data['tvec'].ravel()
        if not p['min_dist'] <= tvec[2] <= p['max_dist']:
            reasons.append(f'board {tvec[2]:.2f} m away; keep {p["min_dist"]}-{p["max_dist"]} m')
        length = float(np.linalg.norm(sel[-1]-sel[0])) if len(sel) > 1 else 0.
        if not p['seg_min'] <= length <= p['seg_max']:
            reasons.append(f'LiDAR segment {length*100:.0f} cm long, board expects {p["seg_min"]*100:.0f}-{p["seg_max"]*100:.0f} cm: wrong object in ROI?')
        centre = tvec+R@np.array([np.mean(data['objp_x']), np.mean(data['objp_y']), 0.]) if 'objp_x' in data else tvec
        c = sel.mean(axis=0)
        if abs(-c[1]-centre[0]) > p['lateral_tol'] or abs(c[0]-centre[2]) > p['depth_tol']:
            reasons.append(f'LiDAR segment (x={c[0]:.2f}, y={c[1]:.2f}) does not match board in camera '
                           f'(fwd={centre[2]:.2f}, left={-centre[0]:.2f}): ROI is not the board')
        if moving_px > p['still_px']:
            reasons.append(f'board moving ({moving_px:.1f} px); hold still')
    if reasons:
        return 'bad', reasons, []
    desc = pose_descriptor(cv2.Rodrigues(data['rvec'])[0][:, 2], data['tvec'])
    novel = all(np.degrees(np.arccos(np.clip(np.dot(_unit(desc), _unit(s)), -1, 1))) > p['novelty_normal']
                or abs(desc[2]-s[2]) > p['novelty_dist'] for s in saved)
    hints = coverage(saved + [desc]) if novel else coverage(saved)
    return ('good' if novel else 'similar'), [], hints


def _unit(d):
    n = np.array([d[0], d[1], np.sqrt(max(1-d[0]**2-d[1]**2, 0))])
    return n/np.linalg.norm(n)


class CaptureGuide(Capture):
    def __init__(self):
        super().__init__('lidar_camera_calib_capture_guide')
        # Defaults assume a hand-held board: tolerant of hand shake and rough camera/LiDAR alignment.
        for k, v in dict(min_points=8, min_dist_m=.3, max_dist_m=1.6, seg_min_m=.06, seg_max_m=.50,
                         lateral_tol_m=.35, depth_tol_m=.45, still_px=6., novelty_normal_deg=6.,
                         novelty_dist_m=.10, target_poses=25, beep=True, stable_s=.3).items():
            self.declare_parameter(k, v)
        self.saved = load_saved(self.output)
        self.centres = []          # (monotonic, mean corner px) for stillness
        self.last_level = 'bad'

    def params(self):
        g = self.p
        return dict(min_points=g('min_points'), min_dist=g('min_dist_m'), max_dist=g('max_dist_m'),
                    seg_min=g('seg_min_m'), seg_max=g('seg_max_m'), lateral_tol=g('lateral_tol_m'),
                    depth_tol=g('depth_tol_m'), still_px=g('still_px'),
                    novelty_normal=g('novelty_normal_deg'), novelty_dist=g('novelty_dist_m'))

    def moving(self, data, now):
        if data['corners'] is None or len(data['corners']) == 0:
            self.centres.clear(); return 0.
        self.centres.append((now, data['corners'].reshape(-1, 2).mean(axis=0)))
        self.centres = [c for c in self.centres if now-c[0] < self.p('stable_s')]
        pts = np.array([c[1] for c in self.centres])
        # Not enough history yet counts as moving so a pose is only good after stable_s of stillness.
        if now-self.centres[0][0] < self.p('stable_s')*.8:
            return 99.
        return float(np.linalg.norm(pts.max(axis=0)-pts.min(axis=0)))

    def prepare(self):
        """Like Capture.prepare, but with a hand-held board: choose the board's cluster instead of requiring a single one."""
        d = super().prepare()
        if d['rvec'] is None or not any('candidate segments' in x or 'Too few points' in x for x in d['problems']):
            return d
        R = cv2.Rodrigues(d['rvec'])[0]
        centre = d['tvec'].ravel()+R@np.array([self.objp[:, 0].mean(), self.objp[:, 1].mean(), 0.])
        points, ids, _, _ = scan_points(d['smsg'])
        clusters = candidate_segments(points, ids, d['gate'], self.p('min_board_points'),
                                      self.p('cluster_gap_m'), self.p('max_line_rmse_m'))
        best, message = pick_board_segment(clusters, (centre[2], -centre[0]), self.params())
        d['problems'] = [x for x in d['problems'] if 'candidate segments' not in x and 'Too few points' not in x]
        if best is None:
            d['problems'].insert(0, message)
        else:
            d['selected'], d['selected_ids'] = best['points'], best['ids']
            d['line_error'] = best['rmse']
        return d

    def evaluate(self):
        import time
        data = self.prepare()
        data['objp_x'], data['objp_y'] = self.objp[:, 0], self.objp[:, 1]
        level, reasons, hints = assess(data, self.saved, self.moving(data, time.monotonic()), self.params())
        return data, level, reasons, hints

    def save(self, d):
        super().save(d)
        self.saved.append(pose_descriptor(cv2.Rodrigues(d['rvec'])[0][:, 2], d['tvec']))

    def banner(self, image, level, reasons, hints):
        color = dict(good=(0, 200, 0), similar=(0, 200, 255), bad=(0, 0, 220))[level]
        title = dict(good='CAPTURE NOW  (press C)', similar='OK but SIMILAR to saved poses - change pose',
                     bad='NOT READY')[level]
        h = image.shape[0]
        cv2.rectangle(image, (0, 0), (image.shape[1], 48), color, -1)
        cv2.putText(image, f'{title}   saved {len(self.saved)}/{self.p("target_poses")}', (12, 33), 0, .9, (255, 255, 255), 2)
        lines = reasons or (['Still needed: '+'; '.join(hints)] if hints else ['Poses look diverse; run solve once you have >= 20'])
        for i, line in enumerate(lines[:4]):
            y = 78+26*i
            cv2.putText(image, line[:110], (12, y), 0, .55, (0, 0, 0), 3)
            cv2.putText(image, line[:110], (12, y), 0, .55, (255, 255, 255), 1)

    def preview(self):
        if not self.p('show_preview'):
            return
        try:
            data, level, reasons, hints = self.evaluate()
            image = data['frame'].copy()
            self.banner(image, level, reasons, hints)
            if level == 'good' and self.last_level != 'good' and self.p('beep'):
                sys.stdout.write('\a'); sys.stdout.flush()
                self.get_logger().info('GOOD pose: press C to capture')
            self.last_level = level
            if level == 'bad':
                self.get_logger().info('NOT READY: '+' | '.join(reasons), throttle_duration_sec=2.)
            cv2.imshow('Capture guide', image)
            canvas = np.full((650, 650, 3), 255, np.uint8)
            scale = 300/max(.5, self.p('max_range_m'))
            for p, gated in zip(data['points'], data['gate']):
                xy = (int(325-p[1]*scale), int(610-p[0]*scale))
                if 0 <= xy[0] < 650 and 0 <= xy[1] < 650:
                    cv2.circle(canvas, xy, 2, (0, 180, 255) if gated else (170, 170, 170), -1)
            for p in data['selected']:
                cv2.circle(canvas, (int(325-p[1]*scale), int(610-p[0]*scale)), 4, (0, 150, 0), -1)
            cv2.putText(canvas, 'Top view: up=laser +X, left=+Y; green=selected', (8, 25), 0, .45, (0, 0, 0), 1)
            cv2.imshow('LiDAR ROI', canvas)
            self.current = (data, level)
        except ValueError as e:
            self.current = None
            self.last_level = 'bad'
            self.get_logger().warning(str(e), throttle_duration_sec=5.)
        key = cv2.waitKey(1) & 255
        if key == ord('c'):
            if self.current and self.current[1] != 'bad':
                try: self.save(self.current[0])
                except ValueError as e: self.get_logger().warning(str(e))
            else:
                self.get_logger().warning('Not saved: pose is not ready (see banner)')
        elif key == ord('q'):
            rclpy.shutdown()

    current = None


def main():
    rclpy.init(); node = None
    try:
        node = CaptureGuide(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
