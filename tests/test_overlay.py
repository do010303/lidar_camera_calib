import unittest
import numpy as np
from lidar_camera_calib.overlay import draw_scan, range_colors


class OverlayTest(unittest.TestCase):
    K = np.array([[100., 0, 160], [0, 100, 120], [0, 0, 1.]])
    # LiDAR (x fwd, y left, z up) -> camera optical (x right, y down, z fwd)
    R = np.array([[0, -1., 0], [0, 0, -1.], [1., 0, 0]])

    def scan(self):
        y = np.linspace(-.5, .5, 21)
        return np.c_[np.full_like(y, 2.), y, np.zeros_like(y)], np.arange(21)

    def test_points_land_on_expected_pixels(self):
        points, ids = self.scan()
        frame = np.zeros((240, 320, 3), np.uint8)
        out, n = draw_scan(frame, points, ids, self.R, np.zeros(3), self.K, np.zeros(5), alpha=1.)
        self.assertEqual(n, 21)
        # y=0 at 2 m -> image centre; y=+.5 (left) -> u = 160 - 100*.5/2 = 135
        self.assertGreater(out[120, 160].sum(), 0)
        self.assertGreater(out[120, 135].sum(), 0)
        self.assertEqual(out[0, 0].sum(), 0)

    def test_behind_camera_dropped(self):
        points, ids = self.scan(); points[:, 0] = -2.
        _, n = draw_scan(np.zeros((240, 320, 3), np.uint8), points, ids, self.R, np.zeros(3), self.K, np.zeros(5))
        self.assertEqual(n, 0)

    def test_height_draws_stripes(self):
        points, ids = self.scan()
        f = np.zeros((240, 320, 3), np.uint8)
        flat, _ = draw_scan(f, points, ids, self.R, np.zeros(3), self.K, np.zeros(5), alpha=1.)
        tall, _ = draw_scan(f, points, ids, self.R, np.zeros(3), self.K, np.zeros(5), height_m=.5, alpha=1.)
        self.assertGreater((tall.sum(2) > 0).sum(), (flat.sum(2) > 0).sum())

    def test_colors_near_blue_far_red(self):
        c = range_colors([0.2, 4.0], .2, 4.0)
        self.assertGreater(c[0][0], c[0][2]); self.assertGreater(c[1][2], c[1][0])


if __name__ == '__main__':
    unittest.main()


class LinkTest(unittest.TestCase):
    def test_no_long_link_between_near_points(self):
        K = OverlayTest.K; R = OverlayTest.R
        pts = np.array([[.2, .1, 0.], [.2, .17, 0.]])   # 7 cm apart, 20 cm away -> 35 px apart
        far = np.array([[.2, .05, 0.], [.2, .12, 0.]])
        f = np.zeros((240, 320, 3), np.uint8)
        linked, _ = draw_scan(f, pts, np.arange(2), R, np.zeros(3), K, np.zeros(5), alpha=1., radius=1, link_px=60)
        cut, _ = draw_scan(f, pts, np.arange(2), R, np.zeros(3), K, np.zeros(5), alpha=1., radius=1, link_px=10)
        self.assertGreater((linked.sum(2) > 0).sum(), (cut.sum(2) > 0).sum())


class LabelTest(unittest.TestCase):
    def test_distance_labels_drawn(self):
        t = OverlayTest(); points, ids = t.scan()
        f = np.zeros((240, 320, 3), np.uint8)
        plain, _ = draw_scan(f, points, ids, t.R, np.zeros(3), t.K, np.zeros(5), alpha=1., label_gap_px=0)
        lab, _ = draw_scan(f, points, ids, t.R, np.zeros(3), t.K, np.zeros(5), alpha=1.)
        self.assertGreater((lab.sum(2) > 0).sum(), (plain.sum(2) > 0).sum())


class ScaleTest(unittest.TestCase):
    def test_scaled_camera_projects_to_scaled_pixels(self):
        import cv2
        from lidar_camera_calib.overlay import scaled_camera
        K = np.array([[736., 0, 614.5], [0, 736., 359.8], [0, 0, 1.]]); D = np.array([-.0028, -.067, 0, 0, 0])
        pt = np.array([[.3, -.2, 2.]])
        full = cv2.projectPoints(pt, np.zeros(3), np.zeros(3), K, D)[0].ravel()
        half = cv2.projectPoints(pt, np.zeros(3), np.zeros(3), scaled_camera(K, .5), D)[0].ravel()
        # cv2.resize maps source pixel centre (u+.5)*s-.5
        np.testing.assert_allclose(half, (full+.5)*.5-.5, atol=1e-6)

    def test_put_lines_fits_width(self):
        from lidar_camera_calib.overlay import put_lines
        img = np.zeros((360, 640, 3), np.uint8)
        put_lines(img, ['x'*200], 4, 1.)
        self.assertEqual(img[:, 636:].sum(), 0)
