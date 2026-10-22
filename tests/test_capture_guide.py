import unittest
import numpy as np
from lidar_camera_calib.capture_guide import assess, coverage, pose_descriptor

P = dict(min_points=12, min_dist=.4, max_dist=1.4, seg_min=.08, seg_max=.40, lateral_tol=.25,
         depth_tol=.35, still_px=2., novelty_normal=8., novelty_dist=.12)


def data(y=0., x=.6, n=20, half=.1, rvec=(0., 0., 0.), z=.6, problems=()):
    ys = np.linspace(y-half, y+half, n)
    objp = np.mgrid[0:9, 0:6].T.reshape(-1, 2)*.025
    return dict(problems=list(problems), selected=np.c_[np.full(n, x), ys, np.zeros(n)],
                rvec=np.array(rvec), tvec=np.array([0., 0., z]), objp_x=objp[:, 0], objp_y=objp[:, 1])


class GuideTest(unittest.TestCase):
    def test_good_when_consistent_and_novel(self):
        level, reasons, _ = assess(data(y=-.1), [], 0., P)
        self.assertEqual((level, reasons), ('good', []))

    def test_similar_to_saved(self):
        saved = [pose_descriptor(np.array([0., 0., 1.]), [0, 0, .6])]
        self.assertEqual(assess(data(y=-.1), saved, 0., P)[0], 'similar')

    def test_wrong_object_rejected(self):
        # wall 1 m to the left in LiDAR while board is centred in camera
        level, reasons, _ = assess(data(y=1.0, half=.6), [], 0., P)
        self.assertEqual(level, 'bad')
        self.assertTrue(any('wrong object' in r for r in reasons))
        self.assertTrue(any('does not match' in r for r in reasons))

    def test_moving_and_far_and_few_points(self):
        self.assertEqual(assess(data(y=-.1), [], 5., P)[0], 'bad')
        self.assertEqual(assess(data(y=-.1, z=2., x=2.), [], 0., P)[0], 'bad')
        self.assertEqual(assess(data(y=-.1, n=5), [], 0., P)[0], 'bad')

    def test_detection_problem_passthrough(self):
        level, reasons, _ = assess(data(problems=['Checkerboard not detected']), [], 0., P)
        self.assertEqual((level, reasons), ('bad', ['Checkerboard not detected']))

    def test_coverage_lists_missing(self):
        self.assertEqual(len(coverage([])), 7)
        full = [pose_descriptor(np.array(n), [0, 0, z]) for n, z in
                (((.4, 0, 0), .5), ((-.4, 0, 0), .7), ((0, .4, 0), 1.), ((0, -.4, 0), 1.))]
        self.assertEqual(coverage(full), [])


if __name__ == '__main__':
    unittest.main()


class PickTest(unittest.TestCase):
    def cluster(self, x, y0, y1, n, rmse=.002):
        ys = np.linspace(y0, y1, n)
        return dict(points=np.c_[np.full(n, x), ys, np.zeros(n)], ids=np.arange(n), rmse=rmse, straight=rmse <= .008)

    def test_board_chosen_over_hand_and_background(self):
        from lidar_camera_calib.capture_guide import pick_board_segment
        board = self.cluster(.6, -.12, .1, 26)
        hand = self.cluster(.62, .12, .22, 8)           # too short
        wall = self.cluster(1.2, -.1, .2, 14)           # right shape but far from expected depth
        best, msg = pick_board_segment([hand, board, wall], (.6, 0.), P)
        self.assertIs(best, board)

    def test_ambiguous_and_none(self):
        from lidar_camera_calib.capture_guide import pick_board_segment
        a, b = self.cluster(.6, -.12, .1, 26), self.cluster(.65, .2, .38, 22)
        self.assertIsNone(pick_board_segment([a, b], (.6, .05), P)[0])
        best, msg = pick_board_segment([self.cluster(.6, -.1, .1, 20, rmse=.02)], (.6, 0.), P)
        self.assertIsNone(best); self.assertIn('no LiDAR segment', msg)
