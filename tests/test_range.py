import unittest
from types import SimpleNamespace
import numpy as np
from lidar_camera_calib.check_range import forward_distances


class RangeTest(unittest.TestCase):
    def test_perpendicular_plane_and_wrap(self):
        angles=np.deg2rad(np.arange(358,363))
        scan=SimpleNamespace(ranges=.5/np.cos(angles),angle_min=angles[0],angle_increment=np.deg2rad(1),range_min=.1,range_max=3.)
        values=forward_distances(scan,2.1)
        self.assertEqual(len(values),5)
        np.testing.assert_allclose(values,.5,atol=1e-12)
        scan.ranges=[np.nan]*5
        self.assertEqual(len(forward_distances(scan,2.1)),0)
