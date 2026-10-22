"""Synthetic checks; these do not certify physical calibration accuracy."""
import unittest
from types import SimpleNamespace
import cv2
import numpy as np
from lidar_camera_calib.geometry import select_segment, project, scan_points
from lidar_camera_calib.solve import linear_initial, optimize, residual


def synthetic():
    rng=np.random.default_rng(41)
    truth=np.array([.3,-.2,.1,.08,-.03,2.])
    R=cv2.Rodrigues(truth[:3])[0]; t=truth[3:]; samples=[]
    for _ in range(30):
        n=rng.normal(size=3); n/=np.linalg.norm(n)
        nl=R.T@n; direction=np.array([-nl[1],nl[0],0]); direction/=np.linalg.norm(direction)
        center=np.r_[rng.uniform(-.7,.7,2),0.]
        points=center+np.linspace(-.12,.12,25)[:,None]*direction
        samples.append(dict(points_l=points,normal_c=n,offset_c=-n@(R@center+t)))
    return truth,samples


class GeometryTest(unittest.TestCase):
    def test_recover_transform(self):
        truth,samples=synthetic()
        fit,_,rank,_=optimize(samples,linear_initial(samples))
        self.assertEqual(rank,6)
        np.testing.assert_allclose(fit.x,truth,atol=1e-6)
        rng=np.random.default_rng(4)
        noisy=[dict(s,points_l=s['points_l']+np.c_[rng.normal(0,.001,(25,2)),np.zeros(25)]) for s in samples]
        fit,_,rank,_=optimize(noisy,linear_initial(noisy))
        self.assertEqual(rank,6)
        np.testing.assert_allclose(fit.x,truth,atol=.01)

    def test_degenerate(self):
        _,samples=synthetic(); samples=[samples[0]]*20
        with self.assertRaises(ValueError): linear_initial(samples)
        _,_,rank,_=optimize(samples,np.zeros(6))
        self.assertLess(rank,6)

    def test_segment_ambiguity(self):
        p=np.c_[np.ones(16),np.arange(16)*.01,np.zeros(16)]
        ids=np.r_[np.arange(8),np.arange(20,28)]
        with self.assertRaises(ValueError): select_segment(p,ids,np.ones(16,bool))
        selected,_,error=select_segment(p[:8],ids[:8],np.ones(8,bool))
        self.assertEqual(len(selected),8); self.assertLess(error,1e-9)

    def test_distortion_and_depth(self):
        K=np.array([[500.,0,640],[0,500,360],[0,0,1]])
        points=np.array([[.5,.2,1],[0,0,-1],[100,0,1]])
        D=np.array([.2,-.05,0,0,0])
        uv,ids=project(points,np.eye(3),np.zeros(3),K,D,1280,720)
        np.testing.assert_array_equal(ids,[0])
        expected=cv2.projectPoints(points[:1],np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
        np.testing.assert_allclose(uv,expected)
        self.assertGreater(abs(uv[0,0]-890),1)

    def test_scan_invalid_ranges(self):
        scan=SimpleNamespace(ranges=[float('nan'),float('inf'),.01,1.,5.],angle_min=-.2,angle_increment=.1,range_min=.1,range_max=3.)
        p,ids,_,_=scan_points(scan)
        np.testing.assert_array_equal(ids,[3]);self.assertEqual(p.shape,(1,3))


if __name__=='__main__': unittest.main()
