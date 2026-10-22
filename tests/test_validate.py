import unittest
import numpy as np
from lidar_camera_calib.validate import draw_validation


class ValidateTest(unittest.TestCase):
    def sample(self):
        points=np.array([[0.,0.,0.],[.05,0.,0.]])
        return dict(frame=np.zeros((240,320,3),np.uint8),points=points,
            selected=points,dt=.02,problems=[],rvec=np.zeros(3),tvec=np.array([0.,0.,1.01]))

    def test_roi_distance_and_highlight(self):
        d=self.sample(); K=np.array([[100.,0,160],[0,100,120],[0,0,1.]])
        frame,stats=draw_validation(d,np.eye(3),np.array([0.,0.,1.]),K,np.zeros(5))
        self.assertAlmostEqual(stats['rmse_mm'],10.)
        self.assertAlmostEqual(stats['bias_mm'],-10.)
        np.testing.assert_array_equal(frame[120,160],[0,255,0])
        d['problems']=['bad board pose']
        _,stats=draw_validation(d,np.eye(3),np.array([0.,0.,1.]),K,np.zeros(5))
        self.assertIsNone(stats)

    def test_comparison_uses_single_pair(self):
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        from lidar_camera_calib.validate import Validate
        data=self.sample()
        data.update(imsg=SimpleNamespace(header=SimpleNamespace(frame_id='cam')),
                    smsg=SimpleNamespace(header=SimpleNamespace(frame_id='laser')))
        parameters=dict(extrinsic_yaml='original.yaml',compare_extrinsic_yaml='half.yaml',show_preview=True)
        comparison=np.eye(4);comparison[2,3]=.5
        logger=Mock()
        node=SimpleNamespace(prepare=Mock(return_value=data),extrinsic=dict(source_frame='laser',target_frame='cam'),
            R=np.eye(3),t=np.array([0.,0.,1.]),K=np.array([[100.,0,160],[0,100,120],[0,0,1.]]),D=np.zeros(5),
            comparison=comparison,p=lambda k:parameters[k],get_logger=lambda:logger)
        with patch('lidar_camera_calib.validate.cv2.imshow') as show, patch('lidar_camera_calib.validate.cv2.waitKey',return_value=-1):
            Validate.preview(node)
        node.prepare.assert_called_once()
        self.assertEqual(show.call_args.args[1].shape,(240,640,3))
        self.assertIn('A RMSE=10.0 mm; B RMSE=510.0 mm',logger.info.call_args.args[0])
