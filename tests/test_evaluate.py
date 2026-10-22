import tempfile
import unittest
from pathlib import Path
import numpy as np
import yaml
from lidar_camera_calib.evaluate import evaluate, metrics


class EvaluateTest(unittest.TestCase):
    def test_metrics(self):
        self.assertAlmostEqual(metrics(np.array([-.003,.004]))['rmse_m'],np.sqrt(12.5)*.001)

    def test_fixed_transform_and_overlap(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); data=root/'check'; data.mkdir(); train=root/'train';train.mkdir()
            cfg=dict(K=np.eye(3).ravel().tolist(),D=[0.]*5,board_cols=9,board_rows=6,square_size_m=.025,image_width=1280,image_height=720)
            camera=root/'camera.yaml'; camera.write_text(yaml.safe_dump(cfg))
            ext=root/'extrinsic.yaml'; T=np.eye(4);T[2,3]=1
            ext.write_text(yaml.safe_dump(dict(transform_4x4=T.tolist(),source_frame='laser',target_frame='optical')))
            before=ext.read_bytes()
            sample=dict(points_l=np.c_[np.linspace(0,.1,6),np.zeros((6,2))],normal_c=[0.,0.,1.],offset_c=-.99,
                source_frame='laser',target_frame='optical',K=np.eye(3),D=np.zeros(5),board_size=[9,6],square_size_m=.025,image_size=[1280,720],stamp_image=1.,stamp_scan=1.)
            np.savez(data/'sample_1.npz',**sample)
            report,_=evaluate(data,ext,camera,train)
            self.assertAlmostEqual(report['point_weighted']['rmse_m'],.01)
            self.assertEqual(before,ext.read_bytes())
            np.savez(train/'sample_1.npz',**sample)
            with self.assertRaisesRegex(ValueError,'overlap'): evaluate(data,ext,camera,train)
            sample['target_frame']='wrong';np.savez(data/'sample_1.npz',**sample)
            with self.assertRaisesRegex(ValueError,'Frame mismatch'): evaluate(data,ext,camera)
