import unittest
from pathlib import Path
import numpy as np
from body_clearance import BodyClearance
from model import Model

class BodyClearanceTest(unittest.TestCase):
    def test_boxes_include_collision_mesh_and_clearance_rejects_close_pose(self):
        root=Path(__file__).resolve().parents[2]/'src/arm_describle'
        guard=BodyClearance(root/'urdf/HRB3Q-LC.urdf',root)
        model=Model(root/'urdf/HRB3Q-LC.urdf',[1,1])
        self.assertEqual(len(guard.boxes),12)
        # 实机反馈曾出现的贴身姿态：两侧独立计算，不只检查TCP点。
        q=np.array([-.26,.97,.026,.015,-.20,-1.607,-1.166,1.643,
                    -.99,0.,-.04,.20,1.64,1.15,-1.66])
        for side in ('l','r'):
            distance,arm,body=guard.minimum(model,q,side)
            self.assertLess(distance,.04)
            self.assertTrue(arm.startswith('arm'+side))
            with self.assertRaises(ValueError):guard.check(model,q,side)
        with self.assertRaises(ValueError):BodyClearance(root/'urdf/HRB3Q-LC.urdf',root,.01)
