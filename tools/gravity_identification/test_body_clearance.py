import unittest
from pathlib import Path
import numpy as np
from body_clearance import BodyClearance,mesh_tree,separated
from model import Model

class BodyClearanceTest(unittest.TestCase):
    def test_refinement_removes_empty_box_false_rejection(self):
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
            guard.check(model,q,side)
        with self.assertRaises(ValueError):BodyClearance(root/'urdf/HRB3Q-LC.urdf',root,.01)

    def test_refinement_still_rejects_near_surfaces(self):
        triangle=np.array([[[0.,0.,0.],[.1,0.,0.],[0.,.1,0.]]])
        tree=mesh_tree(triangle)
        self.assertFalse(separated(tree,tree,np.eye(3),np.array([0,0,.039]),.04))
        self.assertTrue(separated(tree,tree,np.eye(3),np.array([0,0,.041]),.04))
        self.assertFalse(separated(tree,tree,np.eye(3),np.zeros(3),.04))

    def test_disconnected_mesh_empty_space_is_not_solid(self):
        left=np.array([[[-.2,0,0],[-.2,.01,0],[-.2,0,.01]]])
        right=left+np.array([.4,0,0]);small=left+np.array([.2,0,0])
        self.assertTrue(separated(mesh_tree(small),mesh_tree(np.concatenate([left,right])),
                                  np.eye(3),np.zeros(3),.04))
