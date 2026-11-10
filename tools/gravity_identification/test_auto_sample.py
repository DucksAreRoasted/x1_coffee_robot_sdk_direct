import unittest
from pathlib import Path
import numpy as np
from model import Model
from auto_sample import make_plan,path_points

class PlanTest(unittest.TestCase):
    def setUp(self):
        self.model=Model(Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf',[1,1])
    def test_pairs_and_waist_left_fixed(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+float(self.model.by_name[j].find('limit').get('upper')))/2 for j in __import__('model').JOINTS]);plan=make_plan(seed,self.model)
        self.assertEqual(len(plan),60)
        for i in range(0,60,4):
            np.testing.assert_allclose(plan[i+1]['q'],plan[i+3]['q'])
            self.assertEqual(plan[i+1]['approach'],1)
            self.assertEqual(plan[i+3]['approach'],-1)
        for step in plan:np.testing.assert_allclose(step['q'][:8],seed[:8])
    def test_limits_and_nonfinite_refused(self):
        for seed in [np.full(15,np.nan),np.full(15,100)]:
            with self.assertRaises(ValueError):make_plan(seed,self.model)
    def test_segment_includes_endpoints_and_spacing(self):
        p=path_points(np.zeros(15),np.ones(15)*.127)
        np.testing.assert_allclose(p[0],0);np.testing.assert_allclose(p[-1],.127)
        self.assertLessEqual(np.max(np.abs(np.diff(p,axis=0))),.01)

if __name__=='__main__':unittest.main()
