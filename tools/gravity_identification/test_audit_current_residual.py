import unittest
from pathlib import Path
import numpy as np
from model import Model
from current_identify import features
from audit_current_residual import audit,solve

class ResidualAuditTest(unittest.TestCase):
    def test_known_gravity_and_repeats_are_diagnostic_only(self):
        model=Model(Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf',[1,1])
        rng=np.random.default_rng(52);q=rng.uniform(-.6,.6,(25,15));current=[]
        for value in q:
            y,k,prior,_=model.regressor(value,'r');current.append(((y@prior+k)*20).tolist())
        candidate=dict(side='r',sample_poses=q.tolist(),observed_current_permille=current)
        repeat=dict(poses=[dict(pose='pose01',pairs=[dict(repeat=1,mean_current=current[0],
                    windows=[dict(q_median=q[0].tolist()),dict(q_median=q[0].tolist())])])])
        report=audit(model,candidate,repeat)
        self.assertLess(report['models']['gravity']['cross_validation_rms_permille'],1.)
        self.assertFalse(report['deployment_allowed']);self.assertFalse(report['runtime_applied'])
        self.assertEqual(len(report['nested_selected_models']),5)
        self.assertEqual(len(report['models']['gravity']['repeat_checks']),1)

    def test_solver_handles_zero_columns_without_nonfinite_coefficients(self):
        x=np.c_[np.arange(10.),np.zeros(10),np.ones(10)]
        b=solve(x,2*x[:,0]+3)
        self.assertTrue(np.isfinite(b).all())
        self.assertLess(np.max(abs(x@b-(2*x[:,0]+3))),.1)
