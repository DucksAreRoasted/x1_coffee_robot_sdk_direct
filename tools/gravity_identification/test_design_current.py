import unittest
from unittest.mock import patch
import numpy as np
from design_current import design

class DesignTest(unittest.TestCase):
    def candidate(self):
        beta=np.zeros((7,29));beta[:,0]=1
        return dict(side='r',accepted_offline=True,rejection_reasons=[],
            coefficients=beta.tolist(),sample_poses=[[0]*15]*15,
            observed_current_permille=[[20]*7]*15,max_abs_validation_error_permille=[5]*7)

    @patch('design_current.features',return_value=np.full((7,29),10.))
    def test_headroom_and_units(self,_):
        r=design(self.candidate(),None,[100]*7,[.2]*7,.5,2.)
        np.testing.assert_allclose(r['kd_permille_s_rad'],187.5)
        np.testing.assert_allclose(r['slew_permille_s'],50.)
        np.testing.assert_allclose(r['sampled_peak_permille'],20.)
        self.assertFalse(r['deployment_allowed'])

    @patch('design_current.features',return_value=np.full((7,29),30.))
    def test_model_peak_also_consumes_budget(self,_):
        with self.assertRaisesRegex(ValueError,'预算不足'):
            design(self.candidate(),None,[35]*7,[.2]*7,.5,2.)

    def test_invalid_inputs(self):
        for speeds,fraction,ramp in [([0]*7,.5,1),([.2]*7,1,1),([.2]*7,.5,0)]:
            with self.assertRaises(ValueError):
                design(self.candidate(),None,[100]*7,speeds,fraction,ramp)
        c=self.candidate();c['accepted_offline']=False
        with self.assertRaises(ValueError):design(c,None,[100]*7,[.2]*7,.5,1)
