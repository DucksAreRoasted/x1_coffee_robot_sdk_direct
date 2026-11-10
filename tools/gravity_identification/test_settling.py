import unittest
from settling import stable_window,stability_reason,StableConfirmation

class SettlingTest(unittest.TestCase):
    def rows(self):
        return [dict(time_ns=i*100000000,raw_current_valid_j=1,raw_torque_valid_j=1,
                     raw_current_permille_j=50,raw_torque_permille_j=50) for i in range(31)]
    def test_constant_and_duration(self):
        self.assertTrue(stable_window(self.rows(),['j']))
        self.assertFalse(stable_window(self.rows()[:-1],['j']))
    def test_drift_noise_and_invalid(self):
        for kind in ['current','torque']:
            r=self.rows()
            for i,row in enumerate(r):row['raw_'+kind+'_permille_j']=i*2
            self.assertFalse(stable_window(r,['j']))
            r=self.rows()
            for i,row in enumerate(r):row['raw_'+kind+'_permille_j']=50+(-1)**i*25
            self.assertFalse(stable_window(r,['j']))
            r=self.rows();r[5]['raw_'+kind+'_valid_j']=0
            self.assertFalse(stable_window(r,['j']))

    def test_reason_distinguishes_accumulation_and_joint_drift(self):
        self.assertIn('正常积累窗口',stability_reason(self.rows()[:-1],['j']))
        rows=self.rows()
        for i,row in enumerate(rows):row['raw_current_permille_j']=i*2
        reason=stability_reason(rows,['j'])
        self.assertIn('j 电流前后均值漂移',reason)
        self.assertIn('> 10‰',reason)
        self.assertEqual(stability_reason(self.rows(),['j']),'')

    def test_confirmation_requires_uninterrupted_passes(self):
        gate=StableConfirmation()
        self.assertFalse(gate.update(10,True))
        self.assertFalse(gate.update(12.9,True))
        self.assertFalse(gate.update(13,False))
        self.assertFalse(gate.update(14,True))
        self.assertFalse(gate.update(16.9,True))
        self.assertTrue(gate.update(17,True))
