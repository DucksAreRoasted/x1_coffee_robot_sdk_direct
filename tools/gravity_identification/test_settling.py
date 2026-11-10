import unittest
from settling import stable_window

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
