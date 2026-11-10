import unittest
from sample_quality import quality
from model import JOINTS


class QualityTest(unittest.TestCase):
    def test_stationary_position_can_have_drifting_torque(self):
        rows=[]
        for n in range(20):
            r=dict(pose_id='p', approach=1, time_ns=n)
            for j in JOINTS[8:]:
                r['q_'+j]=0.;r['tau_'+j]=float(n)
            rows.append(r)
        result=quality(list(reversed(rows)))[0]['joints'][0]
        self.assertEqual(result['q_span_rad'],0)
        self.assertEqual(result['legacy_tau_end_minus_start'],10)
        with self.assertRaises(ValueError):quality(rows[:10])
