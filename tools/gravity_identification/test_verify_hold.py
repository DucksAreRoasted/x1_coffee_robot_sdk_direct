import copy
import unittest
from verify_hold import check_sample,matching_candidate,summarize_trace

class HoldVerificationTest(unittest.TestCase):
    def sample(self):return dict(healthy=True,q=[0.]*15,dq=[0.]*15,current=[5.]*15)
    def test_unhealthy_nonfinite_and_wrong_shape_fail(self):
        for key,value in [('healthy',False),('q',[float('nan')]*15),('dq',[0.]*14),('current',[float('inf')]*15)]:
            sample=self.sample();sample[key]=value
            with self.assertRaises(RuntimeError):check_sample(sample,[0.]*15)
    def test_drift_and_speed_report_axis(self):
        sample=self.sample();sample['q'][11]=.021
        with self.assertRaisesRegex(RuntimeError,'Joint_r4 漂移'):check_sample(sample,[0.]*15)
        sample=self.sample();sample['dq'][13]=.06
        with self.assertRaisesRegex(RuntimeError,'Joint_r6 速度'):check_sample(sample,[0.]*15)
        check_sample(self.sample(),[0.]*15)
    def test_model_identity_and_single_arm(self):
        digest='a'*64
        self.assertTrue(matching_candidate('已加载 candidate_r='+digest,digest,'r'))
        self.assertFalse(matching_candidate('已加载',digest,'r'))
        self.assertFalse(matching_candidate('candidate_r='+'b'*64,digest,'r'))
        self.assertFalse(matching_candidate('candidate_r='+digest+' candidate_l='+digest,digest,'r'))

    def test_measurement_metrics_and_gap_rejection(self):
        rows=[]
        for i in range(4):
            sample=self.sample();sample.update(stage='holding',received_monotonic=i*.1)
            sample['q'][8]=i*.001;rows.append(sample)
        report=summarize_trace(rows,[0.]*15)
        self.assertAlmostEqual(report['max_drift_rad'][8],.003)
        self.assertEqual(report['sample_count'],4)
        rows[-1]['received_monotonic']=1.
        with self.assertRaises(ValueError):summarize_trace(rows,[0.]*15)

    def test_duplicate_or_backward_measurement_time_rejected(self):
        for times in ([0.,0.], [1.,.9], [0.,float('nan')]):
            rows=[]
            for t in times:
                row=self.sample();row.update(stage='holding',received_monotonic=t);rows.append(row)
            with self.assertRaises(ValueError):summarize_trace(rows,[0.]*15)
