import copy
import hashlib
import json
import unittest
from types import SimpleNamespace
from export_current import arm_record,fingerprint,arm_options
from model import JOINTS


class ExportCurrentTest(unittest.TestCase):
    def setUp(self):
        self.cfg=dict(verified_sides=['r'],joints={j:dict(sign=1,rated_nm=10) for j in JOINTS})
        self.raw=json.dumps(self.cfg).encode();self.urdf=b'test-urdf'
        self.c=dict(schema='x1_current_gravity_v1',side='r',accepted_offline=True,rejection_reasons=[],
            unit='joint_signed_current_permille',basis='urdf_mass_first_moment_28_plus_known_gravity',
            gripper_kg=[1,1],urdf_sha256=hashlib.sha256(self.urdf).hexdigest(),
            conversion_sha256=hashlib.sha256(self.raw).hexdigest(),coefficients=[[0.]*29 for _ in range(7)],
            sample_poses=[[0.]*15 for _ in range(15)],cross_validation_rms_permille=[1.]*7)
    def export(self,c):
        return arm_record(c,self.cfg,self.urdf,self.raw,kd=[20]*7,limits=[100]*7,slew=[500]*7,radius=.03)
    def test_export_and_fingerprint(self):
        self.assertEqual(self.export(self.c)['side'],'r')
        self.assertEqual(fingerprint(b'hello'),0xa430d84680aabd0b)
    def test_rejects_incompatible_or_failed_candidate(self):
        for field,value in [('accepted_offline',False),('unit','Nm'),('gripper_kg',[0,0]),
                            ('urdf_sha256','wrong'),('conversion_sha256','wrong'),
                            ('rejection_reasons',['legacy']),('cross_validation_rms_permille',[11]*7)]:
            c=copy.deepcopy(self.c);c[field]=value
            with self.assertRaises(ValueError):self.export(c)
        c=copy.deepcopy(self.c);c['coefficients'][0][0]=float('nan')
        with self.assertRaises(ValueError):self.export(c)


class ArmOptionsTest(unittest.TestCase):
    def options(self):
        return SimpleNamespace(kd=None,limits=None,slew=None,
            left_kd=[1]*7,left_limits=[20]*7,left_slew=[30]*7,
            right_kd=[2]*7,right_limits=[40]*7,right_slew=[60]*7)

    def test_dual_independent_budgets(self):
        args=self.options()
        self.assertEqual(arm_options(args,'l',True)['limits'],[20]*7)
        self.assertEqual(arm_options(args,'r',True)['limits'],[40]*7)

    def test_dual_never_inherits_shared_budget(self):
        args=self.options();args.kd=[3]*7
        with self.assertRaises(ValueError):arm_options(args,'l',True)
        args=self.options();args.right_slew=None
        with self.assertRaises(ValueError):arm_options(args,'r',True)

    def test_single_legacy_options(self):
        args=self.options()
        args.right_kd=args.right_limits=args.right_slew=None
        args.kd=[2]*7;args.limits=[40]*7;args.slew=[60]*7
        self.assertEqual(arm_options(args,'r',False)['kd'],[2]*7)
