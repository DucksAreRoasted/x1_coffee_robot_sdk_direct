import copy
import json
import hashlib
import io
import unittest
from pathlib import Path
import numpy as np
from model import Model
from current_identify import features
from physical_current_compile import compile_coefficients

class PhysicalCompileTest(unittest.TestCase):
    def setUp(self):
        self.path=Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf'
        self.model=Model(self.path,[1,1])

    def candidate(self,side,payload):
        _,_,prior,_=self.model.regressor(np.zeros(15),side)
        return dict(schema='x1_physical_current_candidate_v1',accepted_offline=True,rejection_reasons=[],
            side=side,payload_semantics='additional_to_urdf_link_inertials_at_tcp',
            bound_hits=[],fold_bound_hits=[[] for _ in range(5)],mass_scale=[1.]*7,com_delta_m=[[0.]*3 for _ in range(7)],
            nominal_current_per_nm=[10.]*7,identified_current_per_nm=[10.]*7,
            shared_inertial_parameters=prior.tolist(),additional_gripper_kg=payload)

    def test_no_double_counting_and_nonzero_extra_payload_both_arms(self):
        rng=np.random.default_rng(12)
        for payload in ([0,0],[.4,.7]):
            physical=Model(self.path,payload)
            for side in ('l','r'):
                beta=compile_coefficients(self.candidate(side,payload),self.model)
                for _ in range(12):
                    q=rng.uniform(-1,1,15)
                    y,k,p,_=physical.regressor(q,side)
                    expected=(y@p+k)*10
                    actual=np.einsum('jk,jk->j',features(self.model,q,side),beta)
                    np.testing.assert_allclose(actual,expected,atol=1e-10)

    def test_rejects_inconsistent_physics_and_payload_semantics(self):
        original=self.candidate('r',[0,0])
        for field,value in [('payload_semantics','total_gripper_mass'),('bound_hits',[27]),
                            ('mass_scale',[2.]*7),('identified_current_per_nm',[-1.]*7),
                            ('shared_inertial_parameters',[0.]*28)]:
            c=copy.deepcopy(original);c[field]=value
            with self.assertRaises(ValueError):compile_coefficients(c,self.model)

    def test_extra_fixed_mass_cannot_be_scaled_as_gripper(self):
        model=Model(self.path,[0,0])
        with self.assertRaisesRegex(ValueError,'单位点质量'):
            compile_coefficients(self.candidate('r',[0,0]),model)

    def test_export_accepts_physical_schema_without_extra_tcp_mass(self):
        from export_current import arm_record
        from model import JOINTS
        candidate=self.candidate('r',[0,0]);raw=self.path.read_bytes()
        cfg=dict(verified_sides=['r'],joints={j:dict(sign=1,rated_nm=10) for j in JOINTS})
        config=json.dumps(cfg).encode()
        candidate.update(urdf_sha256=hashlib.sha256(raw).hexdigest(),
            conversion_sha256=hashlib.sha256(config).hexdigest(),
            sample_poses=[[0.]*15]*30,cross_validation_rms_permille=[0.]*7)
        result=arm_record(candidate,cfg,raw,config,kd=[10]*7,limits=[100]*7,slew=[50]*7,radius=.03)
        np.testing.assert_array_equal(result['beta'][:,28],0)
        self.assertGreater(np.linalg.norm(result['beta'][:,:28]),0)
