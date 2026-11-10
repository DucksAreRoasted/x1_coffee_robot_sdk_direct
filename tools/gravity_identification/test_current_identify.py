import unittest
from pathlib import Path
import numpy as np
from model import Model,JOINTS
from current_identify import features,fit_current,current_pairs


class CurrentIdentificationTest(unittest.TestCase):
    def test_coupled_synthetic_current_predicts_unseen_poses(self):
        model=Model(Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf',[1,1])
        rng=np.random.default_rng(42)
        for side,offset in [('l',1),('r',8)]:
            poses=[]
            for i in range(45):
                q=rng.uniform(-.7,.7,15);q[0]=0.
                # 合成观测来自各关节不同的电流/力矩比例，不强迫全臂共享比例。
                y,k,p,_=model.regressor(q,side)
                t=np.zeros(15);t[offset:offset+7]=(y@p+k)*np.linspace(2,8,7)
                poses.append((str(i),q,q,t))
            r=fit_current(model,poses,side,2.)
            self.assertTrue(r['accepted_offline'],r['cross_validation_rms_permille'])
            self.assertFalse(r['deployment_allowed'])
            self.assertEqual(np.shape(r['coefficients']),(7,29))
            self.assertEqual(len(r['validation_predictions']),45)
            # 独立新姿态复核最终模型，不能只检查训练集输出。
            q=rng.uniform(-.6,.6,15);q[0]=0.
            y,k,p,_=model.regressor(q,side)
            pred=np.einsum('jk,jk->j',features(model,q,side),np.array(r['coefficients']))
            np.testing.assert_allclose(pred,(y@p+k)*np.linspace(2,8,7),atol=2.)

    def test_raw_current_used_and_legacy_explicitly_diagnostic(self):
        cfg=dict(verified_sides=['r'],joints={j:dict(sign=1,rated_nm=10) for j in JOINTS})
        rows=[]
        for pose in range(15):
            for direction in [-1,1]:
                for n in range(31):
                    r=dict(side='r',pose_id=str(pose),approach=direction,time_ns=n*100000000,
                           valid=1,conversion_sha256='fixture')
                    for j in JOINTS:
                        r['q_'+j]=0.;r['dq_'+j]=0.;r['tau_'+j]=999.
                        r['raw_current_permille_'+j]=20.;r['raw_torque_permille_'+j]=20.
                        r['raw_current_valid_'+j]=1;r['raw_torque_valid_'+j]=1
                    rows.append(r)
        poses,reasons=current_pairs(rows,cfg,'r')
        self.assertEqual(reasons,[])
        self.assertEqual(poses[0][3][8],20.)
        for r in rows:
            for j in JOINTS:r.pop('raw_current_permille_'+j)
        with self.assertRaises(ValueError):current_pairs(rows,cfg,'r')
        _,reasons=current_pairs(rows,cfg,'r',True)
        self.assertTrue(reasons)
