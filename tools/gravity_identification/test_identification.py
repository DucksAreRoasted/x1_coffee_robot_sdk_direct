import unittest
from pathlib import Path
import numpy as np
from model import Model,JOINTS
from identify import fit,paired
from drag import torque

URDF=Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf'

class IdentificationTest(unittest.TestCase):
    def setUp(self): self.model=Model(URDF,[1.,1.])

    def test_gravity_matches_potential_gradient_both_arms(self):
        q=np.linspace(-.3,.3,15)
        def potential(q):
            frames,_=self.model.geometry(q)
            return sum(m*9.80665*(frames[n][0]+frames[n][1]@c)[2] for n,(m,c,_) in self.model.links.items())
        for side,offset in [('l',1),('r',8)]:
            y,k,prior,inertia=self.model.regressor(q,side)
            self.assertTrue(np.all(inertia>0))
            for j in range(7):
                a=q.copy();b=q.copy();a[j+offset]+=1e-6;b[j+offset]-=1e-6
                self.assertAlmostEqual((y@prior+k)[j],(potential(a)-potential(b))/2e-6,places=5)

    def test_synthetic_identification_and_holdout(self):
        rng=np.random.default_rng(7)
        for side,offset in [('l',1),('r',8)]:
            poses=[]
            for n in range(25):
                q=rng.uniform(-.6,.6,15);q[0]=0
                y,k,prior,_=self.model.regressor(q,side)
                tau=np.zeros(15);tau[offset:offset+7]=y@(prior*1.1)+k
                poses.append((str(n),q,q,tau))
            result=fit(self.model,poses,side,.1)
            self.assertTrue(result['accepted_offline'],result)
            self.assertEqual(len(result['validation_pose_ids']),5)
            self.assertLess(max(result['validation_rms_nm']),.1)
            self.assertEqual(result['drag_kp_nm_rad'],[0.]*7)

    def test_matches_existing_cpp_model_golden_values(self):
        q=np.zeros(15)
        for side,expected in [('l',[-16.62,16.51]),('r',[16.63,-16.52])]:
            y,k,p,_=self.model.regressor(q,side);tau=y@p+k
            np.testing.assert_allclose(tau[[0,3]],expected,atol=.08)
        q[0]=.5;q[2]=.8;q[4]=-.6
        y,k,p,_=self.model.regressor(q,'l')
        np.testing.assert_allclose((y@p+k)[[1,2,5]],[17.11,-9.78,4.31],atol=.08)

    def test_pairing_accepts_in_memory_integer_valid(self):
        rows=[]
        for pose in range(15):
            for direction in (-1,1):
                for sample in range(20):
                    row=dict(side='r',pose_id=str(pose),approach=direction,
                             conversion_sha256='fixture',valid=1,time_ns=sample*30000000)
                    for j in JOINTS:
                        row['q_'+j]=0.;row['dq_'+j]=0.;row['tau_'+j]=0.
                    rows.append(row)
        self.assertEqual(len(paired(rows,'r')),15)
        for row in rows: row['valid']=0
        with self.assertRaises(ValueError):paired(rows,'r')

    def test_pairing_cancels_opposite_static_friction_and_rejects_duplicates(self):
        rows=[]
        for pose in range(15):
            for direction in [-1,1]:
                for sample in range(20):
                    row={'side':'l','pose_id':str(pose),'approach':str(direction),
                         'conversion_sha256':'verified_fixture','valid':'1',
                         'time_ns':str((pose*2+(direction+1)//2)*1000000000+sample*30000000)}
                    for j in JOINTS:
                        row['q_'+j]='0';row['dq_'+j]='0';row['tau_'+j]=str(2+direction*.5)
                    rows.append(row)
        result=paired(rows,'l')
        self.assertEqual(len(result),15)
        np.testing.assert_allclose(result[0][3],2.)
        with self.assertRaises(ValueError): paired(rows+rows,'l')
        rows[0]['tau_Joint_l1']='nan'
        with self.assertRaises(ValueError): paired(rows,'l')

    def test_missing_pairs_rejected(self):
        with self.assertRaises(ValueError): paired([], 'l')

    def test_unexcited_data_rejected(self):
        q=np.zeros(15);tau=np.zeros(15)
        y,k,p,_=self.model.regressor(q,'r');tau[8:]=y@p+k
        result=fit(self.model,[(str(i),q,q,tau) for i in range(20)],'r',.1)
        self.assertFalse(result['accepted_offline'])

    def test_drag_damping_budget_and_slew(self):
        self.assertAlmostEqual(torque(2,1,2,.001,kd=1,limit_nm=5,slew_nm_s=10),1.99)
        with self.assertRaises(ValueError):torque(2,0,0,.001,kd=1,limit_nm=5,slew_nm_s=10,fc=1,friction_gain=.5)
        with self.assertRaises(ValueError):torque(6,0,0,.001,kd=1,limit_nm=5,slew_nm_s=10)
        with self.assertRaises(ValueError):torque(float('nan'),0,0,.001,kd=1,limit_nm=5,slew_nm_s=10)

if __name__=='__main__':unittest.main()
