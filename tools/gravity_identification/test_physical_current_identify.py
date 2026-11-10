import unittest
from pathlib import Path
import numpy as np
from model import Model
from physical_current_identify import fit_physical

class PhysicalCurrentTest(unittest.TestCase):
    def setUp(self):
        self.model=Model(Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf',[1,1])
        self.gain=np.linspace(3,9,7)

    def poses(self,side):
        rng=np.random.default_rng(38);poses=[]
        slot=slice(1,8) if side=='l' else slice(8,15)
        for i in range(25):
            q=rng.uniform(-.6,.6,15);q[0]=.2
            y,k,p,_=self.model.regressor(q,side);t=np.zeros(15);t[slot]=(y@p+k)*self.gain
            poses.append((str(i),q,q,t))
        return poses

    def test_exact_urdf_keeps_prior_on_both_arms(self):
        for side in ('l','r'):
            r=fit_physical(self.model,self.poses(side),side,self.gain)
            self.assertTrue(r['accepted_offline'])
            np.testing.assert_allclose(r['mass_scale'],1,atol=1e-8)
            np.testing.assert_allclose(r['com_delta_m'],0,atol=1e-8)
            np.testing.assert_allclose(r['cross_validation_rms_permille'],0,atol=1e-8)
            self.assertFalse(r['dynamic_gains_validated'])
            self.assertFalse(r['deployment_allowed'])

    def test_wrong_sign_cannot_be_hidden_by_negative_gain(self):
        poses=self.poses('r')
        for _,_,_,t in poses:t[8:]*=-1
        r=fit_physical(self.model,poses,'r',self.gain)
        self.assertFalse(r['accepted_offline'])
        self.assertTrue(np.all(np.array(r['identified_current_per_nm'])>0))
        self.assertTrue(np.all(np.array(r['mass_scale'])>=.7-1e-12))
        self.assertTrue(np.all(np.abs(r['com_delta_m'])<=.03+1e-12))

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):fit_physical(self.model,[],'r',self.gain)
        with self.assertRaises(ValueError):fit_physical(self.model,self.poses('r'),'r',[-1]*7)

    def test_no_additional_payload_keeps_urdf_end_link_mass(self):
        path=Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf'
        cad=Model(path,[0,0]);extra=Model(path,[1,1])
        q=np.linspace(-.4,.4,15)
        for side in ('l','r'):
            y,k,prior,_=cad.regressor(q,side);yy,kk,pp,_=extra.regressor(q,side)
            np.testing.assert_array_equal(y,yy)
            np.testing.assert_array_equal(prior,pp)
            self.assertAlmostEqual(prior[-4],1.53968802705119)
            np.testing.assert_allclose(k,0,atol=1e-12)
            self.assertGreater(np.linalg.norm(kk-k),.1)


class SessionInputTest(unittest.TestCase):
    def test_same_pose_names_are_separate_between_sessions(self):
        import tempfile
        from physical_current_identify import read_sessions
        with tempfile.TemporaryDirectory() as d:
            paths=[]
            for name in ('one','two'):
                folder=Path(d)/name;folder.mkdir()
                for sign in ('1','-1'):
                    path=folder/(sign+'.csv')
                    path.write_text('pose_id,approach,note\npose01,'+sign+','+name+'\n');paths.append(path)
            rows,sessions=read_sessions(paths)
            self.assertEqual(len(sessions),2)
            self.assertEqual(rows[0]['pose_id'],rows[1]['pose_id'])
            self.assertNotEqual(rows[0]['pose_id'],rows[2]['pose_id'])
            single,_=read_sessions(paths[:2]);self.assertEqual(single[0]['pose_id'],'pose01')
            with self.assertRaisesRegex(ValueError,'重复CSV路径'):read_sessions([paths[0],paths[0]])
            copy=Path(d)/'copy.csv';copy.write_bytes(paths[0].read_bytes())
            with self.assertRaisesRegex(ValueError,'重复CSV内容'):read_sessions([paths[0],copy])
