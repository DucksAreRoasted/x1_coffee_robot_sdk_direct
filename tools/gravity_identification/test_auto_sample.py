import unittest
from pathlib import Path
import numpy as np
from model import Model,JOINTS
from auto_sample import make_plan,path_points,check_local_pose,optimize_visit_order,movement_seconds,repeat_plan

class PlanTest(unittest.TestCase):
    def setUp(self):
        self.model=Model(Path(__file__).resolve().parents[2]/'src/arm_describle/urdf/HRB3Q-LC.urdf',[1,1])
    def test_pairs_and_waist_left_fixed(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+float(self.model.by_name[j].find('limit').get('upper')))/2 for j in __import__('model').JOINTS]);plan=make_plan(seed,self.model)
        self.assertEqual(len(plan),60)
        for i in range(0,60,4):
            np.testing.assert_allclose(plan[i+1]['q'],plan[i+3]['q'])
            self.assertEqual(plan[i+1]['approach'],1)
            self.assertEqual(plan[i+3]['approach'],-1)
        for step in plan:np.testing.assert_allclose(step['q'][:8],seed[:8])
    def test_limits_and_nonfinite_refused(self):
        for seed in [np.full(15,np.nan),np.full(15,100)]:
            with self.assertRaises(ValueError):make_plan(seed,self.model)
    def test_right_joint_at_both_limits_keeps_two_sided_approach_inside_margin(self):
        for edge in ['lower','upper']:
            seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
            seed[9]=float(self.model.by_name['Joint_r2'].find('limit').get(edge))
            plan=make_plan(seed,self.model)
            for step in plan:
                check_local_pose(seed,step['q'])
                np.testing.assert_allclose(step['q'][:8],seed[:8])
                for j in range(8,15):
                    limit=self.model.by_name[JOINTS[j]].find('limit')
                    self.assertGreaterEqual(step['q'][j],float(limit.get('lower'))+.05-1e-12)
                    self.assertLessEqual(step['q'][j],float(limit.get('upper'))-.05+1e-12)
            for i in range(0,60,4):
                self.assertLess(plan[i]['q'][9],plan[i+1]['q'][9])
                self.assertGreater(plan[i+2]['q'][9],plan[i+3]['q'][9])
    def test_local_envelope_rejects_full_turn_and_drift(self):
        seed=np.zeros(15)
        for j,delta in [(9,2*np.pi),(9,-2*np.pi),(14,.201),(0,.021),(7,-.021)]:
            q=seed.copy();q[j]=delta
            with self.assertRaisesRegex(ValueError,'超过局部范围'):check_local_pose(seed,q)
        q=seed.copy();q[8:]=.20;q[:8]=.02
        check_local_pose(seed,q)
        q[9]=np.nan
        with self.assertRaises(ValueError):check_local_pose(seed,q)

    def test_narrow_range_is_rejected(self):
        limit=self.model.by_name['Joint_r2'].find('limit')
        limit.set('lower','-.1');limit.set('upper','.1')
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
        with self.assertRaisesRegex(ValueError,'可用辨识区间'):make_plan(seed,self.model)
    def test_segment_includes_endpoints_and_spacing(self):
        p=path_points(np.zeros(15),np.ones(15)*.127)
        np.testing.assert_allclose(p[0],0);np.testing.assert_allclose(p[-1],.127)
        self.assertLessEqual(np.max(np.abs(np.diff(p,axis=0))),.01)

    def test_left_plan_keeps_waist_right_fixed_and_pairs(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+
                        float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
        fixed=[0,*range(8,15)]
        for edge in ('lower','upper'):
            seed[2]=float(self.model.by_name['Joint_l2'].find('limit').get(edge))
            plan=make_plan(seed,self.model,'l')
            self.assertEqual(len(plan),60)
            for step in plan:
                q=np.asarray(step['q'])
                np.testing.assert_array_equal(q[fixed],seed[fixed])
                check_local_pose(seed,q,'l')
                for j in range(1,8):
                    limit=self.model.by_name[JOINTS[j]].find('limit')
                    self.assertGreaterEqual(q[j],float(limit.get('lower'))+.05-1e-12)
                    self.assertLessEqual(q[j],float(limit.get('upper'))-.05+1e-12)
            for i in range(0,60,4):
                np.testing.assert_array_equal(plan[i+1]['q'],plan[i+3]['q'])
                self.assertLess(plan[i]['q'][2],plan[i+1]['q'][2])
                self.assertGreater(plan[i+2]['q'][2],plan[i+3]['q'][2])

    def test_left_monitor_rejects_right_drift_and_left_full_turn(self):
        seed=np.zeros(15)
        for j,delta in [(0,.021),(8,.021),(14,-.021),(2,2*np.pi),(7,.201)]:
            q=seed.copy();q[j]=delta
            with self.assertRaises(ValueError):check_local_pose(seed,q,'l')
        q=seed.copy();q[1:8]=.20
        check_local_pose(seed,q,'l')
        with self.assertRaises(ValueError):make_plan(seed,self.model,'both')

    def test_more_poses_preserve_pairs_and_local_envelope(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+
                        float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
        plan=make_plan(seed,self.model,'r',30)
        self.assertEqual(len(plan),120)
        records=[step for step in plan if step['record']]
        self.assertEqual(len({step['pose_id'] for step in records}),30)
        for step in plan:check_local_pose(seed,step['q'],'r')
        for count in (0,14,101):
            with self.assertRaises(ValueError):make_plan(seed,self.model,'r',count)

    def test_reorder_preserves_all_pairs_and_never_increases_duration(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+
                        float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
        import json
        for side in ('l','r'):
            plan=make_plan(seed,self.model,side,30);ordered,saved=optimize_visit_order(seed,plan)
            self.assertEqual(sorted(json.dumps(v,sort_keys=True) for v in plan),
                             sorted(json.dumps(v,sort_keys=True) for v in ordered))
            self.assertGreaterEqual(saved,0)
            for i in range(0,len(ordered),4):
                self.assertEqual(len({s['pose_id'] for s in ordered[i:i+4]}),1)
                self.assertEqual([s['approach'] for s in ordered[i:i+4]],[1,1,-1,-1])

    def test_expanded_plan_respects_clearance_filter_and_envelope(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+
                        float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
        def clearance(q):
            if q[8]<seed[8]-.1:raise ValueError('模拟贴近躯干')
        plan=make_plan(seed,self.model,'r',30,.35,clearance)
        self.assertGreater(max(abs(np.array(s['q'])[9:]-seed[9:]).max()for s in plan),.2)
        for step in plan:
            clearance(step['q']);check_local_pose(seed,step['q'],'r',.4)
        with self.assertRaises(ValueError):make_plan(seed,self.model,'r',30,.36)

    def test_repeat_uses_original_envelope_and_keeps_trial_identity(self):
        seed=np.array([(float(self.model.by_name[j].find('limit').get('lower'))+
                        float(self.model.by_name[j].find('limit').get('upper')))/2 for j in JOINTS])
        source=dict(side='r',seed=seed.tolist(),local_radius_rad=.4,
                    steps=make_plan(seed,self.model,'r',30,.35))
        center,radius,plan=repeat_plan(source,['pose14','pose24','pose30'],3,self.model,'r',lambda q:None)
        np.testing.assert_array_equal(center,seed);self.assertEqual(radius,.4)
        self.assertEqual(len(plan),36);self.assertEqual(len({p['pose_id']for p in plan}),9)
        for i in range(0,len(plan),4):np.testing.assert_array_equal(plan[i+1]['q'],plan[i+3]['q'])
        with self.assertRaises(ValueError):repeat_plan(source,['missing'],3,self.model,'r',lambda q:None)
        with self.assertRaises(ValueError):repeat_plan(source,['pose14'],3,self.model,'l',lambda q:None)

if __name__=='__main__':unittest.main()
