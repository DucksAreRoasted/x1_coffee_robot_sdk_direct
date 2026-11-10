"""只读分析已配对样本的模型偏差，不拟合或应用实机参数。"""
import argparse,csv,json,hashlib
from pathlib import Path
import numpy as np
from model import Model,JOINTS
from identify import paired
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv',type=Path,nargs='+')
parser.add_argument('--urdf',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args()
rows=[]
for f in a.csv:
 with f.open() as s:rows.extend(csv.DictReader(s))
poses=paired(rows,'r');m=Model(a.urdf,[1,1]);pred=[];actual=[]
for _,qa,b,t in poses:
 ya,ka,p,_=m.regressor(qa,'r');yb,kb,_,_=m.regressor(b,'r')
 pred.append(((ya+yb)@p+ka+kb)/2);actual.append(t[8:])
pred=np.array(pred);actual=np.array(actual)
result=[]
for i,j in enumerate(JOINTS[8:]):
 halves=[]
 for pose,*_ in poses:
  med=[np.median([float(r['tau_'+j]) for r in rows if r['pose_id']==pose and int(r['approach'])==d]) for d in [-1,1]]
  halves.append(abs(med[1]-med[0])/2)
 result.append(dict(joint=j,cad_mean_nm=float(pred[:,i].mean()),measured_mean_nm=float(actual[:,i].mean()),bias_nm=float((actual-pred)[:,i].mean()),cad_rms_nm=float(np.sqrt(np.mean((actual-pred)[:,i]**2))),direction_half_gap_median_nm=float(np.median(halves)),q_span_rad=float(np.ptp([a[8+i] for _,a,b,t in poses]))))
# 保存来源摘要，诊断结果不构成参数发布许可。
report=dict(schema='x1_gravity_diagnostics_v1',runtime_applied=False,gripper_kg=[1,1],
    paired_poses=len(poses),joints=result,
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in a.csv},
    urdf_sha256=hashlib.sha256(a.urdf.read_bytes()).hexdigest())
with a.output.open('x') as stream:json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
print(a.output)
