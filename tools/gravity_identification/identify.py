#!/usr/bin/env python3
"""从双向静态保持样本辨识双臂质量/一阶矩，并生成待验证的拖动参数。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from model import Model, JOINTS


def paired(rows, side, speed_max=.02, pose_tolerance=.02):
    hashes={r.get('conversion_sha256','') for r in rows}
    if len(hashes)!=1 or '' in hashes: raise ValueError('换算配置摘要缺失或不一致')
    groups = {}
    for row in rows:
        if row['side'] != side: continue
        q = np.array([float(row['q_'+j]) for j in JOINTS])
        dq = np.array([float(row['dq_'+j]) for j in JOINTS])
        tau = np.array([float(row['tau_'+j]) for j in JOINTS])
        if not np.isfinite(np.r_[q,dq,tau]).all(): raise ValueError('样本含 NaN/Inf')
        if row['valid'] != '1' or np.max(np.abs(dq)) > speed_max: continue
        direction = int(row['approach'])
        if direction not in (-1,1): raise ValueError('approach 必须为 ±1')
        groups.setdefault(row['pose_id'], {}).setdefault(direction, []).append((q,tau,float(row['time_ns'])))
    result = []
    for pose, approaches in sorted(groups.items()):
        if set(approaches) != {-1,1} or min(map(len,approaches.values())) < 20: continue
        stable=True
        for samples in approaches.values():
            stamps=np.array([s[2] for s in samples])
            angles=np.array([s[0] for s in samples])
            if not np.isfinite(stamps).all() or len(np.unique(stamps))!=len(stamps) or np.ptp(stamps)<500000000 or np.max(np.ptp(angles,axis=0))>pose_tolerance:
                stable=False
        if not stable: continue
        a,b = [(np.median([s[0] for s in approaches[d]],axis=0),
                np.median([s[1] for s in approaches[d]],axis=0)) for d in (-1,1)]
        if np.max(np.abs(a[0]-b[0])) > pose_tolerance: continue
        # 样本中腰必须保持同一位置；整条链都需接近相同姿态。
        result.append((pose,a[0],b[0],(a[1]+b[1])/2))
    if len(result)<15: raise ValueError(f'{side}: 双向配对姿态不足 15 个，得到 {len(result)}')
    if np.ptp([p[1][0] for p in result]) > pose_tolerance: raise ValueError('腰部未保持同一位置')
    return result


def fit(model, poses, side, rms_limit):
    ys, ks, ts, inertias = [], [], [], []
    slots = slice(1,8) if side=='l' else slice(8,15)
    for _,qa,qb,tau in poses:
        ya,ka,prior,ia = model.regressor(qa,side); yb,kb,_,ib = model.regressor(qb,side)
        ys.append((ya+yb)/2); ks.append((ka+kb)/2); ts.append(tau[slots]); inertias.extend([ia,ib])
    y,k,target = np.array(ys),np.array(ks),np.array(ts)
    # 按姿态整组留出，避免同一姿态正反两侧泄漏到训练与验证。
    holdout = np.arange(len(poses))%5==0
    sigma = np.repeat(np.maximum(prior[::4],.001),4)*np.tile([.3,.03,.03,.03],7)
    def solve(mask):
        a = y[mask].reshape(-1,28); residual = (target-k)[mask].ravel()-a@prior
        scaled = a*sigma
        delta = np.linalg.lstsq(np.vstack([scaled,np.eye(28)*.05]),
                               np.r_[residual,np.zeros(28)],rcond=None)[0]
        return prior+sigma*delta
    train = solve(~holdout)
    rms = lambda theta: np.sqrt(np.mean((np.einsum('njk,k->nj',y[holdout],theta)+k[holdout]-target[holdout])**2,axis=0))
    cad_rms, validation = rms(prior),rms(train)
    theta = solve(np.ones(len(poses),bool))
    scales = theta[::4]/prior[::4]
    com_delta = theta.reshape(7,4)[:,1:]/theta[::4,None]-prior.reshape(7,4)[:,1:]/prior[::4,None]
    reasons=[]
    if np.any(scales<.5) or np.any(scales>2) or np.any(np.abs(com_delta)>.08): reasons.append('质量/质心超出候选范围')
    if np.any(validation > rms_limit): reasons.append('留出姿态误差超阈值')
    if np.mean(validation**2)>np.mean(cad_rms**2): reasons.append('留出误差劣于 CAD')
    # 静力学参数通常不可全部独立辨识，报告秩，不声称得到每节唯一真实质量。
    rank = int(np.linalg.matrix_rank(y[~holdout].reshape(-1,28)*sigma))
    pose_rank = int(np.linalg.matrix_rank(np.array([p[1][slots] for p in poses])-np.mean([p[1][slots] for p in poses],axis=0)))
    if rank<7 or pose_rank<3: reasons.append('姿态激励不足')
    return dict(side=side,accepted_offline=not reasons,rejection_reasons=reasons,
        paired_poses=len(poses),validation_pose_ids=[p[0] for p,h in zip(poses,holdout) if h],
        regressor_rank=rank,pose_rank=pose_rank,parameter_count=28,mass_scale=scales.tolist(),com_delta_m=com_delta.tolist(),
        validation_rms_nm=validation.tolist(),cad_validation_rms_nm=cad_rms.tolist(),
        inertia_diagonal_max_kg_m2=np.max(inertias,axis=0).tolist(),
        # 自由拖动 Kp=0，按惯量/期望速度衰减时间得到阻尼候选，绝非实机稳定性保证。
        drag_kp_nm_rad=[0.]*7,drag_kd_nm_s_rad=(np.max(inertias,axis=0)/.25).tolist(),
        friction_gain=0.,deployment_status='candidate_not_applied')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv',type=Path,nargs='+');p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--gripper-kg',type=float,nargs=2,required=True,metavar=('LEFT','RIGHT'))
    p.add_argument('--max-rms-nm',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not np.isfinite(args.max_rms_nm) or args.max_rms_nm<=0: p.error('误差阈值必须为正')
    model=Model(args.urdf,args.gripper_kg)
    rows=[]
    for path in args.csv:
        with path.open() as f: rows.extend(csv.DictReader(f))
    report={'schema':'x1_drag_candidate_v1','waist_mode':'position_hold','runtime_applied':False,
        'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in args.csv},
        'urdf_sha256':hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
        'gripper_kg':args.gripper_kg,'arms':[fit(model,paired(rows,s),s,args.max_rms_nm) for s in 'lr']}
    # 只产生独立候选，不覆盖运行配置；JSON 也是合法 YAML 1.2。
    with args.output.open('x') as f: json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(args.output)

if __name__=='__main__': main()
