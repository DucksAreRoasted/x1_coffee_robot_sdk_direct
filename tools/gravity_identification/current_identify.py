#!/usr/bin/env python3
"""以驱动器电流千分比辨识耦合重力基函数，不反推唯一的连杆质量。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from model import Model, JOINTS
from identify import paired
from record import validated_joints
from settling import stable_window


def features(model, q, side):
    y, known, _, _ = model.regressor(q, side)
    # 固定部件（含夹爪）作为第29列，不添加可掩盖零位错误的常量偏置。
    return np.column_stack((y, known))


def current_pairs(rows, cfg, side, allow_legacy=False):
    selected = validated_joints(cfg, side)
    converted = []
    legacy = False
    groups = {}
    for row in rows:
        if row['side'] != side:
            continue
        r = dict(row)
        for j in JOINTS:
            r['tau_'+j] = 0.
            if j not in selected:
                continue
            key = 'raw_current_permille_'+j
            if key not in row:
                if not allow_legacy:
                    raise ValueError('缺少原始电流，旧样本只能显式用于诊断')
                legacy = True
                # 原换算已包含方向，此处只恢复关节方向下的千分比。
                r['tau_'+j] = float(row['tau_'+j])*1000/cfg['joints'][j]['rated_nm']
            else:
                if int(row.get('raw_current_valid_'+j, 0)) != 1:
                    r['valid'] = 0
                r['tau_'+j] = float(row[key])*cfg['joints'][j]['sign']
        converted.append(r)
        groups.setdefault((row['pose_id'], int(row['approach'])), []).append(row)
    reasons = []
    if legacy:
        reasons.append('旧样本以扭矩反馈近似电流，不能部署')
    else:
        for name, batch in groups.items():
            batch = sorted(batch, key=lambda r: int(r['time_ns']))
            if not stable_window(batch, selected):
                reasons.append(f'{name}: 电流/力矩窗口未通过稳定性检查')
    return paired(converted, side), reasons


def fit_current(model, poses, side, rms_limit=10.):
    if len(poses)<15 or not np.isfinite(rms_limit) or rms_limit<=0:
        raise ValueError('需要至少15组姿态及正误差阈值')
    slots = slice(1,8) if side=='l' else slice(8,15)
    x = np.array([(features(model,a,side)+features(model,b,side))/2 for _,a,b,_ in poses])
    target = np.array([t[slots] for _,_,_,t in poses])
    if not np.isfinite(np.r_[x.ravel(), target.ravel()]).all():
        raise ValueError('回归数据含非有限值')
    def solve(mask):
        beta = []; ranks = []
        for j in range(7):
            a = x[mask,j,:]
            scale = np.sqrt(np.mean(a*a,axis=0))
            scale[scale<1e-8] = 1.
            normalized = a/scale
            # 固定正则强度；不用验证集挑系数。系数为几何特征到千分比的映射。
            b = np.linalg.lstsq(np.vstack((normalized,.05*np.eye(29))),
                                np.r_[target[mask,j],np.zeros(29)],rcond=None)[0]
            beta.append(b/scale)
            ranks.append(int(np.linalg.matrix_rank(normalized)))
        return np.array(beta), ranks
    prediction = np.zeros_like(target)
    ids = np.arange(len(poses))
    for fold in range(5):
        held = ids%5==fold
        b,_ = solve(~held)
        prediction[held] = np.einsum('njk,jk->nj',x[held],b)
    beta,ranks = solve(np.ones(len(poses),dtype=bool))
    residual = prediction-target
    rms = np.sqrt(np.mean(residual**2,axis=0))
    q = np.array([(a+b)/2 for _,a,b,_ in poses])
    pose_rank = int(np.linalg.matrix_rank(q[:,slots]-q[:,slots].mean(axis=0)))
    reasons = []
    if np.any(rms>rms_limit):reasons.append('整姿态五折验证误差超阈值')
    if pose_rank<3:reasons.append('姿态激励不足')
    return dict(schema='x1_current_gravity_v1',side=side,unit='joint_signed_current_permille',
        basis='urdf_mass_first_moment_28_plus_known_gravity',coefficients=beta.tolist(),
        accepted_offline=not reasons,rejection_reasons=reasons,
        cross_validation_rms_permille=rms.tolist(),max_abs_validation_error_permille=np.max(abs(residual),axis=0).tolist(),
        validation_pose_ids=[p[0] for p in poses],validation_predictions=prediction.tolist(),
        observed_current_permille=target.tolist(),feature_rank_per_joint=ranks,pose_rank=pose_rank,
        q_min=q.min(axis=0).tolist(),q_max=q.max(axis=0).tolist(),sample_poses=q.tolist(),
        validation_limit_permille=rms_limit,
        domain_note='采样范围仅作记录，不能保证范围内所有组合均已验证；禁止未验证外推',
        runtime_applied=False,deployment_allowed=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv',type=Path,nargs='+')
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--conversion',type=Path,default=Path(__file__).with_name('right_arm_session.json'))
    p.add_argument('--side',choices=['l','r'],default='r')
    p.add_argument('--max-rms-permille',type=float,default=10.)
    p.add_argument('--allow-legacy-diagnostic',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); rows=[]
    for f in a.csv:
        with f.open() as stream:rows.extend(csv.DictReader(stream))
    cfg=json.loads(a.conversion.read_text())
    if {r['conversion_sha256'] for r in rows}!={hashlib.sha256(a.conversion.read_bytes()).hexdigest()}:
        raise ValueError('换算配置与样本摘要不一致')
    poses,reasons=current_pairs(rows,cfg,a.side,a.allow_legacy_diagnostic)
    report=fit_current(Model(a.urdf,[1,1]),poses,a.side,a.max_rms_permille)
    report['rejection_reasons'].extend(reasons)
    report['accepted_offline']=not report['rejection_reasons']
    report['source_sha256']={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [*a.csv,a.urdf,a.conversion]}
    report['gripper_kg']=[1,1]
    report['urdf_sha256']=hashlib.sha256(a.urdf.read_bytes()).hexdigest()
    report['conversion_sha256']=hashlib.sha256(a.conversion.read_bytes()).hexdigest()
    with a.output.open('x') as stream:json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(a.output)


if __name__=='__main__':main()
