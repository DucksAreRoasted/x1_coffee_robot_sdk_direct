#!/usr/bin/env python3
"""离线比较重力基函数及低阶残差假设；只诊断，不生成可加载参数。"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from model import Model
from current_identify import features


def solve(x, y):
    # 每折独立归一化和正则，不使用被验证姿态的电流或特征统计。
    scale=np.maximum(np.sqrt(np.mean(x*x,axis=0)),1e-8)
    beta=np.linalg.lstsq(np.vstack((x/scale,.05*np.eye(x.shape[1]))),
                        np.r_[y,np.zeros(x.shape[1])],rcond=None)[0]
    return beta/scale


def bases(model, q, side, axis):
    slot=1 if side=='l' else 8
    gravity=np.array([features(model,v,side)[axis-1]for v in q])
    one=np.ones(len(q));angle=q[:,slot+axis-1]
    return dict(gravity=gravity,constant=np.c_[gravity,one],
                linear_axis=np.c_[gravity,one,angle],
                sincos_axis=np.c_[gravity,one,np.sin(angle),np.cos(angle)],
                linear_wrist=np.c_[gravity,one,q[:,slot+4:slot+7]])


def audit(model, candidate, repeat, axis=6):
    side=candidate['side'];q=np.asarray(candidate['sample_poses'],float)
    y=np.asarray(candidate['observed_current_permille'],float)[:,axis-1]
    if len(q)<15 or not np.isfinite(q).all() or not np.isfinite(y).all():raise ValueError('无效采样数据')
    matrices=bases(model,q,side,axis);ids=np.arange(len(y));out={}
    for name,x in matrices.items():
        predictions=np.zeros(len(y))
        for fold in range(5):
            held=ids%5==fold;predictions[held]=x[held]@solve(x[~held],y[~held])
        beta=solve(x,y);checks=[]
        # 复测含训练时已有姿态，检查重复性而非声称独立工作区验证。
        for pose in repeat['poses']:
            for trial in pose['pairs']:
                windows=trial['windows'];qq=np.array([w['q_median']for w in windows])
                predicted=float(np.mean(bases(model,qq,side,axis)[name]@beta))
                observed=trial['mean_current'][axis-1]
                checks.append(dict(pose=pose['pose'],repeat=trial['repeat'],predicted=predicted,
                                   observed=observed,error=predicted-observed))
        error=predictions-y
        out[name]=dict(cross_validation_rms_permille=float(np.sqrt(np.mean(error**2))),
                       training_rms_permille=float(np.sqrt(np.mean((x@beta-y)**2))),
                       repeat_checks=checks,repeat_rms_permille=float(np.sqrt(np.mean([c['error']**2 for c in checks]))) if checks else None)
    # 内层选假设，外层评估整个选择过程，防止挑最好的一次五折后宣称通过。
    nested=np.zeros(len(y));choices=[]
    for fold in range(5):
        held=ids%5==fold;train_ids=ids[~held];scores={}
        for name,x in matrices.items():
            errors=[]
            for inner in range(4):
                validation=train_ids[np.arange(len(train_ids))%4==inner]
                training=train_ids[np.arange(len(train_ids))%4!=inner]
                errors.extend((x[validation]@solve(x[training],y[training])-y[validation]).tolist())
            scores[name]=float(np.mean(np.square(errors)))
        selected=min(scores,key=scores.get);choices.append(selected);x=matrices[selected]
        nested[held]=x[held]@solve(x[~held],y[~held])
    return dict(schema='x1_current_residual_audit_v1',side=side,axis=axis,models=out,
                nested_selection_rms_permille=float(np.sqrt(np.mean((nested-y)**2))),
                nested_selected_models=choices,runtime_applied=False,deployment_allowed=False,
                note='低阶假设诊断；复测使用已见姿态，不是独立工作区验证；不据此修改零位或方向。')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('candidate','repeat-report','urdf','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();candidate=json.loads(a.candidate.read_text());repeat=json.loads(a.repeat_report.read_text())
    if candidate['urdf_sha256']!=hashlib.sha256(a.urdf.read_bytes()).hexdigest():raise ValueError('URDF摘要不一致')
    if repeat['source_candidate_sha256']!=hashlib.sha256(a.candidate.read_bytes()).hexdigest():raise ValueError('复测来源候选不一致')
    report=audit(Model(a.urdf,[1,1]),candidate,repeat)
    report['source_sha256']={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (a.candidate,a.repeat_report,a.urdf)}
    with a.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({name:round(v['cross_validation_rms_permille'],3)for name,v in report['models'].items()}))
    print('嵌套验证RMS：',report['nested_selection_rms_permille'])

if __name__=='__main__':main()
