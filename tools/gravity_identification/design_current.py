#!/usr/bin/env python3
"""按显式电流预算设计拖动阻尼初值，仅输出离线审阅报告，不加载控制器。"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from current_identify import features
from model import Model
from export_current import arm_record
from physical_current_compile import compile_coefficients


def design(candidate, model, limits, speeds, damping_fraction, ramp_seconds):
    # 参数均为设计输入，不能从额定扭矩或旧 MIT 稳定保持推定实机可用值。
    limits=np.asarray(limits,float);speeds=np.asarray(speeds,float)
    if (limits.shape!=(7,) or speeds.shape!=(7,) or
            not np.isfinite(np.r_[limits,speeds]).all() or np.any(limits<=0) or
            np.any(limits>1000) or np.any(speeds<=0)):
        raise ValueError('需提供7轴正电流预算（不超过1000‰）及正参考速度')
    if not np.isfinite([damping_fraction,ramp_seconds]).all() or not 0<damping_fraction<1 or ramp_seconds<=0:
        raise ValueError('阻尼余量占比必须在0和1之间，爬升时间必须为正')
    if candidate.get('accepted_offline') is not True or candidate.get('rejection_reasons')!=[]:
        raise ValueError('候选未通过离线验证')
    side=candidate['side']
    if side not in ('l','r'):raise ValueError('无效臂名称')
    beta=compile_coefficients(candidate,model) if candidate.get('schema')=='x1_physical_current_candidate_v1' else np.asarray(candidate['coefficients'],float)
    poses=np.asarray(candidate['sample_poses'],float)
    observed=np.asarray(candidate['observed_current_permille'],float)
    error=np.asarray(candidate['max_abs_validation_error_permille'],float)
    if (beta.shape!=(7,29) or poses.ndim!=2 or poses.shape[1]!=15 or len(poses)<15 or
            observed.shape!=(len(poses),7) or error.shape!=(7,) or np.any(error<0) or
            not all(np.isfinite(v).all() for v in (beta,poses,observed,error))):
        raise ValueError('候选数据形状或数值无效')
    predicted=np.array([np.einsum('jk,jk->j',features(model,q,side),beta) for q in poses])
    peak=np.maximum(np.max(abs(predicted),axis=0),np.max(abs(observed),axis=0))
    reserve=limits-peak-error
    if np.any(reserve<=0):
        bad=[f'Joint_{side}{j+1}' for j in range(7) if reserve[j]<=0]
        raise ValueError('预算不足以覆盖采样峰值及验证误差: '+','.join(bad))
    kd=damping_fraction*reserve/speeds
    # 全预算阶跃至少经过指定时长；是指令斜率，不是机械响应时间保证。
    slew=limits/ramp_seconds
    return dict(schema='x1_current_design_v1',side=side,kp=[0.]*7,
        kd_permille_s_rad=kd.tolist(),limits_permille=limits.tolist(),
        slew_permille_s=slew.tolist(),reference_speed_rad_s=speeds.tolist(),
        sampled_peak_permille=peak.tolist(),validation_error_allowance_permille=error.tolist(),
        remaining_current_permille=reserve.tolist(),damping_fraction=damping_fraction,
        full_budget_ramp_seconds=ramp_seconds,runtime_applied=False,deployment_allowed=False,
        note='阻尼为无位置弹簧拖动的调试初值。采样峰值与验证误差不是整个邻域的上界；不证明动力学稳定或实机安全。腰维持CSP。')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('candidate',type=Path)
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--conversion',type=Path,required=True)
    p.add_argument('--limits',type=float,nargs=7,required=True)
    p.add_argument('--speeds',type=float,nargs=7,required=True)
    p.add_argument('--damping-fraction',type=float,required=True)
    p.add_argument('--ramp-seconds',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();raw=a.candidate.read_bytes();candidate=json.loads(raw)
    urdf=a.urdf.read_bytes();conversion=a.conversion.read_bytes()
    # 先复用导出端的来源、单位、方向和交叉验证检查，预算检查稍后完成。
    arm_record(candidate,json.loads(conversion),urdf,conversion,
               kd=[1]*7,limits=a.limits,slew=[1]*7,radius=.03)
    report=design(candidate,Model(a.urdf,[1,1]),a.limits,a.speeds,a.damping_fraction,a.ramp_seconds)
    report['candidate_sha256']=hashlib.sha256(raw).hexdigest()
    report['urdf_sha256']=hashlib.sha256(urdf).hexdigest()
    report['conversion_sha256']=hashlib.sha256(conversion).hexdigest()
    with a.output.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print('仅离线设计，未加载：',a.output)


if __name__=='__main__':main()
