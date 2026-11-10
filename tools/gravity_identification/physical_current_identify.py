#!/usr/bin/env python3
"""以URDF质量/质心为先验，共享物理参数辨识；只输出离线候选。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from model import Model
from current_identify import current_pairs


def nominal_gains(inventory, side, efficiency):
    # Kt、效率仍是假设，明确记入报告，不能作为已测量的输出力矩标定。
    if not np.isfinite(efficiency) or not 0<efficiency<=1:
        raise ValueError('效率假设须在0到1之间')
    gains=[]
    for j in range(1,8):
        motor=inventory['joints'][f'Joint_{side}{j}']
        kt=motor['analysis_kt_hypothesis']['value_nm_per_a']
        denominator=kt*motor['nominal_gear_ratio']*motor['drive_rated_current_a_0x6075']*efficiency
        if not np.isfinite(denominator) or denominator<=0:raise ValueError('无效电流换算依据')
        gains.append(1000/denominator)
    return np.asarray(gains)


def fit_physical(model, poses, side, nominal, rms_limit=10.):
    if side not in ('l','r') or len(poses)<15:raise ValueError('需要有效臂名称及至少15组姿态')
    nominal=np.asarray(nominal,float)
    if nominal.shape!=(7,) or not np.isfinite(nominal).all() or np.any(nominal<=0):raise ValueError('需要7轴正换算初值')
    if not np.isfinite(rms_limit) or rms_limit<=0:raise ValueError('无效误差门槛')
    slot=slice(1,8) if side=='l' else slice(8,15)
    ys=[];ks=[];targets=[]
    for _,qa,qb,t in poses:
        ya,ka,prior,_=model.regressor(qa,side);yb,kb,_,_=model.regressor(qb,side)
        ys.append((ya+yb)/2);ks.append((ka+kb)/2);targets.append(t[slot])
    y=np.asarray(ys);known=np.asarray(ks);target=np.asarray(targets)
    if not all(np.isfinite(v).all() for v in (y,known,target,prior)):raise ValueError('非有限输入')
    masses=prior[::4];com=prior.reshape(7,4)[:,1:]/masses[:,None]
    if np.any(masses<=0):raise ValueError('URDF连杆质量须为正')

    def unpack(z):
        # 硬约束：质量±30%，质心每方向±3cm，电流换算为初值的0.5~2倍。
        p=z[:28].reshape(7,4);mass=masses*(1+.3*p[:,0]);center=com+.03*p[:,1:]
        theta=np.column_stack((mass,mass[:,None]*center)).ravel()
        gain=nominal*np.exp(np.log(2)*z[28:])
        return theta,gain,mass,center

    def prediction(z,mask):
        theta,gain,_,_=unpack(z)
        return (np.einsum('njk,k->nj',y[mask],theta)+known[mask])*gain

    def solve(mask):
        z=np.zeros(35);damping=1e-3
        # 固定正则/范围；验证数据不参与选超参数。共享28个物理量，不逐轴独立拟质量。
        def residual(value):return np.r_[(prediction(value,mask)-target[mask]).ravel()/10.,value]
        r=residual(z)
        for _ in range(80):
            jac=np.empty((len(r),35))
            for k in range(35):
                dz=z.copy();dz[k]+=1e-5;jac[:,k]=(residual(dz)-r)/1e-5
            step=np.linalg.solve(jac.T@jac+damping*np.eye(35),-jac.T@r)
            proposed=np.clip(z+step,-1,1);rr=residual(proposed)
            if rr@rr<r@r:
                change=np.max(abs(proposed-z));z=proposed;r=rr;damping=max(damping*.3,1e-8)
                if change<1e-6:break
            else:
                damping*=10
                if damping>1e9:break
        return z

    ids=np.arange(len(poses));validation=np.zeros_like(target);fold_boundary=[]
    for fold in range(5):
        held=ids%5==fold;z=solve(~held);validation[held]=prediction(z,held)
        fold_boundary.append(np.flatnonzero(abs(z)>.999).tolist())
    all_rows=np.ones(len(poses),bool);z=solve(all_rows);theta,gain,mass,center=unpack(z)
    error=validation-target;rms=np.sqrt(np.mean(error**2,axis=0))
    baseline=prediction(np.zeros(35),all_rows)
    # 数据雅可比不加入正则行，避免把先验当成观测信息、虚报可辨识性。
    base_prediction=prediction(z,all_rows).ravel()
    data_jacobian=np.empty((base_prediction.size,35))
    for column in range(35):
        shifted=z.copy();shifted[column]+=1e-5
        data_jacobian[:,column]=(prediction(shifted,all_rows).ravel()-base_prediction)/1e-5
    singular_values=np.linalg.svd(data_jacobian,compute_uv=False)
    numerical_rank=int(np.sum(singular_values>max(singular_values[0]*1e-6,1e-8)))
    q=np.array([(qa+qb)/2 for _,qa,qb,_ in poses])
    reasons=[]
    if np.any(rms>rms_limit):reasons.append('物理先验模型整姿态五折误差超阈值')
    if np.any(abs(z)>.999) or any(fold_boundary):reasons.append('拟合触及物理或换算边界，需核实模型及扩大激励')
    if np.linalg.matrix_rank(q[:,slot]-q[:,slot].mean(0))<3:reasons.append('姿态激励不足')
    return dict(schema='x1_physical_current_candidate_v1',side=side,accepted_offline=not reasons,
        rejection_reasons=reasons,runtime_applied=False,deployment_allowed=False,
        nominal_current_per_nm=nominal.tolist(),identified_current_per_nm=gain.tolist(),
        mass_scale=(mass/masses).tolist(),com_delta_m=(center-com).tolist(),
        shared_inertial_parameters=theta.tolist(),baseline_rms_permille=np.sqrt(np.mean((baseline-target)**2,axis=0)).tolist(),
        cross_validation_rms_permille=rms.tolist(),max_abs_validation_error_permille=np.max(abs(error),axis=0).tolist(),
        validation_predictions=validation.tolist(),observed_current_permille=target.tolist(),
        sample_poses=q.tolist(),validation_pose_ids=[p[0] for p in poses],
        bounds=dict(mass_scale=[.7,1.3],com_delta_each_axis_m=[-.03,.03],gain_scale=[.5,2]),
        bound_hits=np.flatnonzero(abs(z)>.999).tolist(),fold_bound_hits=fold_boundary,
        physical_parameters_unique=False,data_parameter_count=35,
        data_jacobian_rank=numerical_rank,data_jacobian_singular_values=singular_values.tolist(),
        physical_parameter_order=[f'link{i}_{name}' for i in range(1,8) for name in ('mass_scale','com_x','com_y','com_z')]+[f'joint{i}_current_gain' for i in range(1,8)],
        dynamic_gains_validated=False,
        note='URDF先验下的受约束估计，不证明物理参数唯一、动态稳定或全工作空间精度；须经物理模型编译器导出运行参数。')


def read_sessions(paths):
    """一轮采样一个目录；跨轮姿态名隔离，重复输入不能虚增验证样本。"""
    paths=[Path(p).resolve() for p in paths]
    if len(set(paths))!=len(paths):raise ValueError('重复CSV路径')
    parents=sorted(set(p.parent for p in paths))
    session_ids={parent:f'session{i+1:03}' for i,parent in enumerate(parents)}
    rows=[];seen=set();sessions={}
    for path in paths:
        raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
        if digest in seen:raise ValueError('重复CSV内容，不能重复计入样本：'+str(path))
        seen.add(digest)
        sid=session_ids[path.parent]
        with path.open() as stream:
            for row in csv.DictReader(stream):
                if len(parents)>1:row['pose_id']=sid+':'+row['pose_id']
                rows.append(row)
        sessions.setdefault(sid,dict(directory=str(path.parent),files=[]))['files'].append(str(path))
    return rows,sessions


def build_report(rows, csv_paths, urdf, conversion, inventory, side, efficiency, additional_gripper_kg):
    # 命令行与自动采样共用报告生成，避免负载语义和来源摘要不一致。
    digest=hashlib.sha256(conversion.read_bytes()).hexdigest()
    if {r['conversion_sha256'] for r in rows}!={digest}:raise ValueError('方向配置摘要不符')
    poses,reasons=current_pairs(rows,json.loads(conversion.read_text()),side)
    report=fit_physical(Model(urdf,additional_gripper_kg),poses,side,nominal_gains(json.loads(inventory.read_text()),side,efficiency))
    report['rejection_reasons'].extend(reasons);report['accepted_offline']=not report['rejection_reasons']
    report['efficiency_assumption']=efficiency
    report['additional_gripper_kg']=additional_gripper_kg
    report['payload_semantics']='additional_to_urdf_link_inertials_at_tcp'
    report['urdf_sha256']=hashlib.sha256(urdf.read_bytes()).hexdigest()
    report['conversion_sha256']=digest
    report['source_sha256']={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [*csv_paths,urdf,conversion,inventory]}
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv',type=Path,nargs='+');p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--conversion',type=Path,required=True);p.add_argument('--inventory',type=Path,required=True)
    p.add_argument('--efficiency',type=float,required=True);p.add_argument('--side',choices=['l','r'],required=True)
    p.add_argument('--additional-gripper-kg',type=float,nargs=2,required=True,help='URDF之外额外增加的左右末端质量，已包含夹爪时填0 0')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rows,sessions=read_sessions(a.csv)
    report=build_report(rows,a.csv,a.urdf,a.conversion,a.inventory,a.side,a.efficiency,a.additional_gripper_kg)
    report['sampling_sessions']=sessions
    report['independent_workspace_validation']=False
    with a.output.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print('物理先验候选：',a.output);print('RMS',report['cross_validation_rms_permille']);print('拒绝原因',report['rejection_reasons'])


if __name__=='__main__':main()
