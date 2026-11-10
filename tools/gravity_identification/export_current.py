#!/usr/bin/env python3
"""导出通过离线验证的电流候选供现场验证；不自动启动或批准实机运行。"""
import io
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from record import validated_joints
from model import Model
from physical_current_compile import compile_coefficients


def fingerprint(raw):
    value=14695981039346656037
    for byte in raw:value=((value^byte)*1099511628211)&0xffffffffffffffff
    return value


def arm_record(candidate, cfg, urdf_raw, conversion_raw, *, kd, limits, slew, radius):
    if candidate.get('schema') not in ('x1_current_gravity_v1','x1_physical_current_candidate_v1') or candidate.get('accepted_offline') is not True or candidate.get('rejection_reasons')!=[]:
        raise ValueError('候选未通过离线验证')
    physical=candidate['schema']=='x1_physical_current_candidate_v1'
    if not physical and (candidate.get('unit')!='joint_signed_current_permille' or candidate.get('basis')!='urdf_mass_first_moment_28_plus_known_gravity' or candidate.get('gripper_kg')!=[1,1]):
        raise ValueError('单位、几何基函数或夹爪配置不符')
    if candidate.get('urdf_sha256')!=hashlib.sha256(urdf_raw).hexdigest() or candidate.get('conversion_sha256')!=hashlib.sha256(conversion_raw).hexdigest():
        raise ValueError('URDF或方向换算配置与候选来源不同')
    side=candidate['side']
    if side not in ('l','r'):raise ValueError('无效臂名称')
    joints=validated_joints(cfg,side)
    beta=compile_coefficients(candidate,Model(io.BytesIO(urdf_raw),[1,1])) if physical else np.array(candidate['coefficients'],float)
    poses=np.array(candidate['sample_poses'],float)
    if beta.shape!=(7,29) or poses.ndim!=2 or poses.shape[1]!=15 or not 15<=len(poses)<=4096 or not np.isfinite(beta).all() or not np.isfinite(poses).all():
        raise ValueError('系数或采样姿态无效')
    rms=np.array(candidate['cross_validation_rms_permille'],float)
    if rms.shape!=(7,) or not np.isfinite(rms).all() or np.any(rms<0) or np.any(rms>10.):
        raise ValueError('控制参数导出要求各轴五折RMS不超过10千分比')
    arrays=[np.asarray(v,float) for v in (kd,limits,slew)]
    if any(v.shape!=(7,) or not np.isfinite(v).all() or np.any(v<=0) for v in arrays) or np.any(arrays[1]>1000) or not np.isfinite(radius) or not 0<radius<=.20:
        raise ValueError('阻尼、电流预算、斜率或邻域无效')
    return dict(side=side,beta=beta,poses=poses,direction=[cfg['joints'][j]['sign'] for j in joints],arrays=arrays,radius=radius)



def arm_options(args, side, dual):
    """双臂必须分别声明预算；单臂兼容原有命令行参数。"""
    prefix = 'left' if side == 'l' else 'right'
    result = {}
    for name in ('kd', 'limits', 'slew'):
        specific = getattr(args, prefix + '_' + name)
        common = getattr(args, name)
        if dual and common is not None:
            raise ValueError('双臂导出请分别指定 --left-* 和 --right-* 预算')
        if specific is not None and common is not None:
            raise ValueError('同一参数不能同时指定通用值和单臂值: ' + name)
        result[name] = specific if specific is not None else common
        if result[name] is None:
            raise ValueError('缺少 ' + prefix + ' 的 ' + name)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('candidate',type=Path,nargs='+')
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--conversion',type=Path,help='共用方向换算配置；可用单臂选项覆盖')
    for side in ('left','right'):
        p.add_argument('--'+side+'-conversion',type=Path)
        for name in ('kd','limits','slew'):
            p.add_argument('--'+side+'-'+name,type=float,nargs=7)
    p.add_argument('--kd',type=float,nargs=7,help='各轴阻尼，千分比秒/弧度')
    p.add_argument('--limits',type=float,nargs=7,help='各轴绝对电流预算，千分比')
    p.add_argument('--slew',type=float,nargs=7,help='各轴电流变化率，千分比/秒')
    p.add_argument('--radius',type=float,default=.03)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();urdf=a.urdf.read_bytes()
    if not 1 <= len(a.candidate) <= 2:raise ValueError('需要一个或两个不同臂候选')
    records=[];seen=set()
    for path in a.candidate:
        raw=path.read_bytes();candidate=json.loads(raw)
        side=candidate.get('side')
        if side not in ('l','r'):raise ValueError('无效臂名称')
        options=arm_options(a,side,len(a.candidate)==2)
        conversion_path=getattr(a,('left' if side=='l' else 'right')+'_conversion') or a.conversion
        if conversion_path is None:raise ValueError('缺少 '+side+' 的方向换算配置')
        conversion=conversion_path.read_bytes();cfg=json.loads(conversion)
        r=arm_record(candidate,cfg,urdf,conversion,**options,radius=a.radius)
        if r['side'] in seen:raise ValueError('重复臂候选')
        seen.add(r['side']);records.append((r,hashlib.sha256(raw).hexdigest()))
    lines=['X1_CURRENT_GRAVITY_V1 HARDWARE_VALIDATION',str(fingerprint(urdf)),str(len(records))]
    def nums(values):return ' '.join(format(float(v),'.17g') for v in values)
    for r,digest in records:
        lines.extend([r['side']+' '+digest,str(r['radius']),nums(r['direction'])])
        lines.extend(nums(v) for v in r['arrays'])
        lines.extend(nums(row) for row in r['beta'])
        lines.append(str(len(r['poses'])))
        lines.extend(nums(q) for q in r['poses'])
    lines.append('END')
    with a.output.open('x') as stream:stream.write('\n'.join(lines)+'\n')
    print('仅供现场验证，尚未加载：',a.output)


if __name__=='__main__':main()
