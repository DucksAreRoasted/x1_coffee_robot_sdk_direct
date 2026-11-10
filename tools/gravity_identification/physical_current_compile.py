"""将共享URDF物理参数编译为控制器几何基函数系数，不改变实物模型。"""
import numpy as np


def compile_coefficients(candidate, runtime_model):
    if (candidate.get('schema')!='x1_physical_current_candidate_v1' or
            candidate.get('accepted_offline') is not True or candidate.get('rejection_reasons')!=[]):
        raise ValueError('物理候选未通过离线验证')
    if candidate.get('payload_semantics')!='additional_to_urdf_link_inertials_at_tcp':
        raise ValueError('物理候选缺少额外负载语义')
    if candidate.get('bound_hits')!=[] or any(candidate.get('fold_bound_hits',[['missing']])):
        raise ValueError('物理候选触及约束或缺少边界检查')
    side=candidate.get('side')
    if side not in ('l','r'):raise ValueError('无效臂名称')
    def vector(key,shape):
        result=np.asarray(candidate[key],float)
        if result.shape!=shape or not np.isfinite(result).all():raise ValueError('物理参数无效 '+key)
        return result
    mass_scale=vector('mass_scale',(7,));com_delta=vector('com_delta_m',(7,3))
    nominal=vector('nominal_current_per_nm',(7,));gain=vector('identified_current_per_nm',(7,))
    theta=vector('shared_inertial_parameters',(28,));payload=vector('additional_gripper_kg',(2,))
    if (np.any(mass_scale<.7) or np.any(mass_scale>1.3) or np.any(abs(com_delta)>.03) or
            np.any(nominal<=0) or np.any(gain<nominal*.5) or np.any(gain>nominal*2) or np.any(payload<0)):
        raise ValueError('物理参数超出辨识约束')
    _,_,prior,_=runtime_model.regressor(np.zeros(15),side)
    original=prior.reshape(7,4);mass=original[:,0]*mass_scale
    center=original[:,1:]/original[:,0,None]+com_delta
    reconstructed=np.column_stack((mass,mass[:,None]*center)).ravel()
    if not np.allclose(theta,reconstructed,rtol=1e-9,atol=1e-10):
        raise ValueError('共享质量/一阶矩与URDF修正参数不一致')
    # 运行时第29列必须仅为该臂TCP处1kg的贡献。其他固定负载不能误随额外夹爪倍率缩放。
    selected={runtime_model.by_name[f'Joint_{side}{j}'].find('child').get('link') for j in range(1,8)}
    tip='left_gripper_tcp' if side=='l' else 'right_gripper_tcp'
    frames,_=runtime_model.geometry(np.zeros(15));first=1 if side=='l' else 8
    for name,(_,_,ancestors) in frames.items():
        mass_value,com_value,_=runtime_model.links[name]
        if first not in ancestors or name in selected:continue
        if name==tip:
            if mass_value!=1. or np.any(com_value!=0):raise ValueError('运行基函数TCP须为单位点质量')
        elif mass_value!=0:raise ValueError('运行基函数含额外固定质量，不能直接编译')
    # payload=0时第29列为0：URDF末端连杆质量仍保留，只取消重复添加的TCP质量。
    return gain[:,None]*np.r_[theta,payload[0 if side=='l' else 1]][None,:]
