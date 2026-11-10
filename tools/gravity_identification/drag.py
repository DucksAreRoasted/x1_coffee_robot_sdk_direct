"""双臂拖动控制律的离线参考实现，单位 N·m/rad/s；不连接运行控制栈。"""
import math


def torque(gravity_nm, velocity, previous_nm, dt, *, kd, limit_nm, slew_nm_s,
           fc=0., fv=0., smoothing_speed=.1, friction_gain=0.):
    values=[gravity_nm,velocity,previous_nm,dt,kd,limit_nm,slew_nm_s,fc,fv,smoothing_speed,friction_gain]
    if not all(math.isfinite(x) for x in values): raise ValueError('参数必须有限')
    if dt<=0 or dt>.02 or min(kd,fc,fv)<0 or min(limit_nm,slew_nm_s,smoothing_speed)<=0 or not 0<=friction_gain<=1:
        raise ValueError('无效周期或力矩参数')
    # 正向摩擦补偿斜率不得超过阻尼，防止在零速处引入负阻尼；默认禁用摩擦。
    if kd < friction_gain*(fc/smoothing_speed+fv): raise ValueError('摩擦补偿超过阻尼预算')
    if abs(gravity_nm)>limit_nm: raise ValueError('重力需求超过力矩预算，不能进入拖动')
    ff=friction_gain*(fc*math.tanh(velocity/smoothing_speed)+fv*velocity)
    desired=max(-limit_nm,min(limit_nm,gravity_nm-kd*velocity+ff))
    delta=max(-slew_nm_s*dt,min(slew_nm_s*dt,desired-previous_nm))
    return max(-limit_nm,min(limit_nm,previous_nm+delta))
