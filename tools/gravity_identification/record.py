#!/usr/bin/env python3
"""仅订阅电机状态，记录一个方向接近后人工保持的姿态；不会发布命令。"""
import argparse,csv,json,time,math,hashlib
from pathlib import Path
from model import JOINTS


def validated_joints(cfg, side):
    # 方向验证按臂隔离，右臂已验证不替代左臂验证。
    selected = [f'Joint_{side}{i}' for i in range(1,8)]
    if cfg.get('torque_signs_verified') is not True and side not in cfg.get('verified_sides',[]):
        raise ValueError('所选手臂的扭矩方向尚未确认')
    for j in selected:
        c=cfg['joints'][j]
        if not isinstance(c['rated_nm'],(int,float)) or not math.isfinite(c['rated_nm']) or c['rated_nm']<=0 or c['sign'] not in [-1,1]:
            raise ValueError('换算参数无效 '+j)
    return selected

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--conversion',type=Path,required=True)
    p.add_argument('--side',choices=['l','r'],required=True)
    p.add_argument('--pose-id',required=True);p.add_argument('--approach',type=int,choices=[-1,1],required=True)
    p.add_argument('--seconds',type=float,default=3);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); cfg=json.loads(a.conversion.read_text())
    try: selected=validated_joints(cfg,a.side)
    except (ValueError,KeyError) as e: p.error(str(e))
    if not math.isfinite(a.seconds) or not 0<a.seconds<=60: p.error('采样时长必须为 0~60 秒')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from x1_coffee_interfaces.msg import MotorStateArray
    rclpy.init();node=rclpy.create_node('x1_identification_recorder')
    columns=['conversion_sha256','time_ns','side','pose_id','approach','valid']+[k+j for k in ['q_','dq_','tau_','raw_torque_permille_','raw_current_permille_','raw_current_valid_','raw_torque_valid_'] for j in JOINTS]
    count=0;last_stamp=None
    conversion_hash=hashlib.sha256(a.conversion.read_bytes()).hexdigest()
    with a.output.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
        def receive(msg):
            nonlocal count,last_stamp
            states={s.joint_name:s for s in msg.states}
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if len(states)!=15 or set(states)!=set(JOINTS) or stamp<=0 or (last_stamp is not None and stamp<=last_stamp): return
            last_stamp=stamp
            row=dict(conversion_sha256=conversion_hash,time_ns=stamp,side=a.side,pose_id=a.pose_id,approach=a.approach,valid=1)
            for j in JOINTS:
                s=states[j];c=cfg['joints'][j]
                if not (s.position_valid and s.velocity_valid and (j not in selected or s.torque_valid) and s.enabled and s.position_unit=='rad') or s.alarm or s.follow_error or s.emergency_input_active: row['valid']=0
                row['q_'+j]=s.position;row['dq_'+j]=s.velocity
                # 保留原始驱动反馈，输出侧力矩换算核实后可离线重算。
                row['raw_torque_permille_'+j]=s.torque_permille
                row['raw_current_permille_'+j]=s.current_permille
                row['raw_current_valid_'+j]=int(s.current_valid)
                row['raw_torque_valid_'+j]=int(s.torque_valid)
                # 未辨识手臂只提供几何姿态，扭矩填零且不参与该臂拟合。
                row['tau_'+j]=s.torque_permille*.001*c['rated_nm']*c['sign'] if j in selected else 0.
            w.writerow(row);count+=1
        sub=node.create_subscription(MotorStateArray,'/x1/motor_states',receive,qos_profile_sensor_data)
        end=time.monotonic()+a.seconds
        try:
            while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.1)
        finally: node.destroy_node();rclpy.shutdown()
    if count<20: raise SystemExit('有效接收数量不足，不能用于辨识')
    print(f'已接收 {count} 条，拟合时仍需按 valid/速度过滤')

if __name__=='__main__':main()
