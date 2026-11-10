#!/usr/bin/env python3
"""单臂自动双向采样（可选左/右）；默认只检查路径和保存计划，--execute 才发送 PVT。"""
import argparse,csv,json,time,hashlib
from pathlib import Path
import numpy as np
from model import Model,JOINTS
from record import validated_joints
from current_identify import current_pairs,fit_current
from settling import stable_window


# 自动采样只在启动角附近移动，不做整圈等价角搜索。
LOCAL_RADIUS_RAD=.20
FIXED_DRIFT_RAD=.02


def arm_indices(side):
    # 不以镜像角度推定另一臂方向；这里只选择关节集合。
    if side not in ('l','r'):raise ValueError('无效臂名称')
    return list(range(1,8)) if side=='l' else list(range(8,15))


def check_local_pose(seed,q,side='r'):
    seed=np.asarray(seed,float);q=np.asarray(q,float)
    if seed.shape!=(15,) or q.shape!=(15,) or not np.isfinite([seed,q]).all():
        raise ValueError('局部范围检查收到无效角度')
    moving=arm_indices(side)
    delta=np.abs(q-seed)
    for j,name in enumerate(JOINTS):
        limit=LOCAL_RADIUS_RAD if j in moving else FIXED_DRIFT_RAD
        if delta[j]>limit+1e-12:
            raise ValueError(f'{name} 偏离启动角 {delta[j]:.4f} rad，超过局部范围 {limit:.2f} rad')


def make_plan(seed,model,side='r'):
    moving=arm_indices(side)
    seed=np.asarray(seed,float)
    if seed.shape!=(15,) or not np.isfinite(seed).all(): raise ValueError('初始角无效')
    bounds=[]
    for j,name in enumerate(JOINTS):
        limit=model.by_name[name].find('limit')
        if limit is None: raise ValueError('缺少 URDF 限位 '+name)
        lower,upper=float(limit.get('lower')),float(limit.get('upper'))
        if not np.isfinite([lower,upper]).all() or lower>=upper:
            raise ValueError('无效 URDF 限位 '+name)
        if not lower<=seed[j]<=upper:
            raise ValueError(f'起点越限 {name}: {seed[j]:.4f}，允许 [{lower:.4f}, {upper:.4f}]')
        bounds.append((lower,upper))
    low=[];high=[]
    for j in moving:
        # 目标为正反接近点预留 0.04 rad，再为软限位保留 0.05 rad。
        # 先求可用区间再采样，不裁剪已生成姿态，避免重复点和单侧接近。
        lo=max(seed[j]-.15,bounds[j][0]+.09)
        hi=min(seed[j]+.15,bounds[j][1]-.09)
        if hi-lo < .04:
            raise ValueError(f'{JOINTS[j]} 可用辨识区间仅 {hi-lo:.4f} rad，请先人工移到更居中的安全姿态')
        low.append(lo);high.append(hi)
    rng=np.random.default_rng(17); plan=[]
    for i in range(15):
        q=seed.copy();q[moving]=rng.uniform(low,high)
        for direction in (1,-1):
            approach=q.copy();approach[moving]-=direction*.04
            plan.extend([dict(pose_id=f'pose{i+1:02}',approach=direction,record=False,q=approach.tolist()),
                         dict(pose_id=f'pose{i+1:02}',approach=direction,record=True,q=q.tolist())])
    # 腰和未选手臂保持原位；只对会移动的手臂要求额外限位余量。
    for step in plan:
        check_local_pose(seed,step['q'],side)
        for j in moving:
            if not bounds[j][0]+.05-1e-12 <= step['q'][j] <= bounds[j][1]-.05+1e-12:
                raise ValueError('采样姿态接近或超过限位 '+JOINTS[j])
    return plan


def path_points(start,end):
    # 单目标 PVT 在关节空间的直线段上插值；以不超过 0.01 rad 的步距检查整段。
    count=max(1,int(np.ceil(np.max(np.abs(np.array(end)-start))/.01)))
    return [np.array(start)+(np.array(end)-start)*i/count for i in range(count+1)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--conversion',type=Path)
    p.add_argument('--side',choices=('l','r'),default='r')
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--execute',action='store_true');p.add_argument('--operator-ready',action='store_true')
    a=p.parse_args()
    if a.execute and not a.operator_ready:p.error('执行需要现场操作者确认 --operator-ready')
    if a.conversion is None:
        if a.side=='l':p.error('左臂须显式指定已验证方向的 --conversion 配置')
        a.conversion=Path(__file__).with_name('right_arm_session.json')
    cfg=json.loads(a.conversion.read_text());selected=validated_joints(cfg,a.side)
    fixed=[j for j in range(15) if j not in arm_indices(a.side)]
    model=Model(a.urdf,[1,1]);a.output_dir.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray,String
    from std_srvs.srv import Trigger
    from moveit_msgs.srv import GetStateValidity
    from x1_coffee_interfaces.msg import MotorStateArray
    rclpy.init();node=rclpy.create_node('x1_'+a.side+'_auto_sample')
    latest={};results={};fault=[];sent_sequences=set();last_stamp=0;received=0.;motion_started=False;seed=None
    def receive(msg):
        nonlocal last_stamp,received,latest
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        states={s.joint_name:s for s in msg.states}
        if len(msg.states)!=15 or set(states)!=set(JOINTS):fault.append('反馈轴集合错误');return
        if stamp<=last_stamp:return
        last_stamp=stamp;received=time.monotonic();latest=states
        for j,s in states.items():
            if not (s.enabled and s.position_valid and s.velocity_valid and s.position_unit=='rad' and (j not in selected or s.torque_valid)) or s.alarm or s.follow_error or s.emergency_input_active or not np.isfinite([s.position,s.velocity]).all():fault.append('无效反馈/故障 '+j)
    def result(msg):
        parts=msg.data.split(' ',2)
        try: results[int(float(parts[0]))]=parts[1]
        except (ValueError,IndexError):fault.append('PVT 结果格式错误')
    node.create_subscription(MotorStateArray,'/x1/motor_states',receive,qos_profile_sensor_data)
    node.create_subscription(String,'/x1/pvt_result',result,10)
    pub=node.create_publisher(Float64MultiArray,'/x1/pvt_move',10)
    def competing_command(msg):
        if motion_started and (not msg.data or not np.isfinite(msg.data[0]) or msg.data[0] not in sent_sequences):
            fault.append('检测到其他 PVT 命令，取消采样')
    node.create_subscription(Float64MultiArray,'/x1/pvt_move',competing_command,10)
    validity=node.create_client(GetStateValidity,'/check_state_validity')
    stop=node.create_client(Trigger,'/x1/cancel_pvt')
    def spin():rclpy.spin_once(node,timeout_sec=.02)
    def health():
        if fault:raise RuntimeError(fault[0])
        if time.monotonic()-received>.5:raise RuntimeError('电机反馈过期')
        if seed is not None:check_local_pose(seed,qnow(),a.side)
    def call(client,request):
        future=client.call_async(request);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:spin();health()
        if not future.done():raise RuntimeError('服务超时 '+client.srv_name)
        return future.result()
    def qnow():return np.array([latest[j].position for j in JOINTS])
    def collision(start,end):
        for q in path_points(start,end):
            req=GetStateValidity.Request();req.robot_state.joint_state.name=JOINTS
            req.robot_state.joint_state.position=q.tolist()
            if not call(validity,req).valid:raise RuntimeError('MoveIt 路径采样点无效/碰撞')
    try:
        end=time.monotonic()+10
        while not latest and time.monotonic()<end:spin()
        health();seed=qnow();plan=make_plan(seed,model,a.side)
        if not validity.wait_for_service(timeout_sec=3):raise RuntimeError('缺少 MoveIt /check_state_validity，拒绝运行')
        previous=seed
        for step in plan:collision(previous,step['q']);previous=np.array(step['q'])
        (a.output_dir/'plan.json').write_text(json.dumps({'side':a.side,'seed':seed.tolist(),'steps':plan,'collision_step_rad':.01,'local_radius_rad':LOCAL_RADIUS_RAD,'fixed_drift_rad':FIXED_DRIFT_RAD,'urdf_sha256':hashlib.sha256(a.urdf.read_bytes()).hexdigest()},indent=2))
        print('计划已检查并保存：',a.output_dir/'plan.json',flush=True)
        if not a.execute:return
        if not stop.wait_for_service(timeout_sec=3):raise RuntimeError('缺少 PVT 取消服务')
        if pub.get_subscription_count()<2:
            raise RuntimeError('没有硬件 PVT 订阅者')
        if np.max(np.abs(qnow()-seed))>.02:raise RuntimeError('检查期间机器人位置改变')
        rows=[];cfg_hash=hashlib.sha256(a.conversion.read_bytes()).hexdigest();seq=int(time.time()*1000)
        for step in plan:
            health();target=np.array(step['q'])
            if np.max(np.abs(qnow()[fixed]-seed[fixed]))>.02:raise RuntimeError('腰或未选手臂漂移')
            collision(qnow(),target)
            duration=max(1.,float(np.max(np.abs(target-qnow())))*2/.08)
            seq+=1;msg=Float64MultiArray();msg.data=[float(seq),duration,1.,*target.tolist()]
            sent_sequences.add(seq);motion_started=True;pub.publish(msg)
            deadline=time.monotonic()+duration+8;stable_since=None
            while time.monotonic()<deadline:
                spin();health()
                if np.max(np.abs(qnow()[fixed]-seed[fixed]))>.02:raise RuntimeError('腰或未选手臂漂移')
                if seq in results and results[seq]!='ok':raise RuntimeError('PVT 拒绝 '+str(results[seq]))
                stable=np.max(np.abs(qnow()-target))<.01 and max(abs(s.velocity) for s in latest.values())<.02
                stable_since=(stable_since or time.monotonic()) if stable else None
                if seq in results and stable_since and time.monotonic()-stable_since>=1:break
            else:raise RuntimeError('未测量确认到位/停稳')
            if not step['record']:continue
            batch=[];end=time.monotonic()+15;stamp_seen=0
            while time.monotonic()<end:
                spin();health()
                if np.max(np.abs(qnow()-target))>.01 or max(abs(s.velocity) for s in latest.values())>.02:raise RuntimeError('采样期间移动')
                if last_stamp==stamp_seen:continue
                stamp_seen=last_stamp
                row=dict(conversion_sha256=cfg_hash,time_ns=last_stamp,side=a.side,pose_id=step['pose_id'],approach=step['approach'],valid=1)
                for j in JOINTS:
                    s=latest[j];row['q_'+j]=s.position;row['dq_'+j]=s.velocity
                    # 同时保留未经 Nm 换算的电流和扭矩反馈。
                    row['raw_torque_permille_'+j]=s.torque_permille
                    row['raw_current_permille_'+j]=s.current_permille
                    row['raw_current_valid_'+j]=int(s.current_valid)
                    row['raw_torque_valid_'+j]=int(s.torque_valid)
                    row['tau_'+j]=s.torque_permille*.001*cfg['joints'][j]['rated_nm']*cfg['joints'][j]['sign'] if j in selected else 0.
                batch.append(row)
                # 滚动保留至少3秒反馈；超时不把尚未稳定的保持力矩作为重力样本。
                while len(batch)>1 and int(batch[-1]['time_ns'])-int(batch[1]['time_ns'])>=3000000000:
                    batch.pop(0)
                if stable_window(batch, selected):break
            else:
                if batch:
                    with (a.output_dir/f"{step['pose_id']}_{step['approach']}.unsettled.csv").open('x',newline='') as f:
                        writer=csv.DictWriter(f,fieldnames=list(batch[0]));writer.writeheader()
                        # 失败窗口可用于诊断，但不能被通配符误纳入有效辨识样本。
                        writer.writerows(dict(r,valid=0) for r in batch)
                raise RuntimeError('15秒内电流/力矩反馈未稳定，已保存无效诊断窗口')
            if len(batch)<20:raise RuntimeError('采样数量不足')
            with (a.output_dir/f"{step['pose_id']}_{step['approach']}.csv").open('x',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(batch[0]));writer.writeheader();writer.writerows(batch)
            rows.extend(batch);print('完成',step['pose_id'],step['approach'],flush=True)
        poses,quality_reasons=current_pairs(rows,cfg,a.side)
        report=fit_current(model,poses,a.side,10.)
        report['rejection_reasons'].extend(quality_reasons)
        report['accepted_offline']=not report['rejection_reasons']
        report.update(conversion_sha256=cfg_hash,urdf_sha256=hashlib.sha256(a.urdf.read_bytes()).hexdigest(),runtime_applied=False,gripper_kg=[1,1],waist_mode='position_hold')
        (a.output_dir/'candidate.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
        print('完成，候选未应用：',a.output_dir/'candidate.json')
        print('电流模型离线验证：',report['accepted_offline'],report['rejection_reasons'],flush=True)
    except BaseException as exc:
        if motion_started:
            # 异常后请求取消保持，不在故障中发送新的位置目标；软件停止无法替代硬件急停。
            future=stop.call_async(Trigger.Request());end=time.monotonic()+2
            while not future.done() and time.monotonic()<end:spin()
            accepted=future.done() and future.exception() is None and future.result().success
            stopped=False;stable=None;end=time.monotonic()+2
            while accepted and time.monotonic()<end:
                spin()
                quiet=time.monotonic()-received<.5 and latest and max(abs(s.velocity) for s in latest.values())<.02
                stable=(stable or time.monotonic()) if quiet else None
                if stable and time.monotonic()-stable>.3:stopped=True;break
            print('取消后测量停稳：',bool(stopped),'；未确认时请使用硬件急停',flush=True)
        (a.output_dir/'failure.txt').write_text(str(exc));raise
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
