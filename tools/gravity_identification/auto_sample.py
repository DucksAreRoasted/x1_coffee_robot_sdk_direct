#!/usr/bin/env python3
"""单臂自动双向采样（可选左/右）；默认只检查路径和保存计划，--execute 才发送 PVT。"""
import argparse,csv,json,time,hashlib,signal
from pathlib import Path
import numpy as np
from model import Model,JOINTS
from record import validated_joints
from current_identify import current_pairs,fit_current
from physical_current_identify import build_report,nominal_gains
from settling import stable_window,stability_reason,StableConfirmation


# 自动采样只在启动角附近移动，不做整圈等价角搜索。
LOCAL_RADIUS_RAD=.20
FIXED_DRIFT_RAD=.02


def arm_indices(side):
    # 不以镜像角度推定另一臂方向；这里只选择关节集合。
    if side not in ('l','r'):raise ValueError('无效臂名称')
    return list(range(1,8)) if side=='l' else list(range(8,15))


def check_local_pose(seed,q,side='r',radius=LOCAL_RADIUS_RAD):
    seed=np.asarray(seed,float);q=np.asarray(q,float)
    if seed.shape!=(15,) or q.shape!=(15,) or not np.isfinite([seed,q]).all():
        raise ValueError('局部范围检查收到无效角度')
    moving=arm_indices(side)
    delta=np.abs(q-seed)
    for j,name in enumerate(JOINTS):
        limit=radius if j in moving else FIXED_DRIFT_RAD
        if delta[j]>limit+1e-12:
            raise ValueError(f'{name} 偏离启动角 {delta[j]:.4f} rad，超过局部范围 {limit:.2f} rad')


def make_plan(seed,model,side='r',pose_count=15,target_radius=.15,clearance=None):
    if not np.isfinite(target_radius) or not .15<=target_radius<=.35:raise ValueError('采样半径须为0.15到0.35rad')
    if not isinstance(pose_count,int) or not 15<=pose_count<=100:
        raise ValueError('采样姿态数须为15到100')
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
        lo=max(seed[j]-target_radius,bounds[j][0]+.09)
        hi=min(seed[j]+target_radius,bounds[j][1]-.09)
        if hi-lo < .04:
            raise ValueError(f'{JOINTS[j]} 可用辨识区间仅 {hi-lo:.4f} rad，请先人工移到更居中的安全姿态')
        low.append(lo);high.append(hi)
    rng=np.random.default_rng(17); plan=[]
    if clearance is not None:clearance(seed)
    attempts=0
    while len(plan)<pose_count*4:
        attempts+=1
        if attempts>1000:raise ValueError('无法生成足够远离躯干的姿态，请先调整起点；不会自动缩小净距')
        i=len(plan)//4;q=seed.copy();q[moving]=rng.uniform(low,high);block=[]
        for direction in (1,-1):
            approach=q.copy();approach[moving]-=direction*.04
            block.extend([dict(pose_id=f'pose{i+1:02}',approach=direction,record=False,q=approach.tolist()),
                          dict(pose_id=f'pose{i+1:02}',approach=direction,record=True,q=q.tolist())])
        if clearance is not None:
            previous=plan[-1]['q'] if plan else seed
            try:
                for step in block:
                    for point in path_points(previous,step['q']):clearance(point)
                    previous=step['q']
            except ValueError:continue
        plan.extend(block)
    # 腰和未选手臂保持原位；只对会移动的手臂要求额外限位余量。
    for step in plan:
        check_local_pose(seed,step['q'],side,target_radius+.05)
        for j in moving:
            if not bounds[j][0]+.05-1e-12 <= step['q'][j] <= bounds[j][1]-.05+1e-12:
                raise ValueError('采样姿态接近或超过限位 '+JOINTS[j])
    return plan


def movement_seconds(start, end):
    return max(1.,float(np.max(np.abs(np.asarray(end)-start)))*2/.08)


def optimize_visit_order(seed, plan):
    """只重排完整正反配对块；保留全部姿态、接近方向与运动速度。"""
    blocks=[plan[i:i+4] for i in range(0,len(plan),4)]
    def cost(steps):
        total=0.;previous=seed
        for step in steps:
            total+=movement_seconds(previous,step['q']);previous=step['q']
        return total
    pending=blocks[:];ordered=[];previous=seed
    while pending:
        i=min(range(len(pending)),key=lambda k:movement_seconds(previous,pending[k][0]['q']))
        block=pending.pop(i);ordered.extend(block);previous=block[-1]['q']
    before=cost(plan);after=cost(ordered)
    # 贪心并不保证全局更短，比较总时长后才采用。
    return (ordered,before-after) if after<before else (plan,0.)


def path_points(start,end):
    # 单目标 PVT 在关节空间的直线段上插值；以不超过 0.01 rad 的步距检查整段。
    count=max(1,int(np.ceil(np.max(np.abs(np.array(end)-start))/.01)))
    return [np.array(start)+(np.array(end)-start)*i/count for i in range(count+1)]


def angle_text(values, side):
    """终端用角度辅助观察；采样与控制内部仍使用弧度。"""
    return '  '.join(f'{side}{j+1}={v:+.1f}°' for j,v in enumerate(np.degrees(values)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--conversion',type=Path)
    p.add_argument('--side',choices=('l','r'),default='r')
    p.add_argument('--inventory',type=Path,default=Path(__file__).with_name('motor_inventory.json'))
    p.add_argument('--efficiency',type=float,default=.7,help='电流换算效率先验假设，并非实测值')
    p.add_argument('--additional-gripper-kg',type=float,nargs=2,default=[0.,0.],help='URDF之外额外负载；当前CAD已包含夹爪')
    p.add_argument('--target-radius',type=float,default=.35,help='采样目标相对起点最大角差，默认0.35rad')
    p.add_argument('--poses',type=int,default=30,help='双向配对姿态数，默认30，范围15到100')
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--execute',action='store_true');p.add_argument('--operator-ready',action='store_true')
    a=p.parse_args()
    if not 15<=a.poses<=100:p.error('--poses须在15到100之间')
    if a.execute and not a.operator_ready:p.error('执行需要现场操作者确认 --operator-ready')
    if a.conversion is None:
        if a.side=='l':p.error('左臂须显式指定已验证方向的 --conversion 配置')
        a.conversion=Path(__file__).with_name('right_arm_session.json')
    # 在启用ROS和发送运动前保存输入快照；本轮后续计算只使用快照。
    source_paths={name:getattr(a,name) for name in ('urdf','conversion','inventory')}
    source_bytes={name:path.read_bytes() for name,path in source_paths.items()}
    cfg=json.loads(source_bytes['conversion']);selected=validated_joints(cfg,a.side)
    # 先核实模型输入，避免采完后才发现该臂缺少电机参数。
    nominal_gains(json.loads(source_bytes['inventory']),a.side,a.efficiency)
    if not np.isfinite(a.additional_gripper_kg).all() or min(a.additional_gripper_kg)<0:
        p.error('额外负载必须为有限非负质量')
    print(f'[辨识模型] URDF物理先验；额外负载 {a.additional_gripper_kg}kg；效率假设 {a.efficiency}，尚未经动态验证。',flush=True)
    fixed=[j for j in range(15) if j not in arm_indices(a.side)]
    a.output_dir.mkdir(parents=True,exist_ok=False)
    inputs=a.output_dir/'inputs';inputs.mkdir()
    manifest={}
    for name,content in source_bytes.items():
        target=inputs/(name+('.urdf' if name=='urdf' else '.json'))
        target.write_bytes(content);setattr(a,name,target)
        manifest[name]=dict(source=str(source_paths[name].resolve()),sha256=hashlib.sha256(content).hexdigest())
    (inputs/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    model=Model(a.urdf,[1,1])
    from ament_index_python.packages import get_package_share_directory
    from body_clearance import BodyClearance
    clearance=BodyClearance(a.urdf,get_package_share_directory('arm_describle'))
    def body_check(q):clearance.check(model,q,a.side)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray,String
    from std_srvs.srv import Trigger
    from moveit_msgs.srv import GetStateValidity
    from x1_coffee_interfaces.msg import MotorStateArray
    from rclpy.signals import SignalHandlerOptions
    # Ctrl-C先进入取消PVT流程，保留ROS上下文用于发送停止请求。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('x1_'+a.side+'_auto_sample')
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
        if seed is not None:check_local_pose(seed,qnow(),a.side,a.target_radius+.05)
        if seed is not None:body_check(qnow())
    def call(client,request):
        future=client.call_async(request);end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:spin();health()
        if not future.done():raise RuntimeError('服务超时 '+client.srv_name)
        return future.result()
    def qnow():return np.array([latest[j].position for j in JOINTS])
    def collision(start,end):
        for q in path_points(start,end):
            body_check(q)
            req=GetStateValidity.Request();req.robot_state.joint_state.name=JOINTS
            req.robot_state.joint_state.position=q.tolist()
            if not call(validity,req).valid:raise RuntimeError('MoveIt 路径采样点无效/碰撞')
    try:
        print('[准备] 等待15轴有效反馈，最多10秒；n可取消。',flush=True)
        end=time.monotonic()+10
        while not latest and time.monotonic()<end:spin()
        health();seed=qnow();plan=make_plan(seed,model,a.side,a.poses,a.target_radius,body_check)
        ordered,saved=optimize_visit_order(seed,plan)
        try:
            previous=seed
            for step in ordered:
                for point in path_points(previous,step['q']):body_check(point)
                previous=step['q']
            plan=ordered
        except ValueError:saved=0.  # 重排路径贴近躯干时保留已筛选的原顺序。
        print(f'[提速] 保留{a.poses}个双向姿态；访问顺序预计减少运动 {saved:.1f}s；非采样点停稳确认0.2s。',flush=True)
        if not validity.wait_for_service(timeout_sec=3):raise RuntimeError('缺少 MoveIt /check_state_validity，拒绝运行')
        moving=arm_indices(a.side)
        print(f'[计划] {a.side}臂，{a.poses}个姿态 × 正反2次采样，共{len(plan)}段运动。',flush=True)
        planned=np.array([step['q'] for step in plan])
        print('[计划角度跨度] '+angle_text(np.ptp(planned[:,moving],axis=0),a.side),flush=True)
        previous=seed
        for index,step in enumerate(plan,1):
            if index==1 or index%4==0:
                print(f'[路径检查] {index}/{len(plan)}段',flush=True)
            collision(previous,step['q']);previous=np.array(step['q'])
        (a.output_dir/'plan.json').write_text(json.dumps({'side':a.side,'pose_count':a.poses,'seed':seed.tolist(),'steps':plan,'collision_step_rad':.01,'local_radius_rad':a.target_radius+.05,'target_radius_rad':a.target_radius,'body_clearance_m':clearance.margin,'fixed_drift_rad':FIXED_DRIFT_RAD,'urdf_sha256':hashlib.sha256(a.urdf.read_bytes()).hexdigest()},indent=2))
        print('计划已检查并保存：',a.output_dir/'plan.json',flush=True)
        if not a.execute:return
        if not stop.wait_for_service(timeout_sec=3):raise RuntimeError('缺少 PVT 取消服务')
        if pub.get_subscription_count()<2:
            raise RuntimeError('没有硬件 PVT 订阅者')
        if np.max(np.abs(qnow()-seed))>.02:raise RuntimeError('检查期间机器人位置改变')
        rows=[];cfg_hash=hashlib.sha256(a.conversion.read_bytes()).hexdigest();seq=int(time.time()*1000)
        completed=0;started=time.monotonic()
        observed_low=qnow()[moving].copy();observed_high=observed_low.copy()
        for index,step in enumerate(plan,1):
            health();target=np.array(step['q'])
            label=f"{step['pose_id']} {'正向' if step['approach']==1 else '反向'} {'采样点' if step['record'] else '接近点'}"
            print(f'[运动 {index}/{len(plan)}] {label}；正在检查当前路径',flush=True)
            if np.max(np.abs(qnow()[fixed]-seed[fixed]))>.02:raise RuntimeError('腰或未选手臂漂移')
            collision(qnow(),target)
            duration=movement_seconds(qnow(),target)
            print(f'[下发] 计划运动时间 {duration:.1f}s；目标角度 '+angle_text(target[moving],a.side),flush=True)
            print('[本段目标变化] '+angle_text((target-qnow())[moving],a.side),flush=True)
            seq+=1;msg=Float64MultiArray();msg.data=[float(seq),duration,1.,*target.tolist()]
            sent_sequences.add(seq);motion_started=True;pub.publish(msg)
            sent_at=time.monotonic()
            arrival_quiet=1. if step['record'] else .2
            deadline=time.monotonic()+duration+8;stable_since=None;next_notice=0.
            while time.monotonic()<deadline:
                spin();health()
                if np.max(np.abs(qnow()[fixed]-seed[fixed]))>.02:raise RuntimeError('腰或未选手臂漂移')
                if seq in results and results[seq]!='ok':raise RuntimeError('PVT 拒绝 '+str(results[seq]))
                measured=qnow()[moving]
                observed_low=np.minimum(observed_low,measured);observed_high=np.maximum(observed_high,measured)
                if time.monotonic()>=next_notice:
                    worst=int(np.argmax(np.abs(qnow()-target)))
                    print(f'[等待到位] 最大误差 {JOINTS[worst]}={np.degrees(abs(qnow()[worst]-target[worst])):.2f}°；'
                          f"PVT={'已接受' if seq in results else '等待响应'}；实际角度 "+angle_text(measured,a.side),flush=True)
                    next_notice=time.monotonic()+2
                stable=np.max(np.abs(qnow()-target))<.01 and max(abs(s.velocity) for s in latest.values())<.02
                stable_since=(stable_since or time.monotonic()) if stable else None
                if (seq in results and stable_since and time.monotonic()-stable_since>=arrival_quiet
                        and time.monotonic()-sent_at>=duration):break
            else:raise RuntimeError('未测量确认到位/停稳')
            if not step['record']:continue
            print('[保持采样] 已到位，等待3秒电流/扭矩窗口连续达标3秒，最多30秒。',flush=True)
            batch=[];hold_started=time.monotonic();end=hold_started+30;stamp_seen=0;next_notice=hold_started+2
            confirmation=StableConfirmation(3.)
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
                reason=stability_reason(batch,selected)
                if confirmation.update(time.monotonic(),not reason):break
                if time.monotonic()>=next_notice:
                    span=(int(batch[-1]['time_ns'])-int(batch[0]['time_ns']))/1e9
                    detail=reason or f'窗口已达标，连续确认 {time.monotonic()-confirmation.since:.1f}/3.0秒'
                    print(f'[等待稳定] 已等 {time.monotonic()-hold_started:.1f}/30s；窗口 {span:.1f}s、{len(batch)}条；{detail}。',flush=True)
                    next_notice=time.monotonic()+2
            else:
                if batch:
                    with (a.output_dir/f"{step['pose_id']}_{step['approach']}.unsettled.csv").open('x',newline='') as f:
                        writer=csv.DictWriter(f,fieldnames=list(batch[0]));writer.writeheader()
                        # 失败窗口可用于诊断，但不能被通配符误纳入有效辨识样本。
                        writer.writerows(dict(r,valid=0) for r in batch)
                raise RuntimeError('30秒内电流/力矩反馈未连续稳定，已保存无效诊断窗口')
            if len(batch)<20:raise RuntimeError('采样数量不足')
            with (a.output_dir/f"{step['pose_id']}_{step['approach']}.csv").open('x',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(batch[0]));writer.writeheader();writer.writerows(batch)
            rows.extend(batch);completed+=1
            print(f'[采样进度] {completed}/{a.poses*2} ({100*completed/(a.poses*2):.0f}%)；'
                  f'{label} 已保存{len(batch)}条，累计用时 {(time.monotonic()-started)/60:.1f}分钟。',flush=True)
            print('[累计实测角度跨度] '+angle_text(observed_high-observed_low,a.side),flush=True)
        print('[辨识] 采样完成，正在进行整姿态五折验证；候选不会自动加载。',flush=True)
        poses,quality_reasons=current_pairs(rows,cfg,a.side)
        report=fit_current(model,poses,a.side,10.)
        report['rejection_reasons'].extend(quality_reasons)
        report['accepted_offline']=not report['rejection_reasons']
        report.update(conversion_sha256=cfg_hash,urdf_sha256=hashlib.sha256(a.urdf.read_bytes()).hexdigest(),runtime_applied=False,gripper_kg=[1,1],waist_mode='position_hold')
        # 旧自由电流拟合仅保留作对照，键盘默认候选改为物理先验模型。
        (a.output_dir/'legacy_current_candidate.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
        report=build_report(rows,sorted(a.output_dir.glob('pose*.csv')),a.urdf,a.conversion,
                            a.inventory,a.side,a.efficiency,a.additional_gripper_kg)
        report['waist_mode']='position_hold'
        (a.output_dir/'candidate.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
        print('完成，候选未应用：',a.output_dir/'candidate.json')
        print('电流模型离线验证：',report['accepted_offline'],report['rejection_reasons'],flush=True)
        print('[各轴验证RMS] '+'  '.join(f'{a.side}{j+1}={v:.2f}‰'+(' 超标' if v>10 else '')
              for j,v in enumerate(report['cross_validation_rms_permille'])),flush=True)
    except BaseException as exc:
        # 取消过程中第二次Ctrl-C不能先销毁ROS上下文。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
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
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
