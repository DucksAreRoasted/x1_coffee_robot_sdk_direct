#!/usr/bin/env python3
"""电流补偿无人接触保持验证；默认只读检查，执行需明确现场确认。"""
import argparse
import hashlib
import json
import signal
import time
from pathlib import Path
import numpy as np
from model import JOINTS


def check_sample(sample, seed, drift=.02, speed=.05):
    # 不把缺失/非有限反馈当成零；阈值只用于终止试验，不代表认证安全功能。
    if not sample['healthy']:raise RuntimeError('电机反馈/使能/报警异常')
    q=np.asarray(sample['q']);dq=np.asarray(sample['dq']);current=np.asarray(sample['current'])
    if q.shape!=(15,) or dq.shape!=(15,) or current.shape!=(15,) or not np.isfinite(np.r_[q,dq,current]).all():
        raise RuntimeError('反馈维度或数值无效')
    moved=abs(q-seed)
    if np.max(moved)>drift:
        i=int(np.argmax(moved));raise RuntimeError(f'{JOINTS[i]} 漂移 {moved[i]:.5f}rad > {drift}rad')
    if np.max(abs(dq))>speed:
        i=int(np.argmax(abs(dq)));raise RuntimeError(f'{JOINTS[i]} 速度 {dq[i]:.5f}rad/s > {speed}rad/s')


def matching_candidate(message, digest, side):
    # 暂时只允许单臂无人接触验证，不能因文件额外包含另一臂而意外同时松开。
    tokens=message.split()
    return f'candidate_{side}={digest}' in tokens and not any(
        token.startswith('candidate_'+('l' if side=='r' else 'r')+'=') for token in tokens)


def summarize_trace(rows, seed):
    selected=[r for r in rows if r['stage'] in ('entering','holding')]
    if not selected:raise ValueError('没有补偿期间的测量数据')
    for row in selected:check_sample(row,seed)
    times=np.array([r['received_monotonic'] for r in selected])
    if len(times)<2 or not np.isfinite(times).all() or np.any(np.diff(times)<=0) or np.any(np.diff(times)>.5):raise ValueError('补偿期间反馈记录不连续')
    q=np.array([r['q'] for r in selected]);dq=np.array([r['dq'] for r in selected]);current=np.array([r['current'] for r in selected])
    return dict(sample_count=len(selected),observed_seconds=float(times[-1]-times[0]),
        max_drift_rad=np.max(abs(q-seed),axis=0).tolist(),
        max_speed_rad_s=np.max(abs(dq),axis=0).tolist(),
        peak_current_permille=np.max(abs(current),axis=0).tolist(),
        final_displacement_rad=(q[-1]-seed).tolist(),joint_names=JOINTS)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=5.)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--execute',action='store_true');p.add_argument('--operator-ready',action='store_true')
    a=p.parse_args()
    if a.execute and not a.operator_ready:p.error('需要现场操作者确认 --operator-ready')
    if not np.isfinite(a.seconds) or not 1<=a.seconds<=30:p.error('时长须为1到30秒')
    raw=a.candidate.read_bytes();candidate=json.loads(raw);side=candidate.get('side')
    if side not in ('l','r') or candidate.get('accepted_offline') is not True or candidate.get('rejection_reasons')!=[]:
        p.error('候选未通过离线验证')
    digest=hashlib.sha256(raw).hexdigest()
    a.output_dir.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from std_srvs.srv import Trigger
    from x1_coffee_interfaces.msg import MotorStateArray
    signal.signal(signal.SIGINT,signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('x1_current_hold_verification')
    latest=None;received=0.;stamp=0;stage='preflight';events=[];entry_future=None;entry_sent=False;seed=None;fault=None
    stream=(a.output_dir/'feedback.jsonl').open('x')
    def receive(msg):
        nonlocal latest,received,stamp,fault
        s=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if s<=stamp:return
        stamp=s;states={v.joint_name:v for v in msg.states};received=time.monotonic()
        healthy=len(msg.states)==15 and set(states)==set(JOINTS)
        if not healthy:
            latest=dict(healthy=False,q=[],dq=[],current=[]);fault='反馈轴集合错误';return
        healthy=all(v.enabled and v.position_valid and v.velocity_valid and v.current_valid and
            v.position_unit=='rad' and not(v.alarm or v.follow_error or v.emergency_input_active)for v in states.values())
        latest=dict(stage=stage,time_ns=s,received_monotonic=received,healthy=healthy,
            q=[states[j].position for j in JOINTS],dq=[states[j].velocity for j in JOINTS],
            current=[states[j].current_permille for j in JOINTS])
        stream.write(json.dumps(latest,allow_nan=True)+'\n');stream.flush()
        if stage in ('entering','holding') and seed is not None:
            try:check_sample(latest,seed)
            except RuntimeError as exc:fault=str(exc)
    node.create_subscription(MotorStateArray,'/x1/motor_states',receive,qos_profile_sensor_data)
    clients={name:node.create_client(Trigger,'/x1/current_gravity_'+name)for name in ('load','enter','status','exit','hold_status')}
    def spin():rclpy.spin_once(node,timeout_sec=.02)
    def query(name):
        nonlocal entry_future,entry_sent
        client=clients[name]
        if not client.wait_for_service(timeout_sec=.3):return False,'服务不可用 '+name
        if name=='enter':entry_sent=True
        f=client.call_async(Trigger.Request())
        if name=='enter':entry_future=f
        end=time.monotonic()+.5
        while not f.done() and time.monotonic()<end:spin()
        if not f.done():
            if name!='enter':client.remove_pending_request(f)
            return False,'服务响应超时 '+name
        result=f.result();events.append(dict(service=name,success=result.success,message=result.message,time=time.time()))
        return result.success,result.message
    def health(seed,speed=.05):
        if fault:raise RuntimeError(fault)
        if latest is None or time.monotonic()-received>.5:raise RuntimeError('电机反馈过期')
        check_sample(latest,seed,speed=speed)
    report=dict(candidate_sha256=digest,side=side,requested_seconds=a.seconds,
        executed=False,passed=False,exit_hold_confirmed=False,model_or_gains_approved=False)
    error=None
    try:
        end=time.monotonic()+5
        while latest is None and time.monotonic()<end:spin()
        if latest is None:raise RuntimeError('未收到电机反馈')
        seed=np.asarray(latest['q']);health(seed,speed=.02)
        report['initial_q']=seed.tolist()
        ok,message=query('hold_status')
        if not ok:raise RuntimeError(message)
        print('全轴位置保持反馈有效。',flush=True)
        if a.execute:
            ok,message=query('load')
            if not ok or not matching_candidate(message,digest,side):
                raise RuntimeError('加载模型摘要/所选手臂不匹配：'+message)
            # 进入前再次检查采样点和运动状态；后端独立检查局部范围及电流预算。
            health(seed,speed=.02);stage='entering'
            ok,message=query('enter');report['executed']=entry_sent
            if not ok:raise RuntimeError(message)
            end=time.monotonic()+2
            while time.monotonic()<end:
                health(seed);ok,message=query('status')
                if ok:break
            else:raise RuntimeError('未确认实际模式：'+message)
            stage='holding';started=time.monotonic();next_status=started;next_notice=started
            while time.monotonic()-started<a.seconds:
                spin();health(seed)
                if time.monotonic()>=next_status:
                    ok,message=query('status');health(seed)
                    if not ok:raise RuntimeError(message)
                    next_status=time.monotonic()+.25
                if time.monotonic()>=next_notice:
                    print(f'保持验证 {time.monotonic()-started:.1f}/{a.seconds:.1f}s；最大漂移 {max(abs(np.asarray(latest["q"])-seed)):.5f}rad',flush=True)
                    next_notice=time.monotonic()+1
            report['passed']=True
        else:report['preflight_passed']=True
    except BaseException as exc:
        error=exc;report['failure']=str(exc) or type(exc).__name__
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            if entry_sent:
                stage='exiting';query('exit')
                # 进入响应迟到时需等其完成，再请求退出；旧保持反馈不能证明不会迟到进入。
                end=time.monotonic()+3
                while entry_future is not None and not entry_future.done() and time.monotonic()<end:spin()
                resolved=entry_future is not None and entry_future.done()
                if resolved:query('exit')
                quiet_since=None;quiet_stamp=None;end=time.monotonic()+3
                while resolved and time.monotonic()<end:
                    spin();ok,_=query('hold_status')
                    quiet=ok and latest is not None and time.monotonic()-received<.5 and latest['healthy'] and all(np.isfinite(latest['dq'])) and max(abs(np.asarray(latest['dq'])))<.02
                    # 必须由持续到来的新测量证明停稳，不能重复使用同一条反馈计时。
                    if not quiet:
                        quiet_since=None;quiet_stamp=None
                    elif quiet_stamp != stamp:
                        quiet_since=received if quiet_since is None else quiet_since
                        quiet_stamp=stamp
                    if quiet_since is not None and received-quiet_since>=.3:
                        report['exit_hold_confirmed']=True;break
                report['passed']=report['passed'] and report['exit_hold_confirmed']
        except BaseException as exc:
            report['exit_error']=str(exc);report['passed']=False
        stream.close();report['events']=events
        if entry_sent and seed is not None:
            try:
                trace=[json.loads(line) for line in (a.output_dir/'feedback.jsonl').read_text().splitlines()]
                report['metrics']=summarize_trace(trace,seed)
            except (ValueError,RuntimeError) as exc:
                report['measurement_failure']=str(exc);report['passed']=False
        (a.output_dir/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(report,ensure_ascii=False),flush=True)
    if error or (entry_sent and not report['passed']):raise SystemExit(1)


if __name__=='__main__':main()
