#!/usr/bin/env python3
"""单关节摩擦与电流预算诊断（不做 PD 阶跃整定，也不生成补偿参数）。

三个实验：

  --mode budget     读当前姿态的保持电流 hold_current（0x6077 反馈），
                    给出正反向命令余量。hold_current 不是重力电流，
                    静摩擦可能承担了一部分负载。

  --mode breakaway  起动辨识：MIT 手感模式下 kp=0、ff=1，只叠加缓慢上升的
                    tau_ff 偏置，检测到持续位移立刻撤力。正反方向各测，
                    可按多个速率、多次重复。报告里区分：
                      first_motion_time      首次检出位移的时刻
                      confirmed_motion_time  满足持续判据的时刻
                      ramp_at_first_motion   首次位移时的斜坡值
                      ramp_at_confirm        确认时的斜坡值
                    以及 total = hold_current + ramp（真正下发的总命令）。

  --mode motion     运动摩擦辨识：CSP 小行程匀速往返，逐段保存 (t, q, dq, u)，
                    不再把多段平均成一张表。只接受速度稳定、无饱和、
                    未处于换向或制动的窗口；正反向配对要求姿态接近。

报告分三层，明确哪些能确认、哪些要靠假设、哪些现在不可确认：

  [测量量]   hold_current、正反向起动总命令、首次/确认时刻、起动瞬间余量
  [派生量]   起动区间中点与半宽——只在正反方向都有效时给出，
             中点不等于重力电流，半宽不等于双向统一摩擦
  [不可确认] 重力电流、各方向静摩擦真值、惯量与质量参数
             重力电流只能给出一致性区间 [u_-, u_+]
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from x1_ros_env import ensure_ros_env

JOINTS_ALL = ["Joint_Body"] + [f"Joint_{s}{i}" for s in "lr" for i in range(1, 8)]
RIGHT_JOINTS = [f"Joint_r{i}" for i in range(1, 8)]
RATED_TORQUE = [52.0, 52.0, 21.0, 21.0, 10.0, 5.4, 5.4]
LIMIT_BUDGET = [250, 150, 200, 400, 120, 250, 120]
MOTION_SPEEDS = [0.02, 0.05, 0.10]
MOTION_SWING = 0.10
FIRST_MOTION_RAD = 0.02
CONFIRM_VELOCITY = 0.01

RESULTS: dict = {"budget": [], "breakaway": [], "motion": []}


class Bench:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import SetParameters
        from x1_coffee_interfaces.msg import MotorStateArray

        self.rclpy = rclpy
        self.measured: Dict[str, float] = {}
        self.motors: Dict[str, object] = {}
        self.pvt_result: Optional[str] = None
        self.pvt_seq = 0
        self.stamp = 0.0
        self.node = Node("mit_friction_diagnose")
        self.node.create_subscription(JointState, "/joint_states", self._on_joint, qos_profile_sensor_data)
        self.node.create_subscription(MotorStateArray, "/x1/motor_states", self._on_motor,
                                      qos_profile_sensor_data)
        self.node.create_subscription(String, "/x1/pvt_result", self._on_pvt_result, 10)
        self.command_pub = self.node.create_publisher(Float64MultiArray, "/x1/mit_command", 10)
        self.pvt_pub = self.node.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
        self.params_client = self.node.create_client(
            SetParameters, "/x1_coffee_system_hardware/set_parameters")
        self.enter_client = self.node.create_client(Trigger, "/x1/current_gravity_handfeel")
        self.exit_client = self.node.create_client(Trigger, "/x1/current_gravity_exit")
        # 进入/退出都要等实际模式确认，不能一进就加力、一退就换下一次。
        self.status_client = self.node.create_client(Trigger, "/x1/current_gravity_status")
        self.hold_client = self.node.create_client(Trigger, "/x1/current_gravity_hold_status")

    def _on_joint(self, message) -> None:
        for name, position in zip(message.name, message.position):
            self.measured[name] = float(position)
        self.stamp = time.monotonic()

    def _on_motor(self, message) -> None:
        for state in message.states:
            self.motors[state.joint_name] = state

    def _on_pvt_result(self, message) -> None:
        self.pvt_result = message.data

    def spin(self, duration: float) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_ready(self, timeout: float = 10.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if (all(name in self.measured for name in JOINTS_ALL)
                    and all(name in self.motors for name in RIGHT_JOINTS)
                    and time.monotonic() - self.stamp < 0.5):
                return True
        return False

    def trigger(self, client, label: str):
        if not client.wait_for_service(timeout_sec=3.0):
            return False, f"没有服务 {label}"
        future = client.call_async(client.srv_type.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result() if future.done() else None
        if result is None:
            return False, f"{label} 无响应"
        return bool(result.success), result.message

    def push_params(self, kp: Sequence[float], kd: Sequence[float], ff: float,
                    mask: int = 0x7F00) -> bool:
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        if not self.params_client.wait_for_service(timeout_sec=3.0):
            return False
        request = self.params_client.srv_type.Request()
        request.parameters = [
            Parameter(name="mit_mask", value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=int(mask))),
            Parameter(name="mit_kp", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=list(kp))),
            Parameter(name="mit_kd", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=list(kd))),
            Parameter(name="mit_ff_scale", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=ff)),
        ]
        future = self.params_client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        result = future.result() if future.done() else None
        return result is not None and all(item.successful for item in result.results)

    def wait_ok(self, client, label: str, timeout: float = 2.5) -> tuple:
        """轮询服务直到成功或超时，用来确认模式真的切好了。"""
        end = time.monotonic() + timeout
        detail = ""
        while time.monotonic() < end:
            ok, detail = self.trigger(client, label)
            if ok:
                return True, detail
            self.spin(0.2)
        return False, detail

    def pvt_return(self, joint: int, target: float, timeout: float = 6.0) -> float:
        """退出 MIT 后回到起点：位置模式能把静摩擦彻底走完。"""
        name = RIGHT_JOINTS[joint]
        pose = [self.measured[item] for item in JOINTS_ALL]
        index = JOINTS_ALL.index(name)
        delta = target - pose[index]
        if abs(delta) < 1e-4:
            return 0.0
        pose[index] = target
        if not self.move(pose, max(0.8, abs(delta) / 0.1)):
            return abs(self.measured[name] - target)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if abs(self.measured[name] - target) < 0.002:
                break
        return abs(self.measured[name] - target)

    def return_to(self, joint: int, target: float, kp: float, timeout: float = 3.0) -> float:
        """用有界弹簧把关节送回起点，避免每轮实验后手臂逐步漂移。"""
        name = RIGHT_JOINTS[joint]
        self.publish(joint, target, kp, 80.0, 0.0)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if abs(self.measured[name] - target) < 0.005:
                break
        return abs(self.measured[name] - target)

    def hold_current(self, duration: float = 1.0) -> Dict[str, float]:
        samples: Dict[str, List[float]] = {name: [] for name in RIGHT_JOINTS}
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            for name in RIGHT_JOINTS:
                state = self.motors.get(name)
                if state is not None and state.torque_valid:
                    samples[name].append(float(state.torque_permille))
        return {name: (sum(values) / len(values) if values else float("nan"))
                for name, values in samples.items()}

    def publish(self, joint: int, q_des: float, kp: float, kd: float, tau_ff: float) -> None:
        from std_msgs.msg import Float64MultiArray
        message = Float64MultiArray()
        message.data = [float(joint), float(q_des), 0.0, float(kp), float(kd), float(tau_ff)]
        self.command_pub.publish(message)

    def breakaway(self, joint: int, rate: float, kp: float, kd: float,
                  limit_ratio: float, settle: float, max_seconds: float) -> dict:
        """单向斜坡。返回首次位移/确认两套时刻，以及是否在时间/预算内起动。"""
        name = RIGHT_JOINTS[joint]
        budget = LIMIT_BUDGET[joint]
        ceiling = budget * limit_ratio
        q_start = self.measured[name]
        ramp = 0.0
        # 0.05‰ 对应 1‰/s 的下限；之前写 0.5 会把 rate=8 悄悄变成 10‰/s。
        step = max(0.05, abs(rate) * 0.05)
        started_at = time.monotonic()
        first = None            # (time, ramp, displacement)
        confirmed = None        # (time, ramp)
        reason = "ceiling"
        velocity_history: List[float] = []
        previous = q_start
        while abs(ramp) < ceiling:
            ramp += step if rate > 0 else -step
            self.publish(joint, q_start, kp, kd, ramp)
            self.spin(0.05)
            now = time.monotonic()
            q = self.measured[name]
            velocity_history.append((q - previous) / 0.05)
            velocity_history = velocity_history[-3:]
            previous = q
            if first is None and abs(q - q_start) > FIRST_MOTION_RAD:
                first = (now - started_at, ramp, q - q_start)
            if first is not None and len(velocity_history) == 3 and \
                    all(abs(value) > CONFIRM_VELOCITY for value in velocity_history):
                confirmed = (now - started_at, ramp)
                reason = "started"
                break
            if now - started_at > max_seconds:
                reason = "timeout"
                break
        self.publish(joint, q_start, kp, kd, 0.0)
        self.spin(settle)
        displacement = self.measured[name] - q_start
        return {
            "direction": "positive" if rate > 0 else "negative",
            "rate": rate, "budget": budget, "ceiling": ceiling,
            "reason": reason,
            "ramp_at_first_motion": first[1] if first else None,
            "first_motion_s": first[0] if first else None,
            "ramp_at_confirm": confirmed[1] if confirmed else None,
            "confirm_s": confirmed[0] if confirmed else None,
            "confirm_delay_s": (confirmed[0] - first[0]) if (first and confirmed) else None,
            "displacement": displacement,
        }

    def move(self, target: List[float], duration: float) -> bool:
        from std_msgs.msg import Float64MultiArray
        self.pvt_seq += 1
        self.pvt_result = None
        message = Float64MultiArray()
        message.data = [float(self.pvt_seq), float(duration), 1.0, *target]
        self.pvt_pub.publish(message)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.pvt_result:
                parts = self.pvt_result.split(" ", 2)
                try:
                    same = abs(float(parts[0]) - float(self.pvt_seq)) < 0.1
                except (ValueError, IndexError):
                    same = False
                if same and len(parts) > 1 and parts[1] == "ok":
                    return True
                if same:
                    print(f"位置目标失败: {self.pvt_result}")
                    return False
        # 每次会话的第一条 PVT 偶尔没有回应，重发一次再判失败。
        print("位置目标没有回应，重发一次")
        self.pvt_result = None
        self.pvt_pub.publish(message)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.pvt_result:
                parts = self.pvt_result.split(" ", 2)
                try:
                    same = abs(float(parts[0]) - float(self.pvt_seq)) < 0.1
                except (ValueError, IndexError):
                    same = False
                if same and len(parts) > 1 and parts[1] == "ok":
                    return True
                if same:
                    print(f"位置目标失败: {self.pvt_result}")
                    return False
        return False

    def sweep(self, joint: int, target: float, duration: float,
              budget: int) -> List[List[float]]:
        """边运动边采样：t, q, dq, u。不做平均，交给上层判定稳态。"""
        name = RIGHT_JOINTS[joint]
        samples: List[List[float]] = []
        previous = self.measured[name]
        last = time.monotonic()
        end = last + duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
            state = self.motors.get(name)
            now = time.monotonic()
            if state is None or not state.torque_valid or now - last < 0.01:
                continue
            q = self.measured[name]
            velocity = (q - previous) / (now - last)
            previous = q
            last = now
            samples.append([now, q, velocity, float(state.torque_permille)])
        return samples


def check_enabled(bench: Bench) -> bool:
    disabled = [name for name in RIGHT_JOINTS
                if (state := bench.motors.get(name)) is None or not state.enabled]
    if disabled:
        print(f"✗ 这些轴未使能：{', '.join(disabled)}")
        print("  此时 0x6077 恒为 0，任何摩擦/预算结论都无效。")
        return False
    return True


def mode_budget(bench: Bench) -> int:
    if not check_enabled(bench):
        return 2
    hold = bench.hold_current()
    print("\n[测量量] 当前姿态保持电流与命令余量")
    print("关节  hold_current(‰)  预算(‰)  h+(‰)  h-(‰)  进入MIT")
    rows = []
    for joint, name in enumerate(RIGHT_JOINTS):
        current = hold[name]
        budget = LIMIT_BUDGET[joint]
        entry = "可以" if abs(current) <= budget * 0.8 else "会被拒绝"
        print(f"  r{joint + 1:<2} {current:>13.1f} {budget:>8} {budget - current:>7.1f} "
              f"{budget + current:>6.1f}  {entry}")
        rows.append({"joint": joint + 1, "hold_current": current, "budget": budget,
                     "headroom_positive": budget - current,
                     "headroom_negative": budget + current})
    print("\n注意：hold_current 不等于重力电流——静摩擦可能分担了一部分负载，"
          "本模式不能把它当作重力补偿值。")
    RESULTS["budget"] = rows
    return 0 if rows else 2


def mode_breakaway(bench: Bench, args) -> int:
    if not check_enabled(bench):
        return 2
    rates = [float(value) for value in str(args.rates).split(",") if value.strip()]
    for joint in args.joints:
        for rate in rates:
            needed = LIMIT_BUDGET[joint] * args.limit_ratio / max(rate, 1e-6) + 3.0
            if needed > args.max_seconds:
                print(f"⚠ r{joint + 1} @ {rate}‰/s 要走到预算的 {args.limit_ratio:.0%} 需要 "
                      f"{needed:.0f}s，超过当前 --max-seconds {args.max_seconds:.0f}s；"
                      f"到点会报 timeout，不会给出阈值。")
    print(f"\n起动辨识：速率 {rates} ‰/s，每方向重复 {args.repeat} 次，"
          f"单次最长 {args.max_seconds:.0f}s，记录保持端电流")
    print("[测量量] 关节 速率 方向 保持电流 首动斜坡 确认斜坡 首动/确认时刻 总命令 起动余量 状态")
    results = []
    for joint in args.joints:
        hold = bench.hold_current(0.8).get(RIGHT_JOINTS[joint], float("nan"))
        first_pose = bench.measured[RIGHT_JOINTS[joint]]
        for attempt in range(args.repeat):
            for rate in rates:
                for direction in (1.0, -1.0):
                    if not check_enabled(bench):
                        print("  轴已关断，停止本次辨识；请重新走一遍使能序列")
                        RESULTS["breakaway"] = results
                        return 2
                    # 保持电流要在 CSP 下测：进入 MIT 后读到的是带偏置的 CST 值。
                    run_hold = bench.hold_current(0.4).get(RIGHT_JOINTS[joint], float("nan"))
                    # 只让被测轴进 CST，其余六轴留在 CSP 位置保持，不再整臂浮动。
                    joint_mask = 0x0100 << joint
                    if not bench.push_params([args.kp] * 7, [args.kd] * 7, args.ff, joint_mask):
                        print("参数下发失败")
                        return 2
                    ok, detail = bench.trigger(bench.enter_client, "/x1/current_gravity_handfeel")
                    if not ok:
                        print(f"  r{joint + 1} 进入 MIT 失败：{detail}")
                        break
                    confirmed, detail = bench.wait_ok(bench.status_client, "CST 模式确认")
                    if not confirmed:
                        print(f"  r{joint + 1} 模式未确认（{detail}），本轮跳过，不加力")
                        bench.trigger(bench.exit_client, "/x1/current_gravity_exit")
                        bench.wait_ok(bench.hold_client, "CSP 确认")
                        continue
                    bench.spin(args.dwell)
                    start_pose = bench.measured[RIGHT_JOINTS[joint]]
                    try:
                        result = bench.breakaway(joint, rate * direction, args.kp, args.kd,
                                                 args.limit_ratio, args.settle, args.max_seconds)
                    finally:
                        bench.trigger(bench.exit_client, "/x1/current_gravity_exit")
                        bench.wait_ok(bench.hold_client, "CSP 确认")
                    # 力矩弹簧会被静摩擦卡在半路（残差≈摩擦/kp），回位必须走位置模式。
                    drift = bench.pvt_return(joint, start_pose)
                    result["return_drift"] = drift
                    result.update({"joint": joint + 1, "attempt": attempt + 1,
                                   "hold_current": run_hold, "start_pose": start_pose,
                                   "pose_span": start_pose - first_pose})
                    if result["ramp_at_confirm"] is not None:
                        # 用本次尝试前实测的保持电流，而不是整个关节的第一次读数。
                        result["total"] = run_hold + result["ramp_at_confirm"]
                        result["headroom_at_breakaway"] = result["budget"] - result["total"]
                    else:
                        result["total"] = None
                        result["headroom_at_breakaway"] = None
                    results.append(result)
                    if result["total"] is None:
                        print(f"  r{joint + 1} {rate:>4.0f} {result['direction']:<8} "
                              f"{hold:>8.1f}  ——      ——     ——                ——     ——    "
                              f"{result['reason']}")
                    else:
                        print(f"  r{joint + 1} {rate:>4.0f} {result['direction']:<8} "
                              f"{run_hold:>8.1f} {result['ramp_at_first_motion']:>8.0f} "
                              f"{result['ramp_at_confirm']:>8.0f} "
                              f"{result['first_motion_s']:>6.2f}/{result['confirm_s']:>5.2f}s  "
                              f"{result['total']:>6.0f} {result['headroom_at_breakaway']:>7.0f}  "
                              f"{result['reason']}")
    pose_report(results)
    summarize_breakaway(results)
    RESULTS["breakaway"] = results
    return 0 if results else 2


def pose_report(results: List[dict]) -> None:
    """每次尝试的起点姿态和送回残差：判断离散度里有多少来自姿态漂移。"""
    for joint in sorted({item["joint"] for item in results}):
        items = [item for item in results if item["joint"] == joint]
        poses = [item["start_pose"] for item in items if "start_pose" in item]
        drifts = [item["return_drift"] for item in items if item.get("return_drift") is not None]
        if not poses:
            continue
        span = max(poses) - min(poses)
        text = (f"  r{joint} 起点姿态 {min(poses):+.3f}~{max(poses):+.3f} rad"
                f"（跨度 {span * 1000:.1f} mrad）")
        if drifts:
            text += f"，送回残差最大 {max(drifts) * 1000:.1f} mrad"
        print(text)


def summarize_breakaway(results: List[dict]) -> None:
    """按 (关节, 速率) 汇总，明确区分测量量、派生量和不可确认量。"""
    print("\n[派生量] 同一速率下的起动区间（只在该速率正反方向都有效时给出）")
    print("关节 速率  中点(‰)  半宽(‰)  解释")
    for joint in sorted({item["joint"] for item in results}):
        for rate in sorted({abs(item["rate"]) for item in results if item["joint"] == joint}):
            plus = [item for item in results if item["joint"] == joint
                    and abs(item["rate"] - rate) < 1e-9 and item.get("total") is not None]
            minus = [item for item in results if item["joint"] == joint
                     and abs(item["rate"] + rate) < 1e-9 and item.get("total") is not None]
            if not plus or not minus:
                continue
            up = statistics.median(item["total"] for item in plus)
            down = statistics.median(item["total"] for item in minus)
            midpoint = (up + down) / 2.0
            halfwidth = (up - down) / 2.0
            # 用绝对 ‰ 的离散度：中位数可能接近 0（r1 就是），相对值会失真。
            spreads = [statistics.pstdev([item["total"] for item in group])
                       for group in (plus, minus)]
            budget = plus[0]["budget"]
            worst = max(spreads)
            verdict = ("可用" if worst < budget * 0.08
                       else "勉强" if worst < budget * 0.15 else "不可用")
            print(f"  r{joint:<2} {rate:>4.0f} {midpoint:>9.1f} {halfwidth:>8.1f}  "
                  f"离散 σ: 正{spreads[0]:>5.1f}‰/反{spreads[1]:>5.1f}‰ → {verdict}；"
                  f"中点≠重力电流，半宽≠双向统一摩擦")
            print(f"        重力电流只落在这个区间内：{down:+.0f} ‰ ≤ u_g ≤ {up:+.0f} ‰")
    print("\n[不可确认] 本轮实验不能确定：重力电流 u_g（含静摩擦分担）、"
          "正反向静摩擦真值、惯量与质量参数。")
    print("           这些需要独立的重力模型或多姿态辨识，不能由上面的中点/半宽直接给出。")


def classify_window(samples: List[List[float]], speed: float, budget: int) -> dict:
    """匀速运动本来就在移动，所以判速度/力矩稳定，不判位置不变。

    丢掉前后各 20%（起步与制动），只在中段里找与目标速度一致、
    未饱和的连续样本。
    """
    if len(samples) < 40:
        return {"valid": False, "reason": "样本不足"}
    low = int(len(samples) * 0.2)
    high = int(len(samples) * 0.8)
    middle = samples[low:high]
    steady = []
    for _, q, velocity, torque in middle:
        if abs(velocity - speed) > max(0.015, abs(speed) * 0.35):
            continue
        if abs(torque) > budget * 0.95:
            continue
        steady.append((q, velocity, torque))
    if len(steady) < 20:
        return {"valid": False, "reason": "未形成稳态或已饱和", "n_steady": len(steady)}
    velocities = [item[1] for item in steady]
    torques = [item[2] for item in steady]
    poses = [item[0] for item in steady]
    mean_velocity = sum(velocities) / len(velocities)
    std_velocity = statistics.pstdev(velocities)
    if abs(mean_velocity - speed) > abs(speed) * 0.25:
        return {"valid": False, "reason": f"平均速度偏差大（{mean_velocity:+.3f}）",
                "n_steady": len(steady)}
    if std_velocity > abs(speed) * 0.3:
        return {"valid": False, "reason": f"速度不稳（σ={std_velocity:.3f}）",
                "n_steady": len(steady)}
    return {"valid": True, "reason": "ok", "n_steady": len(steady),
            "ratio": len(steady) / len(samples),
            "mean_velocity": mean_velocity, "std_velocity": std_velocity,
            "mean_q": sum(poses) / len(poses),
            "mean_torque": sum(torques) / len(torques),
            "std_torque": statistics.pstdev(torques),
            "steady": [(item[0], item[2]) for item in steady]}


def mode_motion(bench: Bench, args) -> int:
    if not check_enabled(bench):
        return 2
    out = Path(args.out)
    records = []
    for joint in args.joints:
        name = RIGHT_JOINTS[joint]
        for speed in MOTION_SPEEDS:
            for direction in (1.0, -1.0):
                start = [bench.measured[joint_name] for joint_name in JOINTS_ALL]
                index = JOINTS_ALL.index(name)
                target = list(start)
                target[index] = start[index] + MOTION_SWING * direction
                duration = max(0.8, MOTION_SWING / speed)
                if not bench.move(target, duration):
                    continue
                samples = bench.sweep(joint, target[index], duration, LIMIT_BUDGET[joint])
                verdict = classify_window(samples, direction * speed, LIMIT_BUDGET[joint])
                path = out / f"j{joint + 1}_{direction:+.0f}_{speed:.2f}.csv"
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("w", encoding="utf-8") as handle:
                    handle.write("t,q,dq,u\n")
                    for row in samples:
                        handle.write(f"{row[0]:.6f},{row[1]:.6f},{row[2]:.6f},{row[3]:.3f}\n")
                record = {"joint": joint + 1, "speed": direction * speed,
                          "csv": str(path), **verdict}
                records.append(record)
                if verdict["valid"]:
                    print(f"  r{joint + 1} {direction * speed:+.2f} rad/s  "
                          f"稳态 {verdict['n_steady']} 点/{verdict['ratio'] * 100:.0f}%  "
                          f"实测速度 {verdict['mean_velocity']:+.3f}±{verdict['std_velocity']:.3f}  "
                          f"姿态 {verdict['mean_q']:+.3f} 力矩 {verdict['mean_torque']:+.1f}"
                          f"±{verdict['std_torque']:.1f}‰")
                else:
                    print(f"  r{joint + 1} {direction * speed:+.2f} rad/s  丢弃：{verdict['reason']}")
                bench.move(start, duration)
    print("\n[测量量] 每段原始曲线已写入 CSV；正反向配对只在平均姿态相差 <0.02 rad 时给出。")
    print("关节  |v|      摩擦半差(‰)  残差(‰)  备注")
    for joint in sorted({item["joint"] for item in records if item["valid"]}):
        for speed in sorted({abs(item["speed"]) for item in records
                             if item["joint"] == joint and item["valid"]}):
            plus = [item for item in records if item["joint"] == joint and item["valid"]
                    and abs(item["speed"] - speed) < 1e-9]
            minus = [item for item in records if item["joint"] == joint and item["valid"]
                     and abs(item["speed"] + speed) < 1e-9]
            if not plus or not minus:
                continue
            if abs(plus[0]["mean_q"] - minus[0]["mean_q"]) > 0.02:
                print(f"  r{joint:<2} {speed:.2f}      ——           ——       正反向姿态差过大，不配对")
                continue
            friction = (plus[0]["mean_torque"] - minus[0]["mean_torque"]) / 2
            residual = (plus[0]["mean_torque"] + minus[0]["mean_torque"]) / 2
            print(f"  r{joint:<2} {speed:.2f} {friction:>12.1f} {residual:>9.1f}   "
                  f"残差含重力与方向不对称，不能直接当重力")
    print("\n[不可确认] 本模式不给出重力电流，也不把六段平均成一条 u_f(v)；"
          "负载依赖与方向差异需要独立实验。")
    RESULTS["motion"] = records
    return 0 if records else 2


def fit_at_position(steady: List[tuple], target: float):
    """把一小段稳态样本按位置线性拟合，再取目标位置上的值。"""
    if len(steady) < 10:
        return None
    import numpy as np
    qs = np.array([item[0] for item in steady])
    us = np.array([item[1] for item in steady])
    if qs.max() - qs.min() < 1e-6:
        return float(us.mean())
    basis = np.column_stack([qs - qs.mean(), np.ones_like(qs)])
    coefficients, *_ = np.linalg.lstsq(basis, us, rcond=None)
    return float(coefficients[0] * (target - qs.mean()) + coefficients[1])


def mode_poses(bench: Bench, args) -> int:
    """多姿态运动法：自动把被测关节依次摆到几个角度，每个角度做正反向匀速配对。

    每个角度上正向和反向各走一小段（-swing/+swing），各自按位置线性拟合后
    在角度处取正反向电流：平均是重力项，差是摩擦。这样不要求两次停在同一点。
    """
    if not check_enabled(bench):
        return 2
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = []
    speeds = [0.05, 0.10]
    for joint in args.joints:
        name = RIGHT_JOINTS[joint]
        low_limit, high_limit = _joint_limits(name)
        angles = [float(item) for item in str(args.angles).split(",") if item.strip()] \
            if args.angles else None
        center = bench.measured[name]
        if angles is None:
            angles = [center - 0.15, center, center + 0.15]
        rows = []
        index = JOINTS_ALL.index(name)
        for angle in angles:
            if not (low_limit + 0.02 < angle < high_limit - 0.02):
                print(f"  r{joint + 1} 角度 {angle:+.3f} 超出限位，跳过")
                continue
            pose = [bench.measured[item] for item in JOINTS_ALL]
            pose[index] = angle
            if not bench.move(pose, max(0.8, abs(pose[index] - bench.measured[name]) / 0.1)):
                print(f"  r{joint + 1} 走到 {angle:+.3f} 失败")
                continue
            bench.spin(args.dwell)
            start_pose = bench.measured[name]
            per_speed = {}
            for speed in speeds:
                for direction in (1.0, -1.0):
                    target = start_pose + direction * args.swing
                    goal = list(pose)
                    goal[index] = target
                    if not bench.move(goal, max(0.8, args.swing / speed)):
                        continue
                    samples = bench.sweep(joint, target, max(0.8, args.swing / speed),
                                          LIMIT_BUDGET[joint])
                    verdict = classify_window(samples, direction * speed, LIMIT_BUDGET[joint])
                    if verdict["valid"]:
                        value = fit_at_position(verdict["steady"], start_pose)
                        if value is not None:
                            per_speed.setdefault(speed, {})[direction] = value
                    # 回中，避免行程累积
                    back = list(pose)
                    back[index] = start_pose
                    bench.move(back, max(0.8, args.swing / speed))
                    bench.spin(0.3)
            # 每个速度都配出 (u+ + u-)/2 与 (u+ - u-)/2，再按速度平均
            gravity_values, friction_values = [], []
            for speed, pair in per_speed.items():
                if 1.0 in pair and -1.0 in pair:
                    gravity_values.append((pair[1.0] + pair[-1.0]) / 2)
                    friction_values.append((pair[1.0] - pair[-1.0]) / 2)
            if not gravity_values:
                print(f"  r{joint + 1} {angle:+.3f} rad：没有可配对的稳态段")
                continue
            row = {"joint": joint + 1, "angle": start_pose,
                   "gravity": sum(gravity_values) / len(gravity_values),
                   "friction": sum(friction_values) / len(friction_values),
                   "speed_count": len(gravity_values)}
            rows.append(row)
            print(f"  r{joint + 1} {row['angle']:+.3f} rad  重力 {row['gravity']:>7.1f}‰  "
                  f"摩擦 {row['friction']:>6.1f}‰  （{row['speed_count']} 个速度配对）")
        if len(rows) < 2:
            print(f"  r{joint + 1} 有效姿态不足两个，无法看趋势")
            continue
        import numpy as np
        qs = np.array([item["angle"] for item in rows])
        gravity = np.array([item["gravity"] for item in rows])
        basis = np.column_stack([qs - qs.mean(), np.ones_like(qs)])
        coefficients, *_ = np.linalg.lstsq(basis, gravity, rcond=None)
        rms = float(np.sqrt(np.mean((basis @ coefficients - gravity) ** 2)))
        span = float(gravity.max() - gravity.min())
        print(f"    → u_g(q) = {coefficients[0]:+.0f}‰/rad ·(q−{qs.mean():+.3f}) "
              f"{coefficients[1]:+.1f}，RMS {rms:.1f}‰，窗口内变化 {span:.1f}‰")
        results.append({"joint": joint + 1, "rows": rows, "slope": float(coefficients[0]),
                        "intercept": float(coefficients[1]), "rms": rms})
        RESULTS.setdefault("poses", []).append(results[-1])
    print("\n[不可确认] 这些点是「其余关节固定」时的 u_g(q)；换手臂姿态要重测，"
          "也不能据此反推质量或惯量。")
    return 0 if results else 2


def mode_sweep(bench: Bench, args) -> int:
    """逐点停住采样的重力曲线：位置模式走到每个点、等停稳、读稳态力矩。

    PVT 执行器不按我们给的时长匀速走，靠等速采样拿不到足够行程；
    改成「走一步、停住、测稳态」，正反两个方向各走一遍，
    同一位置的正反向平均是重力项、差是摩擦。
    """
    if not check_enabled(bench):
        return 2
    import numpy as np
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = []
    for joint in args.joints:
        name = RIGHT_JOINTS[joint]
        center = bench.measured[name]
        low = center - args.swing
        high = center + args.swing
        steps = max(3, args.steps)
        positions = [low + (high - low) * index / (steps - 1) for index in range(steps)]
        index = JOINTS_ALL.index(name)
        table = []
        for direction in (1, -1):
            order = positions if direction > 0 else list(reversed(positions))
            for q in order:
                pose = [bench.measured[item] for item in JOINTS_ALL]
                pose[index] = q
                if not bench.move(pose, max(0.8, abs(pose[index] - bench.measured[name]) / 0.1)):
                    print(f"  r{joint + 1} 走到 {q:+.3f} 失败")
                    continue
                bench.spin(args.dwell)
                values = []
                end = time.monotonic() + 0.4
                while time.monotonic() < end:
                    bench.rclpy.spin_once(bench.node, timeout_sec=0.02)
                    state = bench.motors.get(name)
                    if state is not None and state.torque_valid:
                        values.append(float(state.torque_permille))
                if not values:
                    continue
                table.append({"direction": direction, "q": bench.measured[name],
                              "torque": sum(values) / len(values),
                              "spread": max(values) - min(values), "n": len(values)})
        if not table:
            continue
        rows = []
        for item in table:
            if item["direction"] > 0:
                continue
            partner = min((other for other in table if other["direction"] > 0),
                          key=lambda other: abs(other["q"] - item["q"]), default=None)
            if partner is None or abs(partner["q"] - item["q"]) > 0.02:
                continue
            # 停住时正反向的差来自位置环落点误差（静摩擦让关节停得偏一点），
            # 不是摩擦；这里只作为方向差记录，不叫 friction。
            rows.append({"q": (item["q"] + partner["q"]) / 2,
                         "gravity": (item["torque"] + partner["torque"]) / 2,
                         "spread": (partner["torque"] - item["torque"]) / 2,
                         "up": partner["torque"], "down": item["torque"]})
        if len(rows) < 3:
            print(f"  r{joint + 1} 正反向位置没配上，测到的点太少")
            continue
        rows.sort(key=lambda item: item["q"])
        qs = np.array([item["q"] for item in rows])
        gravity = np.array([item["gravity"] for item in rows])
        spread = np.array([item["spread"] for item in rows])
        width = float(qs.max() - qs.min())
        basis = np.column_stack([qs - qs.mean(), np.ones_like(qs)])
        coefficients, *_ = np.linalg.lstsq(basis, gravity, rcond=None)
        rms = float(np.sqrt(np.mean((basis @ coefficients - gravity) ** 2)))
        span = float(gravity.max() - gravity.min())
        print(f"\n  r{joint + 1} 姿态 {low:+.3f}~{high:+.3f} rad，{len(rows)} 个配对点")
        print("    位置      重力(‰)  方向差(‰)   正向    反向")
        for item in rows:
            print(f"    {item['q']:+.4f} {item['gravity']:>9.1f} {item['spread']:>9.1f} "
                  f"{item['up']:>8.1f} {item['down']:>8.1f}")
        print(f"    线性拟合 u_g(q) = {coefficients[0]:+.0f}‰/rad ·(q−{qs.mean():+.3f}) "
              f"{coefficients[1]:+.1f}，RMS {rms:.1f}‰；窗口内变化 {span:.1f}‰")
        print(f"    方向差 {float(spread.min()):+.1f}~{float(spread.max()):+.1f}‰"
              f"（位置环落点差异，不是摩擦）")
        if span < 0.1 * abs(float(gravity.mean())):
            print("    → 重力在这个区间近似常数，前馈可以用一个常数")
        results.append({"joint": joint + 1, "rows": rows, "rms": rms,
                        "slope": coefficients[0], "intercept": coefficients[1]})
        RESULTS.setdefault("sweep", []).append(results[-1])
    print("\n[不可确认] 这是其余关节固定时的 u_g(q)，不能外推到其他手臂姿态，"
          "也不是惯量或质量参数。")
    return 0 if results else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("budget", "breakaway", "motion", "sweep", "poses"),
                        default="budget")
    parser.add_argument("--joints", type=lambda text: [int(v) - 1 for v in text.split(",")],
                        default=list(range(7)))
    parser.add_argument("--rates", default="8,20", help="起动辨识的力矩上升速率 ‰/s，可多个")
    parser.add_argument("--repeat", type=int, default=1, help="每个速率/方向重复次数")
    parser.add_argument("--max-seconds", type=float, default=20.0,
                        help="单次斜坡最长秒数；到点即报 timeout，不自动延长")
    parser.add_argument("--limit-ratio", type=float, default=0.9,
                        help="斜坡最多到预算的多少倍")
    parser.add_argument("--kp", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=80.0)
    parser.add_argument("--ff", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=0.5)
    parser.add_argument("--swing", type=float, default=0.15, help="sweep 模式的单边行程 rad")
    parser.add_argument("--steps", type=int, default=5, help="sweep 模式每个方向测几个点")
    parser.add_argument("--angles", default="",
                        help="poses 模式的角度列表（绝对 rad）；留空取当前±0.15")
    parser.add_argument("--dwell", type=float, default=0.8,
                        help="进入 MIT 并确认模式后，再等多久才开始加力")
    parser.add_argument("--return-kp", type=float, default=400.0,
                        help="每轮结束后用于把关节送回起点的有界弹簧（‰）")
    parser.add_argument("--out", default=f"log/mit_friction_{time.strftime('%Y%m%d_%H%M%S')}")
    args = parser.parse_args()

    ensure_ros_env()
    import rclpy
    rclpy.init()
    bench = Bench()
    try:
        if not bench.wait_ready():
            print("收不到 /joint_states 或 /x1/motor_states")
            return 2
        if args.mode == "budget":
            status = mode_budget(bench)
        elif args.mode == "breakaway":
            status = mode_breakaway(bench, args)
        elif args.mode == "sweep":
            status = mode_sweep(bench, args)
        elif args.mode == "poses":
            status = mode_poses(bench, args)
        else:
            status = mode_motion(bench, args)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(
            {"mode": args.mode, "args": vars(args), "results": RESULTS},
            ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n结果目录 {out}")
        return status
    finally:
        bench.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
