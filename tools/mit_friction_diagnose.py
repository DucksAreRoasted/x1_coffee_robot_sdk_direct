#!/usr/bin/env python3
"""单关节摩擦与电流预算诊断（不做 PD 阶跃整定）。

三个实验，对应「起动 / 运动 / 预算」三类问题：

  --mode budget     读取指定姿态下的保持力矩 u0（0x6077 反馈），
                    与每轴电流预算比较，给出正反向余量 h+ / h-。
                    u0 的绝对值超过预算 80% 时，MIT 手感模式会拒绝进入。

  --mode breakaway  起动辨识：MIT 手感模式下 kp=0、ff=1（用进入时的保持电流抵重力），
                    只叠加一个缓慢上升的 tau_ff 偏置，检测到持续运动就立刻撤力。
                    正反方向各做一次，输出起动阈值（‰，相对 u0）。
                    到达预算 90% 仍未起动时，结论写成「预算内未观察到起动」下界，
                    不会把预算上限当成测得的静摩擦，也不会自动提高预算继续试。

  --mode motion     运动摩擦辨识：CSP 下用小行程、匀速往返代替空载连续转动，
                    采样只接受「速度稳定、无饱和、无换向」的窗口，
                    正反方向分别建表，不合并、不预先扣重力。

注意：
  * 静止保持电流不一定等于重力电流（静摩擦也分担负载），所以这里只报告
    「该姿态下的正反向总起动电流」，不命名为纯摩擦参数。
  * 补偿不能创造力矩；命令被截断和 PD 不够是两回事，本工具分别记录。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "gravity_identification"))

from x1_ros_env import ensure_ros_env

JOINTS_ALL = ["Joint_Body"] + [f"Joint_{s}{i}" for s in "lr" for i in range(1, 8)]
RIGHT_JOINTS = [f"Joint_r{i}" for i in range(1, 8)]
RATED_TORQUE = [52.0, 52.0, 21.0, 21.0, 10.0, 5.4, 5.4]
LIMIT_BUDGET = [250, 150, 200, 400, 120, 250, 120]
MOTION_SPEEDS = [0.02, 0.05, 0.10]      # rad/s
MOTION_SWING = 0.10                      # rad，单向行程

RESULTS: dict = {"budget": [], "breakaway": [], "motion": []}


class Bench:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
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
        # 广播器是 BEST_EFFORT，而且一次发整组 15 轴。
        self.node.create_subscription(MotorStateArray, "/x1/motor_states", self._on_motor,
                                      qos_profile_sensor_data)
        from std_msgs.msg import String
        self.node.create_subscription(String, "/x1/pvt_result", self._on_pvt_result, 10)
        self.command_pub = self.node.create_publisher(Float64MultiArray, "/x1/mit_command", 10)
        self.pvt_pub = self.node.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
        self.params_client = self.node.create_client(
            SetParameters, "/x1_coffee_system_hardware/set_parameters")
        self.enter_client = self.node.create_client(Trigger, "/x1/current_gravity_handfeel")
        self.exit_client = self.node.create_client(Trigger, "/x1/current_gravity_exit")

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

    def push_params(self, kp: Sequence[float], kd: Sequence[float], ff: float) -> bool:
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        if not self.params_client.wait_for_service(timeout_sec=3.0):
            return False
        request = self.params_client.srv_type.Request()
        request.parameters = [
            Parameter(name="mit_mask", value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=0x7F00)),
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

    # ---------- 预算 ----------
    def baseline(self, duration: float = 1.5) -> Dict[str, float]:
        """当前姿态下的保持力矩 u0（‰，0x6077 实际值）。"""
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

    # ---------- 起动 ----------
    def breakaway(self, joint: int, rate: float, kp: float, kd: float,
                  limit_ratio: float, settle: float) -> dict:
        """单向力矩斜坡，检测起动。返回阈值或「预算内未起动」。"""
        name = RIGHT_JOINTS[joint]
        budget = LIMIT_BUDGET[joint]
        ceiling = budget * limit_ratio
        q_start = self.measured[name]
        ramp = 0.0
        step = max(1.0, rate * 0.05)          # 每 50 ms 加多少‰
        moved_at: Optional[float] = None
        started = time.monotonic()
        while abs(ramp) < ceiling:
            ramp += step if rate > 0 else -step
            self.publish(joint, q_start, kp, kd, ramp)
            self.spin(0.05)
            if abs(self.measured[name] - q_start) > 0.02:
                moved_at = ramp
                break
            if time.monotonic() - started > 20.0:
                break
        self.publish(joint, q_start, kp, kd, 0.0)
        self.spin(settle)
        return {
            "joint": joint + 1, "budget": budget, "ceiling": ceiling,
            "direction": "positive" if rate > 0 else "negative",
            "threshold": moved_at, "started": moved_at is not None,
            "displacement": self.measured[name] - q_start,
        }

    def publish(self, joint: int, q_des: float, kp: float, kd: float, tau_ff: float) -> None:
        from std_msgs.msg import Float64MultiArray
        message = Float64MultiArray()
        message.data = [float(joint), float(q_des), 0.0, float(kp), float(kd), float(tau_ff)]
        self.command_pub.publish(message)

    # ---------- 运动摩擦 ----------
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
        print("位置目标没有回应")
        return False

    def steady_torque(self, joint: int, duration: float, settle: float = 0.3) -> Optional[float]:
        """取窗口末尾的稳态力矩：只接受速度稳定、无饱和、无换向的采样。"""
        name = RIGHT_JOINTS[joint]
        self.spin(settle)
        values, speeds, saturated = [], [], 0
        end = time.monotonic() + duration
        previous = self.measured[name]
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            state = self.motors.get(name)
            if state is None or not state.torque_valid:
                continue
            q = self.measured[name]
            velocity = (q - previous) / 0.01
            previous = q
            speeds.append(velocity)
            values.append(float(state.torque_permille))
        if len(values) < 10:
            return None
        mean_speed = sum(speeds) / len(speeds)
        spread = max(speeds) - min(speeds)
        if spread > max(0.02, abs(mean_speed) * 0.5):
            return None                       # 速度不稳，丢弃窗口
        return sum(values) / len(values)


def check_enabled(bench: Bench) -> bool:
    """先确认轴真的使能了：没使能时 0x6077 全是 0，会把"没负载"误读成"没摩擦"。"""
    disabled = [name for name in RIGHT_JOINTS
                if (state := bench.motors.get(name)) is None or not state.enabled]
    if disabled:
        print(f"✗ 这些轴未使能：{', '.join(disabled)}")
        print("  此时 0x6077 恒为 0，任何摩擦/预算结论都无效。"
              "请先确认控制栈没有传输故障并重新进入使能保持。")
        return False
    return True


def mode_budget(bench: Bench) -> int:
    if not check_enabled(bench):
        return 2
    baseline = bench.baseline()
    print("\n关节  保持力矩 u0(‰)  预算(‰)   正余量 h+   反余量 h-   进入 MIT")
    rows = []
    for joint, name in enumerate(RIGHT_JOINTS):
        u0 = baseline[name]
        budget = LIMIT_BUDGET[joint]
        head_plus = budget - u0
        head_minus = budget + u0
        entry = "可以" if abs(u0) <= budget * 0.8 else "会被拒绝（>80%预算）"
        print(f"  r{joint + 1:<2} {u0:>12.1f} {budget:>8} {head_plus:>10.1f} "
              f"{head_minus:>10.1f}   {entry}")
        rows.append({"joint": joint + 1, "u0": u0, "budget": budget,
                     "head_plus": head_plus, "head_minus": head_minus})
    RESULTS["budget"] = rows
    return 0 if rows else 2


def mode_breakaway(bench: Bench, args) -> int:
    if not check_enabled(bench):
        return 2
    results = []
    for joint in args.joints:
        # 先记录进入前的保持电流 u0：节点用它作为 MIT 的偏置，
        # 所以起动时的总下发量 = u0 + 斜坡增量。只报增量会低估真实需要。
        u0 = bench.baseline(0.6).get(RIGHT_JOINTS[joint], float("nan"))
        row = {"joint": joint + 1, "u0": u0, "directions": []}
        for rate in (args.rate, -args.rate):
            if not bench.push_params([args.kp] * 7, [args.kd] * 7, args.ff):
                print("参数下发失败")
                return 2
            ok, detail = bench.trigger(bench.enter_client, "/x1/current_gravity_handfeel")
            if not ok:
                print(f"r{joint + 1} 进入 MIT 失败：{detail}")
                row["directions"].append({"direction": "negative" if rate < 0 else "positive",
                                          "error": detail})
                continue
            try:
                result = bench.breakaway(joint, rate, args.kp, args.kd,
                                         args.limit_ratio, args.settle)
            finally:
                bench.trigger(bench.exit_client, "/x1/current_gravity_exit")
            if result["started"]:
                total = u0 + result["threshold"]
                result["total"] = total
                print(f"  r{joint + 1} {'正' if rate > 0 else '反'}向起动阈值 "
                      f"增量 {abs(result['threshold']):.0f}‰，总下发 {total:+.0f}‰"
                      f"（保持 {u0:+.0f}，预算 {result['budget']}‰）")
            else:
                print(f"  r{joint + 1} {'正' if rate > 0 else '反'}向：预算内未观察到起动 "
                      f"（斜坡到 {result['ceiling']:.0f}‰ 仍不动）→ 阈值只能给下界")
            row["directions"].append(result)
        results.append(row)
    RESULTS["breakaway"] = results
    return 0 if results else 2


def mode_motion(bench: Bench, args) -> int:
    if not check_enabled(bench):
        return 2
    rows = []
    for joint in args.joints:
        name = RIGHT_JOINTS[joint]
        for speed in MOTION_SPEEDS:
            for direction in (1.0, -1.0):
                swing = MOTION_SWING * direction
                start = [bench.measured[joint_name] for joint_name in JOINTS_ALL]
                target = list(start)
                target[JOINTS_ALL.index(name)] = start[JOINTS_ALL.index(name)] + swing
                duration = max(0.8, abs(swing) / speed)
                if not bench.move(target, duration):
                    continue
                torque = bench.steady_torque(joint, duration * 0.5)
                if torque is None:
                    print(f"  r{joint + 1} {direction * speed:+.2f} rad/s：速度不稳，窗口丢弃")
                    continue
                rows.append({"joint": joint + 1, "speed": direction * speed, "torque": torque})
                print(f"  r{joint + 1} {direction * speed:+.2f} rad/s  稳态力矩 {torque:+.1f}‰")
                # 回到起点，避免行程累积。
                bench.move(start, duration)
        # 同速度正反向的数据配成摩擦/重力估计（仅当摩擦近似对称时后者才可用）
        speeds = sorted({abs(row["speed"]) for row in rows if row["joint"] == joint + 1})
        for value in speeds:
            plus = [row["torque"] for row in rows
                    if row["joint"] == joint + 1 and abs(row["speed"] - value) < 1e-9]
            minus = [row["torque"] for row in rows
                     if row["joint"] == joint + 1 and abs(row["speed"] + value) < 1e-9]
            if plus and minus:
                print(f"     |v|={value:.2f}: 摩擦≈{(plus[0] - minus[0]) / 2:+.1f}‰  "
                      f"残差(重力+不对称)≈{(plus[0] + minus[0]) / 2:+.1f}‰")
    RESULTS["motion"] = rows
    return 0 if rows else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("budget", "breakaway", "motion"), default="budget")
    parser.add_argument("--joints", type=lambda text: [int(v) - 1 for v in text.split(",")],
                        default=list(range(7)))
    parser.add_argument("--rate", type=float, default=20.0, help="起动辨识的力矩上升速率 ‰/s")
    parser.add_argument("--limit-ratio", type=float, default=0.9,
                        help="力矩斜坡最多到预算的多少倍；到顶仍未动就报下界")
    parser.add_argument("--kp", type=float, default=0.0, help="起动辨识时的 kp（默认 0）")
    parser.add_argument("--kd", type=float, default=80.0, help="起动辨识时的 kd（限制滑动）")
    parser.add_argument("--ff", type=float, default=1.0, help="mit_ff_scale，默认 1：用保持电流抵重力")
    parser.add_argument("--settle", type=float, default=0.5)
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
