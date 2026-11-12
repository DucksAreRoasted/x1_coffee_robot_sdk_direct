#!/usr/bin/env python3
"""MIT PD 参数整定：模型给初值 → 候选筛选 → 实机验证 → 记录各部分力矩。

四步流程：
  1. 单位核对：0x6076 额定力矩是换算基准；0x6071/0x6077 按额定千分比理解。
     零差手册写的是「额定电流千分比」，只有额定力矩=额定电流对应输出力矩时两者数值相同。
     换型号或固件必须用扭矩传感器复核，这里的换算不替代实测标定。
  2. 惯量：Pinocchio CRBA 取 M_ii(q)，可选叠加转子惯量 N²·J_rotor（URDF 里没有这一项，
     腕部 r5/r7 会因此严重偏小）。--sweep 可以看多姿态的范围。
  3. 候选：在 (f_n, ζ) 网格上生成 PD：
        Kp = J·ωn²            Kd = 2ζ·J·ωn − B_eff
     换算成额定千分比后被 1000/300 上限截断，按截断后的真实 ωn/ζ 报告；
     再按测试阶跃估算力矩需求，超过单轴预算的组合直接筛掉。
  4. 验证：只把前 N 个候选下发到实机，单轴小幅阶跃，记录
     tau_p / tau_d / 总力矩 / 饱和占比 / 过冲 / 上升时间 / 稳态误差，写 CSV。

拖动（零重力）模式另作一套目标：Kp=0，Kd ≈ J/Tv（Tv 是松手后的速度衰减时间常数），
不要拿位置跟踪的 Kd 直接套用。
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

JOINTS_ALL = ["Joint_Body"] + [f"Joint_{s}{i}" for s in "lr" for i in range(1, 8)]
RIGHT_JOINTS = [f"Joint_r{i}" for i in range(1, 8)]
RATED_TORQUE = [52.0, 52.0, 21.0, 21.0, 10.0, 5.4, 5.4]
LIMIT_BUDGET = [250, 150, 200, 400, 120, 250, 120]
FRICTION_EST = [41.4, 42.9, 41.7, 47.1, 24.9, 74.4, 39.7]
KP_LIMIT = 1000.0
KD_LIMIT = 300.0
JOINT_LIMITS = {
    "Joint_r1": (-3.1415926, 3.1415926), "Joint_r2": (-3.1415926, 0.15),
    "Joint_r3": (-3.1415926, 3.1415926), "Joint_r4": (-2.0943951, 1.5707963),
    "Joint_r5": (-3.1415926, 3.1415926), "Joint_r6": (-1.7453292, 1.7453292),
    "Joint_r7": (-6.2831853, 6.2831853),
}


def _joint_limits(name: str):
    return JOINT_LIMITS.get(name, (-1.0, 1.0))


class PinBackend:
    """Pinocchio 后端：CRBA 惯量、重力项、多姿态扫描。"""

    def __init__(self, urdf: str, gripper_kg: float = 1.0) -> None:
        import numpy as np
        import pinocchio as pin
        self.np = np
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.index = {name: self.model.joints[self.model.getJointId(name)].idx_q
                      for name in JOINTS_ALL}
        if gripper_kg > 0:
            for frame_name in ("left_gripper_tcp", "right_gripper_tcp"):
                fid = self.model.getFrameId(frame_name)
                if fid >= len(self.model.frames):
                    continue
                frame = self.model.frames[fid]
                body = pin.Inertia(float(gripper_kg), np.zeros(3), np.zeros((3, 3)))
                self.model.appendBodyToJoint(frame.parentJoint, body, frame.placement)
            self.data = self.model.createData()

    def q_vector(self, q: Sequence[float]):
        vector = self.pin.neutral(self.model)
        for name, value in zip(JOINTS_ALL, q):
            vector[self.index[name]] = value
        return vector

    def inertia(self, q: Sequence[float]) -> List[float]:
        vector = self.q_vector(q)
        matrix = self.pin.crba(self.model, self.data, vector)
        return [float(matrix[self.index[f"Joint_r{i}"], self.index[f"Joint_r{i}"]])
                for i in range(1, 8)]

    def gravity(self, q: Sequence[float]) -> List[float]:
        vector = self.q_vector(q)
        torque = self.pin.computeGeneralizedGravity(self.model, self.data, vector)
        return [float(torque[self.index[f"Joint_r{i}"]]) for i in range(1, 8)]


def _row(index: int, js: float, kp_nm: float, kd_nm: float) -> dict:
    kp_permille = kp_nm / RATED_TORQUE[index] * 1000.0
    kd_permille = kd_nm / RATED_TORQUE[index] * 1000.0
    kp_used = min(kp_permille, KP_LIMIT)
    kd_used = min(kd_permille, KD_LIMIT)
    kp_real = kp_used / 1000.0 * RATED_TORQUE[index]
    kd_real = kd_used / 1000.0 * RATED_TORQUE[index]
    wn_real = math.sqrt(kp_real / js) if js > 0 else 0.0
    zeta_real = (kd_real / (2.0 * math.sqrt(kp_real * js))
                 if js > 0 and kp_real > 0 else 0.0)
    return {
        "joint": index + 1, "inertia": js,
        "kp_nm_rad": kp_real, "kd_nm_s_rad": kd_real,
        "kp": kp_used, "kd": kd_used, "kp_raw": kp_permille, "kd_raw": kd_permille,
        "wn_real": wn_real, "zeta_real": zeta_real,
        "capped": kp_permille > KP_LIMIT or kd_permille > KD_LIMIT,
        "budget": LIMIT_BUDGET[index],
        "budget_nm": LIMIT_BUDGET[index] / 1000.0 * RATED_TORQUE[index],
        "deadband_deg": math.degrees(FRICTION_EST[index] / kp_used) if kp_used > 0 else 999.0,
    }


def design_position(inertia: Sequence[float], wn: float, zeta: float,
                    b_eff: float = 0.0) -> List[dict]:
    """位置保持：Kp = Jωn²，Kd = 2ζJωn − B。"""
    return [_row(index, js, js * wn * wn, max(0.0, 2.0 * zeta * js * wn - b_eff))
            for index, js in enumerate(inertia)]


def design_drag(inertia: Sequence[float], tv: float, b_eff: float = 0.0) -> List[dict]:
    """零重力拖动：Kp=0，Kd ≈ J/Tv − B。"""
    return [_row(index, js, 0.0, max(0.0, js / max(tv, 1e-3) - b_eff))
            for index, js in enumerate(inertia)]


def candidate_metrics(rows: List[dict], step: float, skip: Optional[Sequence[int]] = None) -> dict:
    """按测试阶跃算力矩需求：P 项 |Kp·step| 就是阶跃瞬间的峰值需求。

    skip 里是模型不可信的关节（惯量 <0.01 kg·m²），它们的死区不参与筛选。
    """
    skip = set(skip or [])
    used = [row for row in rows if (row["joint"] - 1) not in skip] or rows
    saturations = [abs(row["kp_nm_rad"] * step) / row["budget_nm"]
                   if row["budget_nm"] > 0 else 99.0 for row in used]
    return {
        "max_saturation": max(saturations),
        "mean_deadband": sum(row["deadband_deg"] for row in used) / len(used),
        "capped": sum(1 for row in rows if row["capped"]),
        "min_zeta": min(row["zeta_real"] for row in rows),
    }


def print_design(rows: List[dict], title: str) -> None:
    print(f"\n{title}")
    print("关节   J (kg·m²)  Kp (N·m/rad)  Kd (N·m·s/rad)   kp‰    kd‰    实际 ωn  实际 ζ   死区")
    for row in rows:
        flag = "*" if row["capped"] else " "
        print(f"  r{row['joint']:<2} {row['inertia']:>10.4f} {row['kp_nm_rad']:>13.2f} "
              f"{row['kd_nm_s_rad']:>15.3f} {row['kp']:>6.0f}{flag} {row['kd']:>5.0f}  "
              f"{row['wn_real']:>8.2f} {row['zeta_real']:>7.2f} {row['deadband_deg']:>5.1f}°")
    print("MIT_KP = [" + ", ".join(f"{row['kp']:.1f}" for row in rows) + "]")
    print("MIT_KD = [" + ", ".join(f"{row['kd']:.1f}" for row in rows) + "]")


def metrics(times: Sequence[float], values: Sequence[float], start: float, target: float) -> dict:
    span = target - start
    if abs(span) < 1e-6 or len(values) < 10:
        return {}
    final = sum(values[-len(values) // 5:]) / max(1, len(values) // 5)
    peak = max(values) if span > 0 else min(values)
    overshoot = (peak - target) / span if span > 0 else (target - peak) / span
    settled = [i for i, value in enumerate(values) if abs(value - final) > 0.05 * abs(span)]
    settling = times[settled[-1]] - times[0] if settled else 0.0
    lo, hi = start + 0.1 * span, start + 0.9 * span
    rise_idx = [i for i, value in enumerate(values) if min(lo, hi) <= value <= max(lo, hi)]
    rise = (times[rise_idx[-1]] - times[rise_idx[0]]) if rise_idx else 0.0
    return {"overshoot": overshoot, "rise_s": rise, "settling_s": settling,
            "final": final, "error": final - target}


def zeta_from_overshoot(overshoot: float) -> Optional[float]:
    if overshoot <= 0.01 or overshoot >= 1.0:
        return None
    log_mp = math.log(overshoot)
    return -log_mp / math.sqrt(math.pi ** 2 + log_mp ** 2)


class Bench:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import SetParameters

        self.rclpy = rclpy
        self.measured: Dict[str, float] = {}
        self.stamp = 0.0
        self.node = Node("mit_pd_design")
        self.node.create_subscription(JointState, "/joint_states", self._on_joint, qos_profile_sensor_data)
        self.command_pub = self.node.create_publisher(Float64MultiArray, "/x1/mit_command", 10)
        self.params_client = self.node.create_client(
            SetParameters, "/x1_coffee_system_hardware/set_parameters")
        self.enter_client = self.node.create_client(Trigger, "/x1/current_gravity_handfeel")
        self.exit_client = self.node.create_client(Trigger, "/x1/current_gravity_exit")

    def _on_joint(self, message) -> None:
        for name, position in zip(message.name, message.position):
            self.measured[name] = float(position)
        self.stamp = time.monotonic()

    def spin(self, duration: float) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_ready(self, timeout: float = 10.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if all(name in self.measured for name in JOINTS_ALL) and time.monotonic() - self.stamp < 0.5:
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
            print("没有参数服务")
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

    def step(self, joint: int, delta: float, settle: float, kp: float, kd: float,
             budget_nm: float, out_dir: Optional[Path]) -> Optional[dict]:
        """单轴阶跃并记录位置、P 项、D 项、总力矩、饱和占比。"""
        from std_msgs.msg import Float64MultiArray
        name = RIGHT_JOINTS[joint]
        q0 = self.measured[name]
        message = Float64MultiArray()
        message.data = [float(joint), float(q0), 0.0, float(kp), float(kd), 0.0]
        self.command_pub.publish(message)
        self.spin(0.4)
        start = self.measured[name]
        target = start + delta
        message.data = [float(joint), float(target), 0.0, float(kp), float(kd), 0.0]
        self.command_pub.publish(message)

        times: List[float] = []
        values: List[float] = []
        trace: List[List[float]] = []
        previous = start
        velocity = 0.0
        end = time.monotonic() + settle
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
            if not self.stamp or (times and self.stamp <= times[-1]):
                continue
            q = self.measured[name]
            dt = (self.stamp - times[-1]) if times else 0.01
            if dt <= 0:
                dt = 1e-3
            velocity += 0.2 * ((q - previous) / dt - velocity)
            previous = q
            times.append(self.stamp)
            values.append(q)
            tau_p = kp / 1000.0 * RATED_TORQUE[joint] * (target - q)
            tau_d = -kd / 1000.0 * RATED_TORQUE[joint] * velocity
            total = tau_p + tau_d
            trace.append([times[-1], q, target, tau_p, tau_d, total,
                          abs(total) / budget_nm if budget_nm > 0 else 0.0])
        message.data = [float(joint), float(start), 0.0, float(kp), float(kd), 0.0]
        self.command_pub.publish(message)
        self.spin(0.6)

        if not times:
            return None
        base = times[0]
        times = [t - base for t in times]
        result = metrics(times, values, start, target)
        if not result:
            return None
        result.update({
            "joint": joint + 1,
            "peak_tau": max(abs(row[5]) for row in trace),
            "peak_tau_p": max(abs(row[3]) for row in trace),
            "peak_tau_d": max(abs(row[4]) for row in trace),
            "saturation_ratio": sum(1 for row in trace if row[6] > 1.0) / len(trace),
            "zeta_measured": zeta_from_overshoot(result["overshoot"]),
        })
        if out_dir is not None:
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / f"j{joint + 1}.csv"
            with path.open("w", encoding="utf-8") as handle:
                handle.write("t,q,q_des,tau_p,tau_d,tau_total,saturation\n")
                for row in trace:
                    handle.write(",".join(
                        f"{row[0] - base:.6f}" if index == 0 else f"{value:.6f}"
                        for index, value in enumerate(row)) + "\n")
            result["csv"] = str(path)
        return result


def load_pose(args) -> Optional[List[float]]:
    if args.q:
        q = [float(v) for v in args.q.replace(" ", "").split(",")]
        return q if len(q) == 15 else None
    if args.pose == "mechanical":
        from mit_damping_identify import mechanical_pose
        pose = mechanical_pose()
        return [pose[name] for name in JOINTS_ALL]
    if args.pose == "zero":
        return [0.0] * 15
    import rclpy
    rclpy.init()
    bench = Bench()
    try:
        if not bench.wait_ready():
            print("收不到 /joint_states；用 --pose zero/mechanical 可离线算")
            return None
        print("姿态取自实时反馈")
        return [bench.measured[name] for name in JOINTS_ALL]
    finally:
        bench.node.destroy_node()
        rclpy.shutdown()


def load_inertia(args, q: Sequence[float]) -> Optional[List[float]]:
    backend = None
    try:
        backend = PinBackend(args.urdf, args.gripper_kg)
        print("惯量后端：pinocchio")
    except Exception as error:
        print(f"pinocchio 不可用（{error}），改用内置 URDF 公式")
    if backend is not None:
        inertia = backend.inertia(q)
    else:
        import numpy as np
        from gravity_identification.model import Model
        model = Model(args.urdf, (args.gripper_kg, args.gripper_kg))
        frames, axes = model.geometry(q)
        inertia = []
        for index in range(8, 15):
            origin, axis = axes[index]
            total = 0.0
            for name, (mass, com, matrix) in model.links.items():
                position, rotation, chain = frames[name]
                if index not in chain:
                    continue
                radius = position + rotation @ com - origin
                total += mass * float(np.sum(np.cross(axis, radius) ** 2))
                total += float(axis @ (rotation @ matrix @ rotation.T) @ axis)
            inertia.append(total)

    if args.rotor_j:
        rotors = [float(v) for v in args.rotor_j.split(",")]
        gears = [float(v) for v in args.gear.split(",")] if args.gear else [1.0] * 7
        rotors = rotors * 7 if len(rotors) == 1 else rotors
        gears = gears * 7 if len(gears) == 1 else gears
        if len(rotors) != 7 or len(gears) != 7:
            print("--rotor-j / --gear 需要 1 个或 7 个值")
            return None
        inertia = [value + (n * n) * rotor for value, rotor, n in zip(inertia, rotors, gears)]
        print("已计入转子惯量 N²·J_rotor")

    weak = [index + 1 for index, value in enumerate(inertia) if value < 0.01]
    if weak:
        print(f"⚠ 关节 {weak} 模型惯量 <0.01 kg·m²，腕部连杆几乎都落在转轴上；"
              "补 --rotor-j/--gear 后才有意义")
    if backend is not None and args.gravity:
        print("当前姿态重力力矩（N·m）：" + "  ".join(
            f"r{i + 1}={value:+.2f}" for i, value in enumerate(backend.gravity(q))))
    if backend is not None and args.sweep:
        import random
        random.seed(0)
        samples = []
        for _ in range(args.sweep):
            pose = list(q)
            for index in range(8, 15):
                low, high = _joint_limits(JOINTS_ALL[index])
                pose[index] = random.uniform(low, high)
            samples.append(backend.inertia(pose))
        print(f"\n{args.sweep} 个随机姿态下的惯量范围（kg·m²）：")
        print("关节    最小      最大      倍数")
        for joint in range(7):
            values = [sample[joint] for sample in samples]
            print(f"  r{joint + 1:<2} {min(values):>9.4f} {max(values):>9.4f} "
                  f"{max(values) / max(min(values), 1e-9):>6.1f}×")
    return inertia, weak


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", default=str(Path(__file__).resolve().parents[1]
                                              / "src" / "arm_describle" / "urdf" / "HRB3Q-LC.urdf"))
    parser.add_argument("--gripper-kg", type=float, default=1.0)
    parser.add_argument("--pose", default="live", choices=("live", "zero", "mechanical"))
    parser.add_argument("--q", default="")
    parser.add_argument("--mode", choices=("position", "drag"), default="position")
    parser.add_argument("--fn", default="2,4,6", help="候选固有频率 Hz，逗号分隔")
    parser.add_argument("--zeta", default="0.7,1.0", help="候选阻尼比，逗号分隔")
    parser.add_argument("--tv", type=float, default=0.15,
                        help="drag 模式的速度衰减时间常数 s（Kd≈J/Tv）")
    parser.add_argument("--rotor-j", default="", help="转子惯量 kg·m²，1 个或 7 个值")
    parser.add_argument("--gear", default="", help="减速比，1 个或 7 个值")
    parser.add_argument("--step", type=float, default=0.04, help="验证阶跃幅度 rad")
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--verify", action="store_true", help="把筛选后的候选下发实机验证")
    parser.add_argument("--verify-top", type=int, default=2)
    parser.add_argument("--max-deadband", type=float, default=5.0,
                        help="平均摩擦死区上限（度）；超过说明 kp 太软，直接剔除")
    parser.add_argument("--min-zeta", type=float, default=0.7,
                        help="候选里实际阻尼比不得低于这个值")
    parser.add_argument("--joints", type=lambda text: [int(v) - 1 for v in text.split(",")],
                        default=list(range(7)))
    parser.add_argument("--gravity", action="store_true")
    parser.add_argument("--sweep", type=int, default=0)
    parser.add_argument("--out", default=f"log/mit_pd_{time.strftime('%Y%m%d_%H%M%S')}")
    args = parser.parse_args()

    from x1_ros_env import ensure_ros_env
    ensure_ros_env()
    q = load_pose(args)
    if q is None:
        return 2
    loaded = load_inertia(args, q)
    if loaded is None:
        return 2
    inertia, weak = loaded

    weak_index = [value - 1 for value in weak]
    candidates = []
    if args.mode == "position":
        for fn in [float(v) for v in args.fn.split(",") if v.strip()]:
            for zeta in [float(v) for v in args.zeta.split(",") if v.strip()]:
                rows = design_position(inertia, 2 * math.pi * fn, zeta)
                candidates.append({"fn": fn, "zeta": zeta, "rows": rows,
                                   **candidate_metrics(rows, args.step, weak_index)})
    else:
        rows = design_drag(inertia, args.tv)
        candidates.append({"fn": 0.0, "zeta": 0.0, "tv": args.tv, "rows": rows,
                           **candidate_metrics(rows, args.step, weak_index)})

    print("\n候选筛选（按测试阶跃 %.3f rad 估算力矩需求）" % args.step)
    print("   fn(Hz)  ζ    封顶轴  最大饱和  平均死区  最小实际ζ")
    feasible = []
    for item in candidates:
        reasons = []
        if item["max_saturation"] > 1.0:
            reasons.append("力矩超预算")
        if item["mean_deadband"] > args.max_deadband:
            reasons.append(f"死区>{args.max_deadband:.0f}°")
        if item["min_zeta"] < args.min_zeta - 1e-6:
            reasons.append("ζ不足")
        tag = ("  ← 剔除：" + "、".join(reasons)) if reasons else "  ← 通过"
        print(f"   {item['fn']:>6.2f} {item['zeta']:>4.1f} {item['capped']:>6} "
              f"{item['max_saturation']:>8.2f} {item['mean_deadband']:>8.1f}° "
              f"{item['min_zeta']:>8.2f}{tag}")
        if not reasons:
            feasible.append(item)
    if not feasible:
        print("没有候选同时满足力矩预算/死区/阻尼；放宽 --max-deadband 或降低 --fn")
        feasible = sorted(candidates, key=lambda item: item["max_saturation"])[:1]

    # 通过的候选里挑刚度最高的几个送实机，避免一路选到最软的那档。
    shortlist = sorted(feasible, key=lambda item: (-item["fn"], item["max_saturation"]))
    shortlist = shortlist[:max(1, args.verify_top)]
    for item in shortlist:
        label = (f"f_n={item['fn']:.2f}Hz ζ={item['zeta']:.2f}" if args.mode == "position"
                 else f"drag Tv={args.tv:.2f}s")
        print_design(item["rows"], label)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "design.json").write_text(json.dumps({
        "pose": q, "mode": args.mode, "step": args.step,
        "candidates": candidates}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n写入 {out}/design.json")
    if not args.verify:
        return 0

    import rclpy
    rclpy.init()
    bench = Bench()
    results = []
    try:
        if not bench.wait_ready():
            print("收不到 /joint_states")
            return 2
        for item in shortlist:
            label = (f"fn{item['fn']:.1f}_z{item['zeta']:.2f}" if args.mode == "position"
                     else "drag")
            if not bench.push_params([row["kp"] for row in item["rows"]],
                                     [row["kd"] for row in item["rows"]], 0.0):
                print("参数下发失败")
                return 2
            ok, detail = bench.trigger(bench.enter_client, "/x1/current_gravity_handfeel")
            if not ok:
                print(f"进入 MIT 失败：{detail}")
                return 2
            print(f"\n=== 验证 {label}")
            try:
                for joint in args.joints:
                    row = item["rows"][joint]
                    result = bench.step(joint, args.step, args.settle,
                                        row["kp"], row["kd"], row["budget_nm"], out / label)
                    if not result:
                        continue
                    zeta_text = (f"{result['zeta_measured']:.2f}"
                                 if result["zeta_measured"] else "过阻尼")
                    print(f"  r{joint + 1}: 过冲 {result['overshoot'] * 100:>5.1f}%  "
                          f"上升 {result['rise_s']:.2f}s  稳定 {result['settling_s']:.2f}s  "
                          f"峰值力矩 {result['peak_tau']:.2f}N·m "
                          f"(P {result['peak_tau_p']:.2f} / D {result['peak_tau_d']:.2f})  "
                          f"饱和 {result['saturation_ratio'] * 100:.0f}%  "
                          f"稳态误差 {result['error']:+.4f}  实测ζ≈{zeta_text}")
                    results.append({"candidate": label, **result})
            finally:
                bench.trigger(bench.exit_client, "/x1/current_gravity_exit")
        (out / "verify.json").write_text(json.dumps(results, ensure_ascii=False, indent=2),
                                         encoding="utf-8")
        print(f"\n验证数据写入 {out}/verify.json 和 {out}/<候选>/j*.csv")
    finally:
        bench.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
