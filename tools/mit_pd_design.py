#!/usr/bin/env python3
"""MIT PD 参数设计：模型给初值 → 实机验证 → 按实测修正。

路线（不靠手感试凑）：
  1. 从 URDF 算每个关节在当前姿态的等效惯量 M_ii(q) = Σ m·|a×(p_com−o)|² + aᵀ(RIRᵀ)a，
     只算该关节子树里的连杆，其余关节视为固定。
  2. 按希望的固有频率和阻尼比算 PD 初值：
        Kp = J·ωn²                 [N·m/rad]
        Kd = 2·ζ·J·ωn − B_eff      [N·m·s/rad]，B_eff 未知时取 0
     再按每轴额定力矩换成硬件用的额定千分比。超过 1000/300‰ 的上限会截断，
     并重新算「封顶后的实际 ωn、ζ」。
  3. --verify 时下发参数、单轴小阶跃，量过冲、上升时间、稳态误差，
     和模型预测对比。--refine 再按实测过冲修正一次 Kd。

力矩单位：0x6076 是额定力矩，0x6071/0x6077 按额定千分比理解。零差手册把 6071
写成「额定电流千分比」，只有当额定力矩就是额定电流对应的输出力矩时两者数值一致。
所以这里的换算以 0x6076 为基准，换型号或固件时要用扭矩传感器复核。
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
LIMIT_BUDGET = [250, 150, 200, 400, 120, 250, 120]   # 单轴输出电流预算（‰）
# CST 斜坡辨识的起动电流（‰）。摩擦死区 = 该值/kp，用来判断刚度够不够。
FRICTION_EST = [41.4, 42.9, 41.7, 47.1, 24.9, 74.4, 39.7]
JOINT_LIMITS = {
    "Joint_r1": (-3.1415926, 3.1415926), "Joint_r2": (-3.1415926, 0.15),
    "Joint_r3": (-3.1415926, 3.1415926), "Joint_r4": (-2.0943951, 1.5707963),
    "Joint_r5": (-3.1415926, 3.1415926), "Joint_r6": (-1.7453292, 1.7453292),
    "Joint_r7": (-6.2831853, 6.2831853),
}


def _joint_limits(name: str):
    return JOINT_LIMITS.get(name, (-1.0, 1.0))


KP_LIMIT = 1000.0      # MitHandfeel::update 的硬上限（‰）
KD_LIMIT = 300.0


class PinBackend:
    """Pinocchio 后端：CRBA 惯量、重力项、多姿态扫描。

    手写的 effective_inertia() 和它数值一致（实测相对差 1e-15），保留是为了
    没装 pin 时也能用；有 pin 时优先走这里，因为它还能给重力项和姿态扫描。
    """

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
            # 把夹爪作为刚体挂到 TCP 帧上：质量放在帧原点、惯量张量未知按 0 处理。
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
        """重力力矩 N·m（右臂 r1..r7），给零重力/前馈用。"""
        vector = self.q_vector(q)
        torque = self.pin.computeGeneralizedGravity(self.model, self.data, vector)
        return [float(torque[self.index[f"Joint_r{i}"]]) for i in range(1, 8)]


def effective_inertia(model, q: Sequence[float]) -> List[float]:
    """右臂 r1..r7 在当前姿态的等效惯量（kg·m²）。

    只算沿该关节轴的转动惯量：连杆质心绕轴的贡献 m·|a×(p−o)|²，
    加上连杆自身惯性张量投影到轴上 aᵀ(RIRᵀ)a。其余关节按固定处理。
    """
    import numpy as np
    frames, axes = model.geometry(q)
    # 每个连杆属于哪些关节的子树，直接从 geometry 的祖先链取。
    result = []
    for index in range(8, 15):                       # JOINTS_ALL 里右臂是 8..14
        origin, axis = axes[index]
        total = 0.0
        for name, (mass, com, inertia) in model.links.items():
            position, rotation, chain = frames[name]
            if index not in chain:
                continue
            com_world = position + rotation @ com
            radius = com_world - origin
            total += mass * float(np.sum(np.cross(axis, radius) ** 2))
            total += float(axis @ (rotation @ inertia @ rotation.T) @ axis)
        result.append(total)
    return result


def design(inertia: Sequence[float], wn: float, zeta: float, b_eff: float = 0.0) -> List[dict]:
    """按 ωn、ζ 算 PD，并做单位换算和上限截断。"""
    rows = []
    for index, js in enumerate(inertia):
        kp_nm = js * wn * wn
        kd_nm = 2.0 * zeta * js * wn - b_eff
        kp_permille = kp_nm / RATED_TORQUE[index] * 1000.0
        kd_permille = max(0.0, kd_nm) / RATED_TORQUE[index] * 1000.0
        kp_used = min(kp_permille, KP_LIMIT)
        kd_used = min(kd_permille, KD_LIMIT)
        # 截断后实际的刚度和阻尼比：上限会显著改变响应，必须如实报出来。
        kp_real = kp_used / 1000.0 * RATED_TORQUE[index]
        kd_real = kd_used / 1000.0 * RATED_TORQUE[index]
        wn_real = math.sqrt(kp_real / js) if js > 0 else 0.0
        zeta_real = kd_real / (2.0 * math.sqrt(kp_real * js)) if js > 0 and kp_real > 0 else 0.0
        rows.append({
            "joint": index + 1, "inertia": js,
            "kp_nm_rad": kp_nm, "kd_nm_s_rad": kd_nm,
            "kp": kp_used, "kd": kd_used,
            "kp_raw": kp_permille, "kd_raw": kd_permille,
            "wn_real": wn_real, "zeta_real": zeta_real,
            "capped": kp_permille > KP_LIMIT or kd_permille > KD_LIMIT,
            "budget": LIMIT_BUDGET[index],
            # 静摩擦造成的死区：kp 越小越"晃荡"，这个数直接反映"能不能推动"。
            "deadband_deg": math.degrees(FRICTION_EST[index] / kp_used) if kp_used > 0 else 0.0,
            "saturation_rad": LIMIT_BUDGET[index] / kp_used if kp_used > 0 else 0.0,
        })
    return rows


def print_table(rows: List[dict], wn: float, zeta: float) -> None:
    print(f"\n设计目标：ωn={wn:.2f} rad/s ({wn / (2 * math.pi):.2f} Hz)，ζ={zeta:.2f}")
    print("关节   J (kg·m²)  Kp (N·m/rad)  Kd (N·m·s/rad)   kp‰    kd‰    实际 ωn  实际 ζ   死区")
    for row in rows:
        flag = "*" if row["capped"] else " "
        print(f"  r{row['joint']:<2} {row['inertia']:>10.4f} {row['kp_nm_rad']:>13.2f} "
              f"{row['kd_nm_s_rad']:>15.3f} {row['kp']:>6.0f}{flag} {row['kd']:>5.0f}  "
              f"{row['wn_real']:>8.2f} {row['zeta_real']:>7.2f} {row['deadband_deg']:>5.1f}°")
    if any(row["capped"] for row in rows):
        print("* 该轴按目标需要超过 1000/300‰ 上限，已截断；请参考「实际」两列。")
    print("\nMIT_KP = [" + ", ".join(f"{row['kp']:.1f}" for row in rows) + "]")
    print("MIT_KD = [" + ", ".join(f"{row['kd']:.1f}" for row in rows) + "]")


def metrics(times: Sequence[float], values: Sequence[float], start: float, target: float) -> dict:
    """阶跃响应的过冲、上升时间、稳定时间、稳态误差。"""
    span = target - start
    if abs(span) < 1e-6 or len(values) < 10:
        return {}
    final = sum(values[-len(values) // 5:]) / max(1, len(values) // 5)
    peak = max(values) if span > 0 else min(values)
    overshoot = (peak - target) / span if span > 0 else (target - peak) / span
    settled = [i for i, v in enumerate(values) if abs(v - final) > 0.05 * abs(span)]
    settling = times[settled[-1]] - times[0] if settled else 0.0
    lo, hi = start + 0.1 * span, start + 0.9 * span
    rise_idx = [i for i, v in enumerate(values) if min(lo, hi) <= v <= max(lo, hi)]
    rise = (times[rise_idx[-1]] - times[rise_idx[0]]) if rise_idx else 0.0
    return {
        "overshoot": overshoot, "rise_s": rise, "settling_s": settling,
        "final": final, "error": final - target,
    }


def zeta_from_overshoot(overshoot: float) -> Optional[float]:
    """Mp = exp(-πζ/√(1-ζ²)) 反解 ζ；过冲 <=0 视为过阻尼，不给估计。"""
    if overshoot <= 0.01 or overshoot >= 1.0:
        return None
    log_mp = math.log(overshoot)
    return -log_mp / math.sqrt(math.pi ** 2 + log_mp ** 2)


class Bench:
    """实机验证：下发参数、单轴阶跃、读 /joint_states。"""

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

    def step(self, joint: int, delta: float, settle: float) -> Optional[dict]:
        from std_msgs.msg import Float64MultiArray
        name = RIGHT_JOINTS[joint]
        q0 = self.measured[name]
        message = Float64MultiArray()
        message.data = [float(joint), float(q0), 0.0, float(self.kp[joint]), float(self.kd[joint]), 0.0]
        self.command_pub.publish(message)
        self.spin(0.4)
        start = self.measured[name]
        message.data = [float(joint), float(start + delta), 0.0,
                        float(self.kp[joint]), float(self.kd[joint]), 0.0]
        self.command_pub.publish(message)
        times, values = [], []
        end = time.monotonic() + settle
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
            if self.stamp and (not times or self.stamp > times[-1]):
                times.append(self.stamp)
                values.append(self.measured[name])
        message.data = [float(joint), float(start), 0.0, float(self.kp[joint]), float(self.kd[joint]), 0.0]
        self.command_pub.publish(message)
        self.spin(0.6)
        if not times:
            return None
        base = times[0]
        times = [t - base for t in times]
        return metrics(times, values, start, start + delta)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", default=str(Path(__file__).resolve().parents[1]
                                              / "src" / "arm_describle" / "urdf" / "HRB3Q-LC.urdf"))
    parser.add_argument("--gripper-kg", type=float, default=1.0, help="末端 TCP 质量（两侧同值）")
    parser.add_argument("--pose", default="live", choices=("live", "zero", "mechanical"),
                        help="live=读 /joint_states；zero=URDF 零位；mechanical=机械零位")
    parser.add_argument("--q", default="", help="手动给 15 个关节角（逗号分隔），覆盖 --pose")
    parser.add_argument("--wn", default="6.0",
                        help="设计固有频率 rad/s；用逗号给多个值可以并排比较")
    parser.add_argument("--zeta", type=float, default=1.0, help="设计阻尼比")
    parser.add_argument("--gravity", action="store_true", help="同时打印当前姿态的重力力矩")
    parser.add_argument("--sweep", type=int, default=0,
                        help="随机采样 N 个姿态，报每轴惯量的最小/最大值（不动机器人）")
    parser.add_argument("--rotor-j", default="",
                        help="转子惯量 kg·m²（7 个逗号分隔值，或 1 个通用值）；不填算 0")
    parser.add_argument("--gear", default="",
                        help="减速比（7 个逗号分隔值）；配合 --rotor-j 折算 N²·J_rotor")
    parser.add_argument("--b-eff", type=float, default=0.0,
                        help="各轴等效阻尼 N·m·s/rad，未知填 0；填了会从 Kd 里扣掉")
    parser.add_argument("--verify", action="store_true", help="下发参数并做单轴阶跃验证")
    parser.add_argument("--refine", action="store_true", help="按实测过冲修正一次 Kd（需 --verify）")
    parser.add_argument("--joints", type=lambda text: [int(v) - 1 for v in text.split(",")],
                        default=list(range(7)))
    parser.add_argument("--step", type=float, default=0.04, help="验证阶跃幅度 rad")
    parser.add_argument("--settle", type=float, default=2.0, help="每次记录时长 s")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    backend = None
    try:
        backend = PinBackend(args.urdf, args.gripper_kg)
        print("惯量后端：pinocchio")
    except Exception as error:                       # 没装 pin 时退回手写公式
        print(f"pinocchio 不可用（{error}），改用内置 URDF 公式")
        from gravity_identification.model import Model
        model = Model(args.urdf, (args.gripper_kg, args.gripper_kg))

    if args.q:
        q = [float(v) for v in args.q.replace(" ", "").split(",")]
        if len(q) != 15:
            print("--q 需要 15 个值")
            return 2
    elif args.pose == "mechanical":
        from mit_damping_identify import mechanical_pose
        pose = mechanical_pose()
        q = [pose[name] for name in JOINTS_ALL]
    elif args.pose == "zero":
        q = [0.0] * 15
    else:
        import rclpy
        rclpy.init()
        bench = Bench()
        try:
            if not bench.wait_ready():
                print("收不到 /joint_states；用 --pose zero 或 --pose mechanical 可离线算")
                return 2
            q = [bench.measured[name] for name in JOINTS_ALL]
        finally:
            bench.node.destroy_node()
            rclpy.shutdown()
        print("姿态取自实时反馈：")
        for name in RIGHT_JOINTS:
            print(f"  {name}: {q[JOINTS_ALL.index(name)]:+.4f}")

    inertia = backend.inertia(q) if backend else effective_inertia(model, q)
    # 转子惯量经减速比折算到输出侧：J_rotor_out = N²·J_rotor。URDF 里没有这一项，
    # 零差资料给了各型号转子惯量时补上，腕部尤其重要（连杆模型会把它们算成接近 0）。
    if args.rotor_j:
        rotors = [float(v) for v in args.rotor_j.split(",")]
        gears = [float(v) for v in args.gear.split(",")] if args.gear else [1.0] * 7
        if len(rotors) == 1:
            rotors *= 7
        if len(gears) == 1:
            gears *= 7
        if len(rotors) != 7 or len(gears) != 7:
            print("--rotor-j / --gear 需要 1 个或 7 个值")
            return 2
        inertia = [j + (n * n) * jr for j, jr, n in zip(inertia, rotors, gears)]
        print("已计入转子惯量 N²·J_rotor")
    if backend and args.gravity:
        torques = backend.gravity(q)
        print("当前姿态重力力矩（N·m）：" +
              "  ".join(f"r{i + 1}={value:+.2f}" for i, value in enumerate(torques)))
    weak = [index + 1 for index, value in enumerate(inertia) if value < 0.01]
    if weak:
        print(f"⚠ 关节 {weak} 的模型惯量 <0.01 kg·m²，连杆质量几乎都落在转轴上，"
              "结果会严重偏软；请补 --rotor-j/--gear 或改用实验值")
    if args.sweep and backend:
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
            low, high = min(values), max(values)
            print(f"  r{joint + 1:<2} {low:>9.4f} {high:>9.4f} {high / max(low, 1e-9):>6.1f}×")

    wn_values = [float(item) for item in str(args.wn).split(",") if item.strip()]
    candidates = []
    for wn in wn_values:
        rows = design(inertia, wn, args.zeta, args.b_eff)
        print_table(rows, wn, args.zeta)
        candidates.append((wn, rows))
    wn, rows = candidates[-1]

    if not args.verify:
        if args.out:
            Path(args.out).write_text(json.dumps({
                "pose": q, "wn": args.wn, "zeta": args.zeta, "rows": rows},
                ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"\n写入 {args.out}")
        return 0

    import rclpy
    rclpy.init()
    bench = Bench()
    bench.kp = [row["kp"] for row in rows]
    bench.kd = [row["kd"] for row in rows]
    report: List[dict] = []
    try:
        if not bench.wait_ready():
            print("收不到 /joint_states")
            return 2
        if not bench.push_params(bench.kp, bench.kd, 0.0):
            print("参数下发失败")
            return 2
        ok, detail = bench.trigger(bench.enter_client, "/x1/current_gravity_handfeel")
        if not ok:
            print(f"进入 MIT 失败：{detail}")
            return 2
        try:
            print("\n验证（实机阶跃，请有人扶着）")
            for joint in args.joints:
                result = bench.step(joint, args.step, args.settle)
                if not result:
                    continue
                zeta_meas = zeta_from_overshoot(result["overshoot"])
                row = rows[joint]
                delta_kd = ""
                if args.refine and zeta_meas and zeta_meas > 0.05:
                    ratio = args.zeta / max(zeta_meas, 0.05)
                    kd_new = min(KD_LIMIT, row["kd"] * ratio)
                    delta_kd = f"  → 建议 kd {row['kd']:.0f}→{kd_new:.0f}"
                    row["kd_refined"] = kd_new
                zeta_text = f"{zeta_meas:.2f}" if zeta_meas else "过阻尼"
                print(f"  r{joint + 1}: 过冲 {result['overshoot'] * 100:>5.1f}%  "
                      f"上升 {result['rise_s']:.2f}s  稳定 {result['settling_s']:.2f}s  "
                      f"稳态误差 {result['error']:+.4f} rad  实测 ζ≈{zeta_text}{delta_kd}")
                report.append({"joint": joint + 1, **result, "zeta_measured": zeta_meas})
        finally:
            ok, detail = bench.trigger(bench.exit_client, "/x1/current_gravity_exit")
            print(f"\n退出 MIT：{'成功' if ok else detail}")
    finally:
        bench.node.destroy_node()
        rclpy.shutdown()

    if args.out:
        Path(args.out).write_text(json.dumps({
            "pose": q, "wn": args.wn, "zeta": args.zeta, "rows": rows, "verify": report},
            ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"写入 {args.out}")
    if args.refine:
        refined = [row.get("kd_refined", row["kd"]) for row in rows]
        print("\n修正后的 MIT_KD = [" + ", ".join(f"{value:.1f}" for value in refined) + "]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
