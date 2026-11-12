#!/usr/bin/env python3
"""右上肢 MIT 阶跃响应辨识：逐轴测阻尼比 ζ 和固有频率 ω_n。

思路：单轴给一个小阶跃（q_des 跳 --step 弧度），记录 /joint_states 的响应，
按二阶欠阻尼模型拟合 q(t) = q_end - e^{-ζωt}[a·cos(ω_d t) + b·sin(ω_d t)]。
有了 ζ 和 ω_n 就能反推：J_eff = K/ω_n²，再用它算「ζ=0.7 需要多少 kd」。

流程：
  1. --home 时先用 /x1/pvt_move 把 15 轴送回机械零位，等停稳
  2. 下发 kp/kd（mit_ff_scale=0，纯 PD）并进入 /x1/current_gravity_handfeel
  3. 逐轴阶跃、记录、拟合；测完把 q_des 还原
  4. 退出 MIT，打印表格，写 JSON 和每轴原始曲线 CSV

安全：单轴阶跃 ≤ 0.06 rad；只测右臂；跟随偏差超过 0.3 rad、或服务返回失败，
立刻退出 MIT。整个过程中手臂会按阶跃来回动，必须有人在场、随时可按急停。
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

RIGHT_JOINTS = [f"Joint_r{index}" for index in range(1, 8)]
ALL_JOINTS = ["Joint_Body", *[f"Joint_l{index}" for index in range(1, 8)], *RIGHT_JOINTS]
# 每轴额定力矩（0x6076 实测读回），用来把 kp/kd 的千分比换算成 N·m。
RATED_TORQUE = [52.0, 52.0, 21.0, 21.0, 10.0, 5.4, 5.4]
# URDF 在标定姿态下的折算惯量，只用来给理论阻尼比做对照。
URDF_INERTIA = [0.330, 0.198, 0.300, 0.215, 0.087, 0.132, 0.0013]
JOINT_LIMITS = {
    "Joint_r1": (-3.1415926, 3.1415926),
    "Joint_r2": (-3.1415926, 0.15),
    "Joint_r3": (-3.1415926, 3.1415926),
    "Joint_r4": (-2.0943951, 1.5707963),
    "Joint_r5": (-3.1415926, 3.1415926),
    "Joint_r6": (-1.7453292, 1.7453292),
    "Joint_r7": (-6.2831853, 6.2831853),
}
MAX_STEP_RAD = 0.06
MAX_DEVIATION_RAD = 0.30
MAX_JOINT_VELOCITY = 0.4
MIN_SEGMENT_SEC = 0.8


def mechanical_pose() -> Dict[str, float]:
    """机械零位：工厂编码器零点对应的关节角，来自 initial_positions.yaml。"""
    candidates: List[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory("x1_coffee_integration")) / "config" / "initial_positions.yaml")
    except Exception:
        pass
    candidates.append(Path(__file__).resolve().parents[1]
                      / "src" / "x1_coffee_integration" / "config" / "initial_positions.yaml")
    import yaml
    for path in candidates:
        if not path.is_file():
            continue
        values = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("initial_positions") or {}
        if all(name in values for name in ALL_JOINTS):
            return {name: float(values[name]) for name in ALL_JOINTS}
    raise RuntimeError("找不到 initial_positions.yaml，无法确定机械零位")


def segment_seconds(start: Dict[str, float], goal: Dict[str, float]) -> float:
    """按最大关节速度拉伸时长，避免一次大跨度回零走得太快。"""
    delta = max(abs(goal[name] - start.get(name, 0.0)) for name in ALL_JOINTS)
    if delta <= 1.0e-9:
        return MIN_SEGMENT_SEC
    return max(MIN_SEGMENT_SEC, delta / MAX_JOINT_VELOCITY)


def fit_second_order(t: Sequence[float], y: Sequence[float]) -> Optional[dict]:
    """拟合 y(t) = e^{-ζωt}[a·cos(ω_d t) + b·sin(ω_d t)]。

    给定 (ω, ζ) 时 a、b 是线性的，用最小二乘直接解；再对 (ω, ζ) 做粗网格 + 细化。
    y 必须已经减去终值，t 以秒为单位、从阶跃时刻起算。
    """
    if len(t) < 12 or len(y) < 12:
        return None
    t0 = t[0]
    ts = [value - t0 for value in t]

    def solve(wn: float, zeta: float) -> Tuple[float, float, float]:
        decay = wn * zeta
        wd = wn * math.sqrt(max(1.0 - zeta * zeta, 1.0e-9)) if zeta < 1.0 else 0.0
        cc = cs = ss = cy = sy = 0.0
        for ti, yi in zip(ts, y):
            envelope = math.exp(-decay * ti)
            if zeta < 1.0:
                c = envelope * math.cos(wd * ti)
                s = envelope * math.sin(wd * ti)
            else:  # 过阻尼：用双指数基，仍能给出等效时间常数
                c = envelope
                s = envelope * ti
            cc += c * c
            cs += c * s
            ss += s * s
            cy += c * yi
            sy += s * yi
        det = cc * ss - cs * cs
        if abs(det) < 1.0e-12:
            return 0.0, 0.0, float("inf")
        a = (cy * ss - sy * cs) / det
        b = (sy * cc - cy * cs) / det
        rms = 0.0
        for ti, yi in zip(ts, y):
            envelope = math.exp(-decay * ti)
            if zeta < 1.0:
                c = envelope * math.cos(wd * ti)
                s = envelope * math.sin(wd * ti)
            else:
                c = envelope
                s = envelope * ti
            rms += (a * c + b * s - yi) ** 2
        return a, b, math.sqrt(rms / len(ts))

    best = (float("inf"), 3.0, 0.3)
    for wn in [0.5 * 1.15 ** k for k in range(28)]:          # 0.5 ~ 24 rad/s
        for zeta in [0.05 + 0.05 * k for k in range(39)]:     # 0.05 ~ 1.95
            _, _, rms = solve(wn, zeta)
            if rms < best[0]:
                best = (rms, wn, zeta)
    coarse = best
    # 在最好的格子附近细化一次，避免网格分辨率限制精度。
    for wn in [coarse[1] * (0.94 + 0.012 * k) for k in range(11)]:
        for zeta in [max(0.02, coarse[2] + 0.01 * (k - 5)) for k in range(11)]:
            _, _, rms = solve(wn, zeta)
            if rms < best[0]:
                best = (rms, wn, zeta)
    return {"rms": best[0], "wn": best[1], "zeta": best[2]}


class Identifier:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import SetParameters

        self.rclpy = rclpy
        self.args = args
        self.measured: Dict[str, float] = {}
        self.stamp = 0.0
        self.pvt_result: Optional[str] = None
        self.pvt_seq = 0
        self.joint = 0

        self.node = Node("mit_damping_identify")
        self.node.create_subscription(JointState, "/joint_states", self._on_joint, qos_profile_sensor_data)
        self.node.create_subscription(String, "/x1/pvt_result", self._on_pvt_result, 10)
        self.pvt_pub = self.node.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
        self.mit_command_pub = self.node.create_publisher(Float64MultiArray, "/x1/mit_command", 10)
        self.params_client = self.node.create_client(
            SetParameters, "/x1_coffee_system_hardware/set_parameters")
        self.handfeel_client = self.node.create_client(Trigger, "/x1/current_gravity_handfeel")
        self.exit_client = self.node.create_client(Trigger, "/x1/current_gravity_exit")

    def _on_joint(self, message) -> None:
        for name, position in zip(message.name, message.position):
            self.measured[name] = float(position)
        self.stamp = time.monotonic()

    def _on_pvt_result(self, message) -> None:
        self.pvt_result = message.data

    def spin(self, duration: float) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_joint_states(self, timeout: float = 10.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if all(name in self.measured for name in ALL_JOINTS) and time.monotonic() - self.stamp < 0.5:
                return True
        return False

    def trigger(self, client, label: str) -> Tuple[bool, str]:
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
            print("没有参数服务 /x1_coffee_system_hardware/set_parameters")
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
        ok = result is not None and all(item.successful for item in result.results)
        if not ok:
            print("MIT 参数下发失败")
        return ok

    def send_home(self) -> bool:
        from std_msgs.msg import Float64MultiArray
        goal = mechanical_pose()
        duration = segment_seconds(self.measured, goal)
        self.pvt_seq += 1
        self.pvt_result = None
        message = Float64MultiArray()
        message.data = [float(self.pvt_seq), duration, 1.0, *[goal[name] for name in ALL_JOINTS]]
        self.pvt_pub.publish(message)
        print(f"\n回机械零位：最大行程 "
              f"{max(abs(goal[name] - self.measured[name]) for name in ALL_JOINTS):.3f} rad，约 {duration:.1f} s")
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
                    break
                if same:
                    print(f"位置目标失败: {self.pvt_result}")
                    return False
        else:
            print("位置目标没有回应")
            return False
        # 等它真的停稳：最大偏差小于 0.02 rad。
        end = time.monotonic() + duration + 8.0
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if max(abs(self.measured[name] - goal[name]) for name in ALL_JOINTS) < 0.02:
                print("已到机械零位")
                return True
        print("回零超时，最大偏差 "
              f"{max(abs(self.measured[name] - goal[name]) for name in ALL_JOINTS):.3f} rad")
        return False

    def record(self, duration: float) -> Tuple[List[float], List[float]]:
        times: List[float] = []
        values: List[float] = []
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
            if self.stamp and (not times or self.stamp > times[-1]):
                times.append(self.stamp)
                values.append(self.measured.get(RIGHT_JOINTS[self.joint], float("nan")))
        return times, values

    def run(self) -> int:
        joints = self.args.joints
        print(__doc__.split("\n\n")[0])
        if self.args.dry_run:
            print("干跑：不会连接硬件。参数 " + f"kp={self.args.kp} kd={self.args.kd} step={self.args.step}")
            return 0
        if not self.wait_joint_states():
            print("收不到 15 轴 /joint_states，先确认下位机控制栈已起来")
            return 2
        if self.args.home and not self.send_home():
            return 2
        kp = [self.args.kp] * 7
        kd = [self.args.kd] * 7
        if not self.push_params(kp, kd, 0.0):
            return 2
        ok, detail = self.trigger(self.handfeel_client, "/x1/current_gravity_handfeel")
        if not ok:
            print(f"进入 MIT 失败：{detail}")
            return 2
        print(f"已进入 MIT 手感：kp={self.args.kp} kd={self.args.kd} ff=0")
        results: List[dict] = []
        out_dir = Path(self.args.out)
        out_dir.mkdir(parents=True, exist_ok=True)
        try:
            for joint in joints:
                self.joint = joint
                name = RIGHT_JOINTS[joint]
                low, high = JOINT_LIMITS[name]
                q0 = self.measured[name]
                direction = 1.0
                if q0 + self.args.step > high - 0.02:
                    direction = -1.0
                if q0 + direction * self.args.step < low + 0.02:
                    print(f"\n{name} 离限位太近（{q0:.3f}），跳过")
                    continue
                target = q0 + direction * self.args.step
                print(f"\n=== J{joint + 1} {name}: {q0:.4f} → {target:.4f} rad")
                self.publish_command(joint, target, kp[joint], kd[joint])
                times, values = self.record(self.args.settle)
                self.publish_command(joint, q0, kp[joint], kd[joint])
                self.spin(0.8)
                deviation = max(abs(v - q0) for v in values if not math.isnan(v)) if values else 0.0
                if deviation > MAX_DEVIATION_RAD:
                    print(f"跟随偏差 {deviation:.3f} rad 超过上限，停止测试")
                    break
                result = self.analyze(joint, times, values, target, out_dir)
                if result:
                    results.append(result)
                    print(f"    ζ={result['zeta']:.2f}  ω_n={result['wn']:.2f} rad/s  "
                          f"J_eff={result['j_eff']:.3f} kg·m²")
        finally:
            ok, detail = self.trigger(self.exit_client, "/x1/current_gravity_exit")
            print(f"\n退出 MIT：{'成功' if ok else '失败 ' + detail}")
        if results:
            self.report(results, out_dir)
        return 0

    def publish_command(self, joint: int, q_des: float, kp: float, kd: float) -> None:
        from std_msgs.msg import Float64MultiArray
        message = Float64MultiArray()
        message.data = [float(joint), float(q_des), 0.0, float(kp), float(kd), 0.0]
        self.mit_command_pub.publish(message)

    def analyze(self, joint: int, times: List[float], values: List[float],
                target: float, out_dir: Path) -> Optional[dict]:
        if len(times) < 20:
            print("   采样太少，跳过")
            return None
        name = RIGHT_JOINTS[joint]
        csv = out_dir / f"j{joint + 1}.csv"
        with csv.open("w", encoding="utf-8") as handle:
            handle.write("t,q\n")
            for stamp, value in zip(times, values):
                handle.write(f"{stamp - times[0]:.6f},{value:.6f}\n")
        # 终值取最后 25% 的均值，然后整体减去它，只拟合瞬态部分。
        tail = values[int(len(values) * 0.75):]
        q_end = sum(tail) / len(tail)
        y = [value - q_end for value in values]
        fit = fit_second_order(times, y)
        if not fit:
            print("   拟合失败")
            return None
        k_eff = self.args.kp / 1000.0 * RATED_TORQUE[joint]
        j_eff = k_eff / (fit["wn"] ** 2) if fit["wn"] > 0 else float("nan")
        zeta_urdf = ((self.args.kd / 1000.0 * RATED_TORQUE[joint])
                     / (2 * math.sqrt(k_eff * URDF_INERTIA[joint])))
        # ζ 目标值 0.7 需要的 kd。
        kd_need = (0.7 * 2 * math.sqrt(k_eff * j_eff)) / RATED_TORQUE[joint] * 1000.0
        return {
            "joint": joint + 1, "name": name, "zeta": fit["zeta"], "wn": fit["wn"],
            "rms": fit["rms"], "j_eff": j_eff, "zeta_urdf": zeta_urdf,
            "kd_for_zeta_0_7": kd_need, "q_start": values[0], "q_end": q_end,
            "sag": q_end - target, "csv": str(csv),
        }

    def report(self, results: List[dict], out_dir: Path) -> None:
        print("\n关节   ζ实测  ω_n(rad/s)  J_eff    ζ理论(URDF)  kd@ζ0.7  静态残差")
        for item in results:
            print(f"  r{item['joint']:<2} {item['zeta']:>5.2f} {item['wn']:>9.2f} "
                  f"{item['j_eff']:>7.3f} {item['zeta_urdf']:>11.2f} "
                  f"{item['kd_for_zeta_0_7']:>8.0f} {item['sag']:>9.4f}")
        path = out_dir / "damping.json"
        path.write_text(json.dumps({
            "kp": self.args.kp, "kd": self.args.kd, "step": self.args.step,
            "joints": results}, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n结果写入 {path}")
        print("说明：ζ 是在当前 kp/kd 下测到的。kd@ζ0.7 用实测 J_eff 反推，"
              "可以直接写回键盘的 MIT_KD。")


def selftest() -> int:
    """合成一组已知 (ω, ζ) 的欠阻尼响应，检查拟合器能不能还原。"""
    cases = [(3.0, 0.10), (5.0, 0.30), (8.0, 0.70), (12.0, 0.45), (20.0, 0.80)]
    worst = 0.0
    for wn, zeta in cases:
        times, values = [], []
        wd = wn * math.sqrt(1.0 - zeta * zeta)
        for index in range(200):
            t = index * 0.01
            times.append(t)
            values.append(-0.05 * math.exp(-zeta * wn * t) * math.cos(wd * t))
        fit = fit_second_order(times, values)
        if fit is None:
            print(f"ω={wn:>5.1f} ζ={zeta:.2f}: 拟合失败")
            return 1
        worst = max(worst, abs(fit["wn"] - wn) / wn, abs(fit["zeta"] - zeta))
        print(f"ω={wn:>5.1f} ζ={zeta:.2f} → ω̂={fit['wn']:>6.2f} ζ̂={fit['zeta']:>5.2f} rms={fit['rms']:.5f}")
    print(f"最大误差 {worst * 100:.1f}%")
    return 0 if worst < 0.06 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--home", action="store_true", help="先回机械零位")
    parser.add_argument("--step", type=float, default=0.05, help="阶跃幅度（rad，上限 0.06）")
    parser.add_argument("--settle", type=float, default=2.0, help="每次阶跃记录时长（秒）")
    parser.add_argument("--kp", type=float, default=600.0, help="测试用 kp（额定千分比）")
    parser.add_argument("--kd", type=float, default=60.0, help="测试用 kd；给小一点振荡更清楚")
    parser.add_argument("--joints", type=lambda text: [int(v) - 1 for v in text.split(",")],
                        default=list(range(7)), help="只测这些关节，例如 4,6")
    parser.add_argument("--out", default=f"log/mit_damping_{time.strftime('%Y%m%d_%H%M%S')}")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--selftest", action="store_true",
                        help="用合成曲线检查拟合器，不需要硬件")
    args = parser.parse_args()
    if args.step > MAX_STEP_RAD or args.step <= 0:
        print(f"--step 必须在 (0, {MAX_STEP_RAD}] 之间")
        return 2
    if any(joint < 0 or joint > 6 for joint in args.joints):
        print("--joints 只能是 1~7")
        return 2
    if args.selftest:
        return selftest()
    if args.dry_run:
        # 干跑不碰 ROS，没 source 环境也能看流程。
        print(f"干跑：joints={[j + 1 for j in args.joints]} kp={args.kp} kd={args.kd} "
              f"step={args.step} settle={args.settle} home={args.home} out={args.out}")
        print("阶跃幅度上限 " f"{MAX_STEP_RAD} rad；偏差上限 {MAX_DEVIATION_RAD} rad")
        return 0

    import rclpy
    rclpy.init()
    identifier = None
    try:
        identifier = Identifier(args)
        return identifier.run()
    finally:
        if identifier is not None:
            identifier.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
