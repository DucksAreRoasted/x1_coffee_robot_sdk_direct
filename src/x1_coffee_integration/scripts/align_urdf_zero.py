#!/usr/bin/env python3
"""Jog from the mechanical zero until the arm matches the URDF zero pose.

The operator marks the mechanical pose, moves one joint at a time, then marks
the pose that matches the URDF picture. The error of each joint is the angle
reading at that picture. The model zero becomes the old model zero minus that
reading, so the matched pose is reported as 0 rad. Factory encoder pulses are
left unchanged.
"""

import os
import re
import select
import sys
import termios
import time
import tty
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keyboard_tcp_teleop as teleop

MAX_JOINT_VELOCITY = 0.4
MIN_SEGMENT_SEC = 0.8
JOINT_LINE = re.compile(r"^(  )(Joint_(?:Body|[lr][1-7])):.*$", re.M)


def segment_seconds(distance: float) -> float:
    if distance <= 1.0e-9:
        return MIN_SEGMENT_SEC
    return max(MIN_SEGMENT_SEC, distance / MAX_JOINT_VELOCITY)


def model_zero_for_match(mechanical_angle: float, matched_angle: float) -> float:
    """URDF angle of the factory pulse after the matched pose is defined as 0.

    Both angles are the controller's live readings. The previous value in the
    yaml is not a baseline, because that file may not be what the process loaded.
    """
    return mechanical_angle - matched_angle


def write_model_zeros(text: str, zeros: Dict[str, float]) -> str:
    def replace(match):
        name = match.group(2)
        if name not in zeros:
            return match.group(0)
        return f"{match.group(1)}{name}: {zeros[name]:.6f}"

    return JOINT_LINE.sub(replace, text)


def sdk_root() -> str:
    path = os.path.abspath(__file__)
    for _ in range(6):
        path = os.path.dirname(path)
        src = os.path.join(path, "src/x1_coffee_integration/config/initial_positions.yaml")
        install = os.path.join(
            path,
            "install/x1_coffee_integration/share/x1_coffee_integration/config/initial_positions.yaml",
        )
        if os.path.isfile(src) or os.path.isfile(install):
            return path
    return os.getcwd()


def position_files() -> List[str]:
    root = sdk_root()
    candidates = [
        os.path.join(root, "src/x1_coffee_integration/config/initial_positions.yaml"),
        os.path.join(
            root,
            "install/x1_coffee_integration/share/x1_coffee_integration/config/initial_positions.yaml",
        ),
    ]
    return [item for item in candidates if os.path.isfile(item)]


def render_result(mechanical: Dict[str, float], matched: Dict[str, float]) -> str:
    lines = [
        "# 误差 = 摆成 URDF 零位时的关节读数。",
        "# 新 model_zero = 机械零位读数 - URDF 图读数。不用文件里的旧角度。",
        "joints:",
    ]
    for name in teleop.COFFEE_JOINTS:
        error = matched[name]
        travel = matched[name] - mechanical[name]
        updated = model_zero_for_match(mechanical[name], error)
        lines.append(f"  {name}:")
        lines.append(f"    mechanical_rad: {mechanical[name]:.6f}")
        lines.append(f"    urdf_match_rad: {matched[name]:.6f}")
        lines.append(f"    travel_rad: {travel:.6f}")
        lines.append(f"    error_rad: {error:.6f}")
        lines.append(f"    model_zero: {updated:.6f}")
    return "\n".join(lines) + "\n"


def _self_check() -> int:
    if abs(model_zero_for_match(0.0, 0.280) + 0.280) > 1e-9:
        print("self-check failed")
        return 1
    if abs(model_zero_for_match(-0.280, 0.0) + 0.280) > 1e-9:
        print("self-check failed")
        return 1
    sample = "initial_positions:\n  Joint_Body: -0.280\n  Joint_l1: 0.0\n"
    updated = write_model_zeros(sample, {"Joint_Body": -0.1, "Joint_l1": 0.25})
    if "Joint_Body: -0.100000" not in updated or "Joint_l1: 0.250000" not in updated:
        print(updated)
        print("self-check failed")
        return 1
    if abs(segment_seconds(0.8) - 2.0) > 1e-9:
        print("self-check failed")
        return 1
    print("align urdf zero self-check passed")
    return 0


def main() -> int:
    if "--self-check" in sys.argv:
        return _self_check()
    return _session()


def _session() -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String

    files = position_files()
    rclpy.init()
    node = Node("align_urdf_zero")
    measured: Dict[str, float] = {}
    pvt_result: Dict[str, Optional[str]] = {"data": None}
    sequence = {"value": 0.0}

    def on_joints(message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            measured[name] = float(position)

    def on_result(message: String) -> None:
        pvt_result["data"] = message.data

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    node.create_subscription(String, "/x1/pvt_result", on_result, 10)
    publisher = node.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)

    def spin_until(predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    if not spin_until(lambda: all(name in measured for name in teleop.COFFEE_JOINTS), 10.0):
        print("没有收齐 /joint_states")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if not spin_until(lambda: publisher.get_subscription_count() > 0, 10.0):
        print("没有 /x1/pvt_move。请先启动控制栈。")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    mechanical: Dict[str, float] = {}
    matched: Dict[str, float] = {}
    index = 0
    step = 0.02
    shown = -1
    return_until = 0.0

    def publish(positions: List[float], duration: float) -> bool:
        sequence["value"] += 1.0
        token = sequence["value"]
        pvt_result["data"] = None
        message = Float64MultiArray()
        message.data = [token, duration, 1.0, *positions]
        publisher.publish(message)

        def accepted() -> bool:
            data = pvt_result["data"]
            if not data:
                return False
            parts = data.split(" ", 2)
            try:
                return abs(float(parts[0]) - token) < 0.1
            except (ValueError, IndexError):
                return False

        if not spin_until(accepted, 5.0):
            print("PVT 没有回应。")
            return False
        parts = (pvt_result["data"] or "").split(" ", 2)
        if len(parts) < 2 or parts[1] != "ok":
            detail = parts[2] if len(parts) > 2 else pvt_result["data"]
            print(f"PVT 失败: {detail}")
            return False
        spin_until(lambda: False, duration + 0.3)
        return True

    def jog(joint: str, delta: float) -> None:
        lower, upper = teleop.LIMITS[joint]
        target = min(upper, max(lower, measured[joint] + delta))
        if abs(target - measured[joint]) < 1e-6:
            print("这一方向已经到软件限位。")
            return
        positions = [measured[name] for name in teleop.COFFEE_JOINTS]
        positions[teleop.COFFEE_JOINTS.index(joint)] = target
        duration = segment_seconds(abs(target - measured[joint]))
        if publish(positions, duration):
            print(f"{joint} -> {target:.4f} rad，用时 {duration:.1f} s")

    def show() -> None:
        joint = teleop.COFFEE_JOINTS[index]
        current = measured.get(joint, 0.0)
        if joint in mechanical:
            travel = current - mechanical[joint]
            origin = f"机械 {mechanical[joint]:+.4f}  行程 {travel:+.4f}"
        else:
            origin = "机械零点尚未记录"
        print(f"\n[{index + 1}/15] {joint}  当前 {current:+.4f} rad  {origin}")
        print(f"步长 {step:.2f} rad。n/p 换关节，+/- 点动，1/2/3 改步长。")

    print("急停放在手边。先退出键盘程序。")
    print("RViz 那幅全零姿势就是 URDF 零位：夹爪水平朝前，不是自然下垂。")
    print("先把真机停在机械零点，按 m 记录。再逐个关节摆到和那幅图一样，按 u 计算误差。")
    print("+/- 点动，1=0.01 rad，2=0.05 rad，3=0.10 rad。n/p 换关节。")
    print("r r 连按两次回到刚记录的机械零点。q 退出。")

    terminal = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if shown != index:
                show()
                shown = index
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key in ("n", "j"):
                index = (index + 1) % len(teleop.COFFEE_JOINTS)
            elif key in ("p", "k"):
                index = (index - 1) % len(teleop.COFFEE_JOINTS)
            elif key in ("+", "="):
                jog(teleop.COFFEE_JOINTS[index], step)
                shown = -1
            elif key == "-":
                jog(teleop.COFFEE_JOINTS[index], -step)
                shown = -1
            elif key == "1":
                step = 0.01
                shown = -1
            elif key == "2":
                step = 0.05
                shown = -1
            elif key == "3":
                step = 0.10
                shown = -1
            elif key == "m":
                mechanical = {name: measured[name] for name in teleop.COFFEE_JOINTS}
                print("已记录机械零点。")
                for name in teleop.COFFEE_JOINTS:
                    print(f"  {name}  {mechanical[name]:+.4f}")
                shown = -1
            elif key == "u":
                if not mechanical:
                    print("先在机械零点按 m。")
                    continue
                matched = {name: measured[name] for name in teleop.COFFEE_JOINTS}
                print("\nURDF 零位误差（摆成图中姿势时的读数，单位 rad）：")
                for name in teleop.COFFEE_JOINTS:
                    error = matched[name]
                    travel = matched[name] - mechanical[name]
                    updated = model_zero_for_match(mechanical[name], error)
                    print(
                        f"  {name:12}  误差 {error:+.4f}  行程 {travel:+.4f}"
                        f"  model_zero {mechanical[name]:+.4f} -> {updated:+.4f}"
                    )
                shown = -1
            elif key == "r":
                now = time.monotonic()
                if not mechanical:
                    print("还没有机械零点记录。")
                    continue
                if now > return_until:
                    return_until = now + 3.0
                    print("再按一次 r：回到机械零点。请让开手臂。")
                    continue
                return_until = 0.0
                positions = []
                farthest = 0.0
                for name in teleop.COFFEE_JOINTS:
                    lower, upper = teleop.LIMITS[name]
                    target = min(upper, max(lower, mechanical[name]))
                    farthest = max(farthest, abs(target - measured[name]))
                    positions.append(target)
                duration = segment_seconds(farthest)
                if publish(positions, duration):
                    print(f"正在回到机械零点，最大行程 {farthest:.3f} rad，约 {duration:.1f} s")
                shown = -1
            elif key == "q":
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        path = os.path.join(sdk_root(), "urdf_zero_alignment.yaml")
        if mechanical and matched:
            text = render_result(mechanical, matched)
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(text)
            print(f"\n误差已写入 {path}")
            print("输入 apply 才把新的 model_zero 写入配置。直接回车不改文件。")
            answer = input().strip()
            if answer == "apply":
                written = []
                corrected = {
                    name: model_zero_for_match(mechanical[name], matched[name])
                    for name in teleop.COFFEE_JOINTS
                }
                for item in position_files():
                    original = open(item, encoding="utf-8").read()
                    updated = write_model_zeros(original, corrected)
                    if updated != original:
                        with open(item, "w", encoding="utf-8") as handle:
                            handle.write(updated)
                        written.append(item)
                if written:
                    print("已写入:")
                    for item in written:
                        print(f"  {item}")
                    print("停掉控制栈再启动后生效。出厂零位脉冲没有改。")
                else:
                    print("没有需要修改的配置。")
        else:
            print("\n还没有同时记录机械零点和 URDF 零位，未计算误差。")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
