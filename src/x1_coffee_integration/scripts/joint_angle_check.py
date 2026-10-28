#!/usr/bin/env python3
"""Move one joint to 45° or 90° from a marked pose.

Other joints return to that pose, so the picture shows a single hinge. The
script never enables drives and never calls /x1/start_motion.
"""

import math
import os
import select
import sys
import termios
import time
import tty
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keyboard_tcp_teleop as teleop

MAX_JOINT_VELOCITY = 0.4
MIN_SEGMENT_SEC = 0.8
CONFIRM_SEC = 3.0
LIMIT_SLACK = 1.0e-3
DEG45 = math.pi / 4.0
DEG90 = math.pi / 2.0

LABELS = {
    "Joint_Body": "腰",
    "Joint_l1": "左 1",
    "Joint_l2": "左 2",
    "Joint_l3": "左 3",
    "Joint_l4": "左 4",
    "Joint_l5": "左 5",
    "Joint_l6": "左 6",
    "Joint_l7": "左 7",
    "Joint_r1": "右 1",
    "Joint_r2": "右 2",
    "Joint_r3": "右 3",
    "Joint_r4": "右 4",
    "Joint_r5": "右 5",
    "Joint_r6": "右 6",
    "Joint_r7": "右 7",
}


def segment_seconds(distance: float) -> float:
    if distance <= 1.0e-9:
        return MIN_SEGMENT_SEC
    return max(MIN_SEGMENT_SEC, distance / MAX_JOINT_VELOCITY)


def plan_offset(
    reference: Dict[str, float],
    measured: Dict[str, float],
    joint: str,
    delta_rad: float,
    limits: Dict[str, Tuple[float, float]] = teleop.LIMITS,
    names: Sequence[str] = teleop.COFFEE_JOINTS,
) -> Optional[Tuple[List[float], float]]:
    """Return joint targets and the longest travel, or None if this joint cannot reach."""
    lower, upper = limits[joint]
    goal = reference[joint] + delta_rad
    if goal < lower - LIMIT_SLACK or goal > upper + LIMIT_SLACK:
        return None
    goal = min(upper, max(lower, goal))
    positions: List[float] = []
    farthest = 0.0
    for name in names:
        target = goal if name == joint else reference[name]
        low, high = limits[name]
        target = min(high, max(low, target))
        current = measured.get(name, reference[name])
        farthest = max(farthest, abs(target - current))
        positions.append(target)
    return positions, farthest


def reachable_span(
    reference_angle: float,
    limits: Tuple[float, float],
) -> Tuple[float, float]:
    lower, upper = limits
    return upper - reference_angle, reference_angle - lower


def _self_check() -> int:
    reference = {name: 0.0 for name in teleop.COFFEE_JOINTS}
    measured = dict(reference)
    plus = plan_offset(reference, measured, "Joint_l1", DEG90)
    if plus is None or abs(plus[0][teleop.COFFEE_JOINTS.index("Joint_l1")] - DEG90) > 1e-9:
        print("self-check failed: +90")
        return 1
    if any(abs(value) > 1e-9 for index, value in enumerate(plus[0]) if teleop.COFFEE_JOINTS[index] != "Joint_l1"):
        print("self-check failed: other joints moved")
        return 1
    if abs(segment_seconds(DEG90) - DEG90 / MAX_JOINT_VELOCITY) > 1e-9:
        print("self-check failed: duration")
        return 1
    if plan_offset(reference, measured, "Joint_l2", -DEG45) is not None:
        print("self-check failed: l2 -45 should be blocked")
        return 1
    if plan_offset(reference, measured, "Joint_l2", DEG45) is None:
        print("self-check failed: l2 +45")
        return 1
    if plan_offset(reference, measured, "Joint_r2", DEG45) is not None:
        print("self-check failed: r2 +45 should be blocked")
        return 1
    if plan_offset(reference, measured, "Joint_r4", -DEG90) is None:
        print("self-check failed: r4 -90")
        return 1
    print("joint angle check self-check passed")
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

    rclpy.init()
    node = Node("joint_angle_check")
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

    reference = {name: measured[name] for name in teleop.COFFEE_JOINTS}
    index = 0
    shown = -1
    pending: Dict[str, float] = {"key": "", "until": 0.0}

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

    def show() -> None:
        joint = teleop.COFFEE_JOINTS[index]
        plus, minus = reachable_span(reference[joint], teleop.LIMITS[joint])
        print(
            f"\n[{index + 1}/15] {LABELS[joint]}  {joint}"
            f"  基准 {math.degrees(reference[joint]):+.1f}°"
            f"  正向最多 {math.degrees(plus):.0f}°"
            f"  反向最多 {math.degrees(minus):.0f}°"
        )
        print("4 +45°    5 -45°    9 +90°    8 -90°    都要在 3 秒内再按一次才动")
        print("0 0 回到基准    b 把当前姿势记为基准    n/p 换关节    q 退出")

    def move(delta_rad: float, label: str) -> None:
        joint = teleop.COFFEE_JOINTS[index]
        planned = plan_offset(reference, measured, joint, delta_rad)
        if planned is None:
            lower, upper = teleop.LIMITS[joint]
            print(
                f"{LABELS[joint]} 到不了 {label}。"
                f"限位是 {math.degrees(lower):.0f}° 到 {math.degrees(upper):.0f}°。"
            )
            return
        positions, farthest = planned
        duration = segment_seconds(farthest)
        print(f"{LABELS[joint]} 转到基准 {label}。其他关节回到基准。约 {duration:.1f} s。请让开手臂。")
        if publish(positions, duration):
            print(f"到位。看 RViz 和真机是不是都只这一节差了 {label}。")

    print("急停放在手边。先退出键盘程序。")
    print("基准已取当前姿势。要和 URDF 零位比的话，先用键盘连按 0 0，再开这个程序。")
    print("每次只让当前关节离开基准 45° 或 90°，其余关节回到基准。")

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
                pending["until"] = 0.0
            elif key in ("p", "k"):
                index = (index - 1) % len(teleop.COFFEE_JOINTS)
                pending["until"] = 0.0
            elif key == "b":
                if not all(name in measured for name in teleop.COFFEE_JOINTS):
                    print("还没有收齐关节角。")
                    continue
                reference = {name: measured[name] for name in teleop.COFFEE_JOINTS}
                print("已把当前 15 个关节记为基准。")
                shown = -1
            elif key in ("4", "5", "8", "9", "0"):
                now = time.monotonic()
                if pending["key"] != key or now > pending["until"]:
                    pending["key"] = key
                    pending["until"] = now + CONFIRM_SEC
                    print("再按一次才动。请让开手臂。")
                    continue
                pending["until"] = 0.0
                if key == "4":
                    move(DEG45, "+45°")
                elif key == "5":
                    move(-DEG45, "-45°")
                elif key == "9":
                    move(DEG90, "+90°")
                elif key == "8":
                    move(-DEG90, "-90°")
                else:
                    move(0.0, "0°")
                shown = -1
            elif key == "q":
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
