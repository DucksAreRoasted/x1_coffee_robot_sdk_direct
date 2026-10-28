#!/usr/bin/env python3
"""Walk the 15 joints and record which motor directions match the URDF.

The operator jogs one joint at a time. Each jog records the raw encoder
before and after. The operator then says whether the real gripper moved the
way the URDF predicted. A reversed joint is marked so its pulses_per_unit
sign can be flipped. The running control process keeps the old sign until
it is restarted.
"""

import os
import re
import select
import sys
import termios
import time
import tty
from typing import Dict, List, Optional, Tuple

STEP_RAD = 0.05
DURATION_SEC = 0.8
SCALE_RE = re.compile(
    r'(<param name="pulses_per_unit\.(Joint_(?:Body|[lr][1-7]))">)(-?\d+(?:\.\d+)?)(</param>)'
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keyboard_tcp_teleop as teleop


def predicted_deltas(urdf_path: str, measured: Dict[str, float], joint: str, step: float = STEP_RAD):
    goal_angle = measured.get(joint, 0.0) + step
    sides = []
    if joint == "Joint_Body" or joint in teleop.LEFT_JOINTS:
        sides.append(("left", "left_gripper_tcp", teleop.LEFT_JOINTS))
    if joint == "Joint_Body" or joint in teleop.RIGHT_JOINTS:
        sides.append(("right", "right_gripper_tcp", teleop.RIGHT_JOINTS))
    deltas = []
    for side, tip, joints in sides:
        chain = teleop.load_chain(urdf_path, tip, joints)
        seed = {name: measured.get(name, 0.0) for name in [*joints, "Joint_Body"]}
        start = teleop._origin_of(chain.fk(seed))
        moved = dict(seed)
        moved[joint] = goal_angle
        end = teleop._origin_of(chain.fk(moved))
        deltas.append((side, [end[axis] - start[axis] for axis in range(3)]))
    return deltas


def describe_delta(delta: List[float]) -> str:
    axis = max(range(3), key=lambda index: abs(delta[index]))
    if abs(delta[axis]) < 0.001:
        return "预测位移很小"
    name = ("全局 X", "全局 Y", "全局 Z")[axis]
    if axis == 2:
        sense = "升高" if delta[axis] > 0 else "降低"
    else:
        sense = "增大" if delta[axis] > 0 else "减小"
    return f"{name}{sense} {delta[axis]:+.4f} m"


def encoder_followed(command_rad: float, encoder_delta: int, scale: float) -> bool:
    expected = command_rad * scale
    if abs(expected) < 1.0 or abs(encoder_delta) < abs(expected) * 0.25:
        return False
    return (encoder_delta > 0) == (expected > 0)


def negate_literal(literal: str) -> str:
    if literal.startswith("-"):
        return literal[1:]
    return "-" + literal


def apply_verdicts(text: str, verdicts: Dict[str, str]) -> str:
    def replace(match):
        name = match.group(2)
        literal = match.group(3)
        if verdicts.get(name) == "reversed":
            literal = negate_literal(literal)
        return f"{match.group(1)}{literal}{match.group(4)}"

    return SCALE_RE.sub(replace, text)


def xacro_paths() -> List[str]:
    found = []
    path = os.path.abspath(__file__)
    for _ in range(6):
        path = os.path.dirname(path)
        candidates = [
            os.path.join(path, "src/x1_coffee_ros2_control/urdf/x1_coffee.ros2_control.xacro"),
            os.path.join(
                path,
                "install/x1_coffee_ros2_control/share/x1_coffee_ros2_control/urdf/x1_coffee.ros2_control.xacro",
            ),
        ]
        existing = [item for item in candidates if os.path.isfile(item)]
        if existing:
            return existing
    return found


def record_path() -> str:
    paths = xacro_paths()
    if paths:
        root = os.path.dirname(paths[0])
        for _ in range(6):
            if os.path.isdir(os.path.join(root, "src")) and os.path.isdir(os.path.join(root, "install")):
                return os.path.join(root, "direction_alignment.yaml")
            root = os.path.dirname(root)
    return os.path.join(os.getcwd(), "direction_alignment.yaml")


def load_scales(paths: List[str]) -> Dict[str, float]:
    scales = {name: 83443.02680376362 for name in teleop.COFFEE_JOINTS}
    if not paths:
        return scales
    text = open(paths[0], encoding="utf-8").read()
    for match in SCALE_RE.finditer(text):
        scales[match.group(2)] = float(match.group(3))
    return scales


def render_record(verdicts: Dict[str, str], samples: Dict[str, List[dict]]) -> str:
    lines = [
        "# 正指令和 URDF 预测相反的关节记为 reversed。",
        "# 编码器增量只证明电机跟着这条指令转了，方向是否符合 URDF 由操作者确认。",
        "joints:",
    ]
    for name in teleop.COFFEE_JOINTS:
        if name not in verdicts and name not in samples:
            continue
        lines.append(f"  {name}:")
        lines.append(f"    verdict: {verdicts.get(name, 'unmarked')}")
        lines.append("    samples:")
        for sample in samples.get(name, []):
            lines.append(f"      - command_rad: {sample['command_rad']:.6f}")
            lines.append(f"        angle_before: {sample['angle_before']:.6f}")
            lines.append(f"        angle_after: {sample['angle_after']:.6f}")
            lines.append(f"        encoder_before: {sample['encoder_before']}")
            lines.append(f"        encoder_after: {sample['encoder_after']}")
            lines.append(f"        encoder_delta: {sample['encoder_delta']}")
            lines.append(f"        encoder_followed: {str(sample['encoder_followed']).lower()}")
            lines.append(f"        status_before: {sample['status_before']}")
            lines.append(f"        status_after: {sample['status_after']}")
            for side, delta in sample["predicted"]:
                formatted = ", ".join(f"{value:.6f}" for value in delta)
                lines.append(f"        predicted_{side}_m: [{formatted}]")
    return "\n".join(lines) + "\n"


def write_scales(verdicts: Dict[str, str]) -> List[str]:
    written = []
    for path in xacro_paths():
        original = open(path, encoding="utf-8").read()
        updated = apply_verdicts(original, verdicts)
        if updated != original:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(updated)
            written.append(path)
    return written


def _self_check() -> int:
    urdf = sys.argv[2] if len(sys.argv) > 2 else teleop.default_urdf()
    measured = {name: 0.3 for name in teleop.COFFEE_JOINTS}
    deltas = predicted_deltas(urdf, measured, "Joint_l2")
    if not deltas or describe_delta(deltas[0][1]) == "":
        print("self-check failed")
        return 1
    if not encoder_followed(0.05, 4000, 83443.0):
        print("self-check failed")
        return 1
    if encoder_followed(0.05, -4000, 83443.0):
        print("self-check failed")
        return 1
    sample = (
        '<param name="pulses_per_unit.Joint_l2">83443.02680376362</param>\n'
        '<param name="pulses_per_unit.Joint_l3">83443.02680376362</param>\n'
    )
    updated = apply_verdicts(sample, {"Joint_l2": "reversed", "Joint_l3": "matched"})
    if "-83443.02680376362" not in updated or updated.count("83443.02680376362") != 2:
        print(updated)
        print("self-check failed")
        return 1
    print("align joint direction self-check passed")
    return 0


def main() -> int:
    if "--self-check" in sys.argv:
        return _self_check()
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        return _predict_one(sys.argv[1], "--move" in sys.argv)
    return _session()


def _predict_one(joint: str, do_move: bool) -> int:
    if joint not in teleop.COFFEE_JOINTS:
        print(f"未知关节 {joint}")
        return 2
    print("逐轴记录请直接运行本程序，不要加关节名。")
    print(f"单关节预测仍可用。{'会发送运动。' if do_move else '这次不运动。'}")
    return _session_one_shot(joint, do_move)


def _session_one_shot(joint: str, do_move: bool) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    rclpy.init()
    node = Node("align_joint_direction")
    measured: Dict[str, float] = {}

    def on_joints(message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            measured[name] = float(position)

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    publisher = node.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(name in measured for name in teleop.COFFEE_JOINTS):
            break
    if any(name not in measured for name in teleop.COFFEE_JOINTS):
        print("没有收齐 /joint_states")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    lower, upper = teleop.LIMITS[joint]
    target = min(upper, max(lower, measured[joint] + STEP_RAD))
    deltas = predicted_deltas(teleop.default_urdf(), measured, joint)
    print(f"{joint}  {measured[joint]:.4f} -> {target:.4f} rad")
    for side, delta in deltas:
        print(f"  {side}  {describe_delta(delta)}")
    if do_move and abs(target - measured[joint]) > 1e-6:
        message = Float64MultiArray()
        positions = [measured[name] for name in teleop.COFFEE_JOINTS]
        positions[teleop.COFFEE_JOINTS.index(joint)] = target
        message.data = [1.0, DURATION_SEC, 1.0, *positions]
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.2)
        print(f"已发送，用时 {DURATION_SEC:.1f} s。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def _session() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
    from x1_coffee_interfaces.msg import MotorStateArray

    scales = load_scales(xacro_paths())
    rclpy.init()
    node = Node("align_joint_direction")
    measured: Dict[str, float] = {}
    motors: Dict[str, object] = {}
    pvt_result: Dict[str, Optional[str]] = {"data": None}
    sequence = {"value": 0.0}

    def on_joints(message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            measured[name] = float(position)

    def on_motors(message: MotorStateArray) -> None:
        for state in message.states:
            motors[state.joint_name] = state

    def on_result(message: String) -> None:
        pvt_result["data"] = message.data

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    node.create_subscription(MotorStateArray, "/x1/motor_states", on_motors, qos_profile_sensor_data)
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
    if not spin_until(lambda: all(name in motors for name in teleop.COFFEE_JOINTS), 10.0):
        print("没有收齐 /x1/motor_states，不能记录编码器。")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    if not spin_until(lambda: publisher.get_subscription_count() > 0, 10.0):
        print("没有 /x1/pvt_move。请先启动控制栈。")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    verdicts: Dict[str, str] = {}
    samples: Dict[str, List[dict]] = {}
    urdf = teleop.default_urdf()
    print("急停放在手边。先退出键盘程序，避免两处同时发运动。")
    print("空格：当前关节正方向 0.05 rad，并记录编码器。")
    print("-：反方向 0.05 rad。 y：和预测同向。 n：和预测相反。 s：跳过。 q：保存退出。")
    print("关节角反馈看不出反了。编码器增量只说明电机转了，方向以你看到的末端为准。")

    def send(joint: str, step: float) -> Optional[dict]:
        if joint not in measured or joint not in motors:
            print("还没有这一轴的电机状态。")
            return None
        lower, upper = teleop.LIMITS[joint]
        target = min(upper, max(lower, measured[joint] + step))
        if abs(target - measured[joint]) < 1e-6:
            print("这一方向已经到限位。")
            return None
        before_angle = measured[joint]
        before_encoder = int(motors[joint].encoder_pulse_raw)
        before_status = int(motors[joint].status_bits)
        prediction = predicted_deltas(urdf, measured, joint, step)
        positions = [measured[name] for name in teleop.COFFEE_JOINTS]
        positions[teleop.COFFEE_JOINTS.index(joint)] = target
        sequence["value"] += 1.0
        token = sequence["value"]
        pvt_result["data"] = None
        message = Float64MultiArray()
        message.data = [token, DURATION_SEC, 1.0, *positions]
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
            return None
        parts = pvt_result["data"].split(" ", 2)
        if len(parts) < 2 or parts[1] != "ok":
            detail = parts[2] if len(parts) > 2 else pvt_result["data"]
            print(f"PVT 失败: {detail}")
            return None
        spin_until(lambda: False, DURATION_SEC + 0.4)
        after = motors.get(joint)
        if after is None:
            print("运动后没有电机状态。")
            return None
        sample = {
            "command_rad": target - before_angle,
            "angle_before": before_angle,
            "angle_after": measured.get(joint, before_angle),
            "encoder_before": before_encoder,
            "encoder_after": int(after.encoder_pulse_raw),
            "encoder_delta": int(after.encoder_pulse_raw) - before_encoder,
            "status_before": before_status,
            "status_after": int(after.status_bits),
            "predicted": prediction,
        }
        sample["encoder_followed"] = encoder_followed(
            sample["command_rad"], sample["encoder_delta"], scales.get(joint, 83443.02680376362)
        )
        samples.setdefault(joint, []).append(sample)
        print(
            f"编码器 {sample['encoder_before']} -> {sample['encoder_after']}"
            f"  增量 {sample['encoder_delta']:+d}"
            f"  关节角 {sample['angle_before']:.4f} -> {sample['angle_after']:.4f}"
        )
        if not sample["encoder_followed"]:
            print("编码器没有按当前比例跟着这条指令走，先不要对这一下按 y 或 n。")
        for side, delta in prediction:
            print(f"  预测 {side} {describe_delta(delta)}")
        return sample

    terminal = termios.tcgetattr(sys.stdin)
    index = 0
    shown = -1
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and index < len(teleop.COFFEE_JOINTS):
            joint = teleop.COFFEE_JOINTS[index]
            rclpy.spin_once(node, timeout_sec=0.05)
            if shown != index:
                deltas = predicted_deltas(urdf, measured, joint, STEP_RAD)
                print(f"\n[{index + 1}/15] {joint}  当前 {measured.get(joint, 0.0):.4f} rad"
                      f"  编码器 {int(motors[joint].encoder_pulse_raw)}")
                for side, delta in deltas:
                    print(f"  正方向预测 {side} {describe_delta(delta)}")
                shown = index
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == " ":
                send(joint, STEP_RAD)
                shown = -1
            elif key == "-":
                send(joint, -STEP_RAD)
                shown = -1
            elif key in ("y", "n"):
                if not samples.get(joint):
                    print("先按空格转一下，再判断。")
                    continue
                if not any(item["command_rad"] > 0.0 and item["encoder_followed"] for item in samples[joint]):
                    print("先按空格做一次正方向，并确认编码器跟着走了，再判断。")
                    continue
                verdicts[joint] = "matched" if key == "y" else "reversed"
                print(f"{joint} 记为 {verdicts[joint]}")
                index += 1
            elif key == "s":
                verdicts[joint] = "skipped"
                index += 1
            elif key == "q":
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        path = record_path()
        text = render_record(verdicts, samples)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)
        print(f"\n记录已写入 {path}")
        reversed_names = [name for name, verdict in verdicts.items() if verdict == "reversed"]
        if reversed_names:
            print("相反的轴: " + ", ".join(reversed_names))
            print("输入 apply 才把这些轴的 pulses_per_unit 改号。直接回车不改文件。")
            answer = input().strip()
            if answer == "apply":
                written = write_scales(verdicts)
                if written:
                    print("已修改:")
                    for item in written:
                        print(f"  {item}")
                    print("停掉控制栈再启动后，新符号才会进电机。零位脉冲没有改。")
                else:
                    print("没有找到可写的 xacro，或这些轴已经是目标符号。")
        else:
            print("没有标成相反的轴，xacro 未改。")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
