#!/usr/bin/env python3
"""Compute encoder zero offsets from a hardware limit that is already active.

This does not enable drives and does not command motion. Status bits follow
GEN programming manual table 6-2: positive limit 0x20, negative limit 0x40,
alarm 0x02, following error 0x10, emergency 0x100. A joint is accepted only
when its configured limit bit is set and the opposite limit, alarm, following
error, and emergency bits are clear. The offset records the current encoder;
it does not call GTN_ZeroPos and does not move the axis.
"""

import argparse
import math
import os
import sys
from typing import Dict, Optional, Sequence

import yaml

JOINTS = ["Joint_Body"] + [f"Joint_{side}{index}" for side in ("l", "r") for index in range(1, 8)]
POSITIVE_LIMIT = 0x20
NEGATIVE_LIMIT = 0x40
ALARM = 0x02
FOLLOW_ERROR = 0x10
EMERGENCY = 0x100
DEFAULT_PULSES_PER_RAD = 83443.02680376362
DEFAULT_WRAP_PERIOD = 524288


def active_limit_sides(status_bits: int) -> set:
    sides = set()
    if status_bits & POSITIVE_LIMIT:
        sides.add("positive")
    if status_bits & NEGATIVE_LIMIT:
        sides.add("negative")
    return sides


def offset_pulse(
    encoder_pulse: int,
    angle_rad: float,
    pulses_per_unit: float = DEFAULT_PULSES_PER_RAD,
    wrap_period: int = DEFAULT_WRAP_PERIOD,
    model_zero: float = 0.0,
) -> int:
    if wrap_period <= 0 or pulses_per_unit == 0.0 or not math.isfinite(angle_rad):
        raise ValueError("标定参数无效")
    target_delta = (angle_rad - model_zero) * pulses_per_unit
    if abs(target_delta) >= wrap_period / 2.0:
        raise ValueError("限位角度超出编码器半个周期，无法唯一对应零点")
    return int(round(float(encoder_pulse) - target_delta))


def load_references(path: str) -> Dict[str, Optional[dict]]:
    with open(path, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    table = document.get("limit_reference")
    if not isinstance(table, dict):
        raise ValueError(f"{path} 缺少 limit_reference")
    references = {}
    for name in JOINTS:
        item = table.get(name)
        if not isinstance(item, dict):
            raise ValueError(f"{path} 缺少关节 {name}")
        side = item.get("side")
        angle = item.get("angle_rad")
        if side is None and angle is None:
            references[name] = None
            continue
        if side not in ("positive", "negative"):
            raise ValueError(f"{name} 的 side 必须是 positive 或 negative")
        angle_rad = float(angle)
        if not math.isfinite(angle_rad):
            raise ValueError(f"{name} 的 angle_rad 不是有限数值")
        references[name] = {"side": side, "angle_rad": angle_rad}
    return references


def evaluate_joint(state, reference: Optional[dict]) -> dict:
    name = state.joint_name
    if reference is None:
        sides = ",".join(sorted(active_limit_sides(int(state.status_bits)))) or "无"
        return {
            "joint": name,
            "accepted": False,
            "reason": (
                f"未配置限位侧和角度，encoder={int(state.encoder_pulse_raw)} "
                f"status=0x{int(state.status_bits):x} 限位={sides}"
            ),
        }
    if not state.encoder_valid:
        return {"joint": name, "accepted": False, "reason": "编码器无效"}
    if state.status_bits & (ALARM | FOLLOW_ERROR | EMERGENCY):
        return {"joint": name, "accepted": False, "reason": f"驱动异常，status=0x{int(state.status_bits):x}"}
    sides = active_limit_sides(int(state.status_bits))
    if sides != {reference["side"]}:
        if not sides:
            reason = "限位未触发"
        elif len(sides) > 1:
            reason = "正负限位同时触发"
        else:
            reason = f"当前是{next(iter(sides))}限位，配置的是{reference['side']}"
        return {"joint": name, "accepted": False, "reason": reason}
    try:
        pulse = offset_pulse(int(state.encoder_pulse_raw), reference["angle_rad"])
    except ValueError as error:
        return {"joint": name, "accepted": False, "reason": str(error)}
    return {"joint": name, "accepted": True, "offset_pulse": pulse, "reason": "限位已触发"}


def render_offsets(rows: Sequence[dict]) -> str:
    lines = ["zero_offset_pulse:"]
    for row in rows:
        if row["accepted"]:
            lines.append(f"  {row['joint']}: {row['offset_pulse']}")
    if len(lines) == 1:
        lines.append("  {}")
    return "\n".join(lines) + "\n"


def merge_offsets(path: str, rows: Sequence[dict]) -> None:
    with open(path, encoding="utf-8") as handle:
        lines = handle.readlines()
    replacements = {
        row["joint"]: int(row["offset_pulse"])
        for row in rows if row["accepted"]
    }
    found = set()
    updated = []
    for line in lines:
        stripped = line.lstrip()
        name = stripped.split(":", 1)[0] if ":" in stripped else ""
        if name in replacements:
            indent = line[:len(line) - len(stripped)]
            updated.append(f"{indent}{name}: {replacements[name]}\n")
            found.add(name)
        else:
            updated.append(line)
    missing = set(replacements) - found
    if missing:
        raise ValueError(f"{path} 缺少关节 {sorted(missing)[0]}")
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.writelines(updated)
    os.replace(temporary, path)


def _self_check() -> None:
    class State:
        def __init__(self, name, status, encoder=100000, valid=True):
            self.joint_name = name
            self.status_bits = status
            self.encoder_pulse_raw = encoder
            self.encoder_valid = valid

    assert active_limit_sides(POSITIVE_LIMIT) == {"positive"}
    assert active_limit_sides(NEGATIVE_LIMIT) == {"negative"}
    assert offset_pulse(100000, 0.0, pulses_per_unit=1000.0) == 100000
    assert offset_pulse(100000, 1.0, pulses_per_unit=1000.0) == 99000
    assert offset_pulse(0, -1.0, pulses_per_unit=1000.0) == 1000
    try:
        offset_pulse(0, 4.0, pulses_per_unit=100000.0, wrap_period=524288)
        raise AssertionError("超范围角度应被拒绝")
    except ValueError:
        pass
    held = evaluate_joint(State("Joint_l1", NEGATIVE_LIMIT), {"side": "negative", "angle_rad": -0.15})
    assert held["accepted"] and held["offset_pulse"] == offset_pulse(100000, -0.15)
    assert not evaluate_joint(State("Joint_l1", 0), {"side": "negative", "angle_rad": 0.0})["accepted"]
    assert not evaluate_joint(State("Joint_l1", POSITIVE_LIMIT | NEGATIVE_LIMIT), {"side": "positive", "angle_rad": 0.0})["accepted"]
    assert not evaluate_joint(State("Joint_l1", NEGATIVE_LIMIT | ALARM), {"side": "negative", "angle_rad": 0.0})["accepted"]
    assert not evaluate_joint(State("Joint_l1", NEGATIVE_LIMIT), None)["accepted"]
    reference_path = os.path.join(os.path.dirname(__file__), "..", "config", "limit_calibration.yaml")
    loaded = load_references(reference_path)
    assert all(loaded[name] is None for name in JOINTS)
    sample = os.path.join(os.path.dirname(__file__), ".zero_offsets_self_check.yaml")
    with open(sample, "w", encoding="utf-8") as handle:
        handle.write("zero_offset_pulse:\n  Joint_l1: 0\n  Joint_l2: 3\n")
    try:
        merge_offsets(sample, [{"joint": "Joint_l1", "accepted": True, "offset_pulse": 42}])
        with open(sample, encoding="utf-8") as handle:
            text = handle.read()
        assert "Joint_l1: 42" in text and "Joint_l2: 3" in text
    finally:
        os.remove(sample)
    print("limit_zero_calibration self-check passed")


def main(argv: Sequence[str]) -> int:
    if "--self-check" in argv:
        _self_check()
        return 0

    parser = argparse.ArgumentParser(description="根据已经触发的硬件限位计算零点脉冲")
    parser.add_argument("--reference", required=True, help="limit_calibration.yaml 路径")
    parser.add_argument("--write", default="", help="把通过的关节写入这个 zero_offsets.yaml，默认只打印")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(argv[1:])
    references = load_references(args.reference)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from x1_coffee_interfaces.msg import MotorStateArray

    class Reader(Node):
        def __init__(self):
            super().__init__("limit_zero_calibration")
            self.states = None
            self.create_subscription(MotorStateArray, "/x1/motor_states", self._on_motors, qos_profile_sensor_data)

        def _on_motors(self, message):
            by_name = {state.joint_name: state for state in message.states}
            if all(name in by_name for name in JOINTS):
                self.states = [by_name[name] for name in JOINTS]

    rclpy.init(args=None)
    node = Reader()
    end = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while rclpy.ok() and node.states is None and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.states is None:
            print("超时：没有收到完整的 /x1/motor_states", file=sys.stderr)
            return 1
        rows = [evaluate_joint(state, references[state.joint_name]) for state in node.states]
    finally:
        node.destroy_node()
        rclpy.shutdown()

    for row in rows:
        if row["accepted"]:
            print(f"{row['joint']}: zero_offset_pulse={row['offset_pulse']}")
        else:
            print(f"{row['joint']}: 跳过，{row['reason']}")
    accepted = [row for row in rows if row["accepted"]]
    print(render_offsets(rows), end="")
    if not args.write:
        print("未写入文件。确认数值后使用 --write 指定 zero_offsets.yaml。")
        return 0
    if not accepted:
        print("没有关节通过标定，未修改文件。", file=sys.stderr)
        return 1
    merge_offsets(args.write, rows)
    print(f"已写入 {args.write} 中通过标定的 {len(accepted)} 个关节。重新加载控制程序后才会生效。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
