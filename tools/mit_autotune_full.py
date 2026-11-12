#!/usr/bin/env python3
"""MIT 全自动标定：匀速拖动测摩擦/重力 → 算逐轴 kp/kd 和重力偏置 → 写回 YAML。

流程（每根轴约 20 秒）：
1. 用 PVT 以约 0.01 rad/s 让该轴正反各走 0.08 rad，采匀速段 0x6077；
2. 上行/下行取平均得重力 τg、取差得摩擦 τf；
3. kp = |τf| / 死区（默认 0.05 rad），kd = 2ζ√(kp·J)，J 由 URDF 模型按当前姿态算；
4. 重力偏置 = 实测 τg − 模型预测，写进 joint_bias，使前馈在当前姿态成立。

写 YAML 前会备份原文件；YAML 只在启动时读取，写完需要重启保持程序才生效。

用法：
  python3 tools/mit_autotune_full.py                # 全部 7 轴，写入
  python3 tools/mit_autotune_full.py Joint_r7       # 只标一根
  python3 tools/mit_autotune_full.py --dry-run      # 只测和打印，不写文件
"""
import argparse
import math
import os
import shutil
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "gravity_identification"))

import rclpy
from mit_drag_identify import DELTA, ORDER, RIGHT, SEG, Drag
from model import Model

URDF = "src/arm_describle/urdf/HRB3Q-LC.urdf"
YAML = "src/x1_coffee_hardware_core/config/right_arm_gravity.yaml"
MODEL_KEYS = ["mass_scale", "com_dx", "com_dy", "com_dz", "joint_scale", "joint_bias"]


def read_yaml(path):
    values = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            key, rest = line.split(":", 1)
            numbers = [float(v) for v in rest.split()[:7]]
            if len(numbers) == 7:
                values[key] = numbers
    return values


def write_yaml(path, values):
    lines = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            key = stripped.split(":", 1)[0] if ":" in stripped else None
            if key in values and not stripped.startswith("#"):
                lines.append("%s: %s\n" % (key, " ".join("%g" % v for v in values[key])))
            else:
                lines.append(line)
    shutil.copyfile(path, path + ".bak")
    with open(path, "w", encoding="utf-8") as handle:
        handle.writelines(lines)


def model_state(model, q, values):
    frames, axes = model.geometry(q)
    up = np.array([0.0, 0.0, 9.80665])
    selected = [model.by_name["Joint_r%d" % i].find("child").get("link") for i in range(1, 8)]
    tau = np.zeros(15)
    for name, (p, r, chain) in frames.items():
        mass, com, _ = model.links[name]
        if name in selected:
            slot = selected.index(name)
            mass *= values["mass_scale"][slot]
            com = com + np.array([values["com_dx"][slot], values["com_dy"][slot],
                                  values["com_dz"][slot]])
        world = p + r @ com
        for idx in chain:
            pivot, axis = axes[idx]
            tau[idx] += mass * (axis @ np.cross(world - pivot, up))
    _, _, _, inertia = model.regressor(q, "r")
    return tau[8:15], np.array([float(v) for v in inertia])


def measure(node, name):
    axis = ORDER.index(name)
    low = node.pose()
    high = list(low)
    low[axis] -= DELTA
    high[axis] += DELTA
    node.pvt(low, SEG)
    plus = node.pvt(high, SEG, sample_axis=axis + 1)
    minus = node.pvt(low, SEG, sample_axis=axis + 1)
    node.pvt(high, SEG)
    if plus is None or minus is None:
        return None, None
    return (plus + minus) / 2.0, (plus - minus) / 2.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("axes", nargs="*")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--deadband", type=float, default=0.05)
    parser.add_argument("--zeta", type=float, default=0.7)
    args = parser.parse_args()

    values = read_yaml(YAML)
    model = Model(URDF, (1.0, 1.0))
    rated = dict(RIGHT)

    rclpy.init()
    node = Drag()
    from std_srvs.srv import Trigger
    client = node.create_client(Trigger, "/x1/start_motion")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("/x1/start_motion 不可用")
    client.call_async(Trigger.Request())
    node.spin(1.0)

    kp = list(values.get("mit_kp", [0.0] * 7))
    kd = list(values.get("mit_kd", [0.0] * 7))
    bias = list(values.get("joint_bias", [0.0] * 7))
    print("轴     重力实测  模型预测   偏差    摩擦      J(kg·m²)  kp     kd")
    for name, rated_nm in RIGHT:
        if args.axes and name not in args.axes:
            continue
        slot = int(name[-1]) - 1
        gravity, friction = measure(node, name)
        if gravity is None:
            print("%-8s 采样失败，跳过" % name)
            continue
        q = node.pose()
        model_tau, inertia = model_state(model, q, values)
        gravity_nm = gravity * rated_nm / 1000.0
        friction_nm = friction * rated_nm / 1000.0
        bias_nm = gravity_nm - model_tau[slot]
        kp_si = abs(friction_nm) / args.deadband
        kp_pm = kp_si / rated_nm * 1000.0
        kd_si = 2.0 * args.zeta * math.sqrt(kp_si * float(inertia[slot]))
        kd_pm = kd_si / rated_nm * 1000.0
        kp[slot] = round(min(kp_pm, 2000.0), 0)
        kd[slot] = round(min(kd_pm, 500.0), 0)
        bias[slot] = round(bias_nm, 3)
        print("%-8s %+7.1f‰  %+7.2f  %+7.2f  %+6.1f‰  %7.4f  %5.0f  %5.0f" % (
            name, gravity, model_tau[slot], bias_nm, friction, float(inertia[slot]),
            kp[slot], kd[slot]))

    print("\n计算结果：")
    print("joint_bias: " + " ".join("%g" % v for v in bias))
    print("mit_kp:     " + " ".join("%g" % v for v in kp))
    print("mit_kd:     " + " ".join("%g" % v for v in kd))
    if args.dry_run:
        print("\n--dry-run：未写文件")
    else:
        values["joint_bias"] = bias
        values["mit_kp"] = kp
        values["mit_kd"] = kd
        write_yaml(YAML, values)
        print("\n已写入 %s（原文件备份为 .bak），重启保持程序后生效。" % YAML)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
