#!/usr/bin/env python3
"""MIT 摩擦/重力自动辨识。

对每根轴做一次正向和反向力矩缓升（0 → ±50‰，5‰/s），记录关节开始运动时的
起动扭矩。同轴两次结果满足：

    起动_+ = 重力分量 + 静摩擦
    起动_- = 重力分量 - 静摩擦   （方向相反时符号互换）

因此 重力 ≈ (起动_+ + 起动_-)/2，静摩擦 ≈ (起动_+ - 起动_-)/2。
两次都在 ±50‰ 内没有运动时，说明该轴的重力超过 5% 额定，这个量程测不了。

用法：ros2 run 或直接 python3 tools/mit_autotune.py [轴名...]
"""
import re
import subprocess
import sys
import time

NODE = "/x1_coffee_system_hardware"
AXES = [
    ("Joint_r1", 0x0100, 52.0),
    ("Joint_r2", 0x0200, 52.0),
    ("Joint_r3", 0x0400, 21.0),
    ("Joint_r4", 0x0800, 21.0),
    ("Joint_r5", 0x1000, 10.0),
    ("Joint_r6", 0x2000, 5.40),
    ("Joint_r7", 0x4000, 5.40),
]
RAMP_TO = 50
RAMP_RATE = 5.0


def run(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=40)


def pset(name, value):
    r = run("ros2 param set %s %s %s" % (NODE, name, value))
    if "successful" not in r.stdout:
        raise SystemExit("参数设置失败 %s=%s: %s%s" % (name, value, r.stdout, r.stderr))


def call(svc):
    r = run('ros2 service call %s std_srvs/srv/Trigger "{}"' % svc)
    m = re.search(r"success=(True|False), message='(.*)'\)", r.stdout, re.S)
    if not m:
        raise SystemExit("服务无响应 %s: %s%s" % (svc, r.stdout, r.stderr))
    return m.group(1) == "True", m.group(2)


def ramp(mask, target):
    pset("ramp_mask", mask)
    pset("ramp_from", 0)
    pset("ramp_to", target)
    pset("ramp_rate", RAMP_RATE)
    ok, msg = call("/x1/mit_ramp")
    if not ok:
        return None, msg
    deadline = time.time() + abs(target) / RAMP_RATE + 10.0
    while time.time() < deadline:
        ok, msg = call("/x1/mit_ramp_status")
        if "缓升中" not in msg:
            break
        time.sleep(0.5)
    m = re.search(r"起动扭矩 (-?\d+)‰", msg)
    return (int(m.group(1)) if m else None), msg


def main():
    wanted = sys.argv[1:]
    rows = []
    for name, mask, rated in AXES:
        if wanted and name not in wanted:
            continue
        plus, msg_plus = ramp(mask, RAMP_TO)
        call("/x1/mit_zero_check")          # 退出 CST，回到 CSP 保持
        minus, msg_minus = ramp(mask, -RAMP_TO)
        call("/x1/mit_zero_check")
        gravity = friction = None
        if plus is not None and minus is not None:
            gravity = (plus + minus) / 2.0
            friction = (plus - minus) / 2.0
        rows.append((name, rated, plus, minus, gravity, friction))
        print("%-9s 正向=%-6s 反向=%-6s 重力=%-7s 摩擦=%-7s" % (
            name,
            "无" if plus is None else "%d‰" % plus,
            "无" if minus is None else "%d‰" % minus,
            "-" if gravity is None else "%.1f‰" % gravity,
            "-" if friction is None else "%.1f‰" % friction))

    print()
    print("按死区 0.02 rad 反推 kp（‰/rad），kd 按 ζ=0.7 留待结合惯量计算：")
    for name, rated, plus, minus, gravity, friction in rows:
        if friction is None:
            print("%-9s 量程内未起动，需要更高力矩或换姿态" % name)
            continue
        kp = abs(friction) / 0.02
        print("%-9s 摩擦 %.1f‰ → kp ≈ %.0f‰/rad" % (name, friction, kp))


if __name__ == "__main__":
    main()
