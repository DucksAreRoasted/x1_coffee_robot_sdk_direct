#!/usr/bin/env python3
"""MIT 摩擦/重力辨识：CSP 匀速拖动，正反两次取扭矩差。

CST 缓升的问题是被测轴在零力矩下自己就在漂，起动判据分不清漂移和起动。
改成在保持模式（CSP）下用 PVT 让被测轴以约 0.01 rad/s 匀速走 ±0.04 rad，
采匀速段的 0x6077（额定千分比）：

    tau_+ = tau_g + tau_f
    tau_- = tau_g - tau_f   ==>  tau_g=(tau_++tau_-)/2, tau_f=(tau_+-tau_-)/2

任意姿态可用；要求被测轴能小幅正反移动、其余轴保持 CSP、手臂有支撑。
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from x1_coffee_interfaces.msg import MotorStateArray

ORDER = ["Joint_Body", "Joint_l1", "Joint_l2", "Joint_l3", "Joint_l4", "Joint_l5",
         "Joint_l6", "Joint_l7", "Joint_r1", "Joint_r2", "Joint_r3", "Joint_r4",
         "Joint_r5", "Joint_r6", "Joint_r7"]
RIGHT = [("Joint_r1", 52.0), ("Joint_r2", 52.0), ("Joint_r3", 21.0), ("Joint_r4", 21.0),
         ("Joint_r5", 10.0), ("Joint_r6", 5.40), ("Joint_r7", 5.40)]
DELTA = 0.04     # rad，单程幅度
SEG = 4.0        # 走 2*DELTA 用的秒数 → 0.01 rad/s
SKIP = 1.0       # 丢掉段首尾各 1 秒，只看匀速段


class Drag(Node):
    def __init__(self):
        super().__init__("mit_drag_identify")
        self.q = {}
        self.tau = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        self.create_subscription(MotorStateArray, "/x1/motor_states", self._on_ms, 20)
        self.pub = self.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
        self.seq = 0

    def _on_js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.q[name] = position

    def _on_ms(self, msg):
        for state in msg.states:
            self.tau[state.hardware_axis] = state.torque_permille

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        while len(self.q) < 15:
            self.spin(0.1)
        return [self.q[name] for name in ORDER]

    def pvt(self, target, duration, sample_axis=None):
        msg = Float64MultiArray()
        msg.data = [float(self.seq), float(duration), 1.0] + [float(v) for v in target]
        self.seq += 1
        self.pub.publish(msg)
        samples = []
        t0 = time.time()
        while time.time() - t0 < duration + 0.4:
            rclpy.spin_once(self, timeout_sec=0.02)
            elapsed = time.time() - t0
            if sample_axis is not None and SKIP < elapsed < duration - SKIP:
                value = self.tau.get(sample_axis)
                if value is not None:
                    samples.append(float(value))
        return statistics.mean(samples) if samples else None


def main():
    wanted = sys.argv[1:]
    rclpy.init()
    node = Drag()
    client = node.create_client(Trigger, "/x1/start_motion")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("/x1/start_motion 不可用")
    client.call_async(Trigger.Request())
    node.spin(1.0)

    for name, rated in RIGHT:
        if wanted and name not in wanted:
            continue
        axis = ORDER.index(name)
        low = node.pose()
        high = list(low)
        low[axis] -= DELTA
        high[axis] += DELTA
        node.pvt(low, SEG)                       # 先走到下端点
        plus = node.pvt(high, SEG, sample_axis=axis + 1)   # 匀速上行，采扭矩
        minus = node.pvt(low, SEG, sample_axis=axis + 1)   # 匀速下行，采扭矩
        node.pvt(high, SEG)                      # 回原位
        gravity = friction = None
        if plus is not None and minus is not None:
            gravity = (plus + minus) / 2.0
            friction = (plus - minus) / 2.0
        print("%-9s 上行=%s‰ 下行=%s‰ 重力=%s 摩擦=%s" % (
            name,
            "-" if plus is None else "%.1f" % plus,
            "-" if minus is None else "%.1f" % minus,
            "-" if gravity is None else "%.1f‰" % gravity,
            "-" if friction is None else "%.1f‰" % friction))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
