#!/usr/bin/env python3
"""PVT 路点打包。不打开网口，也不创建动作服务器。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import pvt_trajectory_executor as executor


class PvtTrajectoryPacking(unittest.TestCase):
    def test_unlisted_joints_keep_the_measured_angle(self):
        measured = {name: float(index) for index, name in enumerate(executor.COFFEE_JOINTS)}
        positions = executor.full_positions(["Joint_l1"], [0.2], measured)
        self.assertEqual(positions[1], 0.2)
        self.assertEqual(positions[0], measured["Joint_Body"])
        self.assertEqual(positions[8], measured["Joint_r1"])

    def test_duration_keeps_moveit_time_when_it_is_slow_enough(self):
        previous = [0.0] * 15
        goal = [0.0] * 15
        goal[1] = 0.05
        duration = executor.segment_duration(previous, goal, 1.0)
        self.assertAlmostEqual(duration, 1.0)

    def test_duration_respects_the_joint_speed_cap(self):
        previous = [0.0] * 15
        goal = [0.0] * 15
        goal[1] = 0.8
        duration = executor.segment_duration(previous, goal, 0.1)
        self.assertAlmostEqual(duration, 0.8 / executor.MAX_JOINT_SPEED)

    def test_message_is_one_fifteen_axis_waypoint(self):
        message = executor.pvt_message(3, 1.5, list(range(15)))
        self.assertEqual(len(message.data), 18)
        self.assertEqual(message.data[0], 3.0)
        self.assertEqual(message.data[1], 1.5)
        self.assertEqual(message.data[2], 1.0)
        self.assertEqual(list(message.data[3:]), [float(index) for index in range(15)])

    def test_trajectory_time_uses_message_duration_fields(self):
        class Stamp:
            sec = 1
            nanosec = 500_000_000

        self.assertAlmostEqual(executor.stamp_seconds(Stamp()), 1.5)

    def test_path_message_carries_each_segment_time(self):
        rows = [[float(index) for index in range(15)], [float(index + 1) for index in range(15)]]
        message = executor.pvt_path_message(4, [0.4, 0.6], rows)
        self.assertEqual(list(message.data[:3]), [4.0, 1.0, 2.0])
        self.assertEqual(message.data[3], 0.4)
        self.assertEqual(list(message.data[4:19]), rows[0])
        self.assertEqual(message.data[19], 0.6)

    def test_unknown_joint_is_rejected(self):
        measured = {name: 0.0 for name in executor.COFFEE_JOINTS}
        with self.assertRaises(ValueError):
            executor.full_positions(["not_a_joint"], [0.0], measured)


if __name__ == "__main__":
    unittest.main()
