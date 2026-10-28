#!/usr/bin/env python3
"""Offline regressions for startup PVT protocol and measured arrival."""
import ast
from pathlib import Path
import runpy
import sys
import types
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "scripts/robot_startup.py"
NS = runpy.run_path(str(SOURCE), run_name="startup_test")
TREE = ast.parse(SOURCE.read_text())
METHODS = {}
for item in ast.walk(TREE):
    if isinstance(item, ast.FunctionDef) and item.name in ("run", "_return_to_mechanical_zero", "_start_motion"):
        scope = dict(NS)
        exec(compile(ast.Module(body=[item], type_ignores=[]), str(SOURCE), "exec"), scope)
        METHODS[item.name] = (scope[item.name], scope)

class StartupRegression(unittest.TestCase):
    def exercise(self, reply="1 ok", reaches=True):
        clock = [0.0]
        logs = []
        sent = []
        target = {name: 0.0 for name in NS["JOINTS"]}
        node = types.SimpleNamespace(joint_states={name: 0.2 for name in target})
        node.get_parameter = lambda key: types.SimpleNamespace(value=0.02 if key == "pose_tolerance_rad" else "unused")
        node.get_logger = lambda: types.SimpleNamespace(info=logs.append, error=logs.append)
        node.create_subscription = lambda kind, topic, callback, qos: setattr(node, "result_callback", callback)
        def publish(message):
            sent.append(message.data)
            if reply is not None:
                node.result_callback(types.SimpleNamespace(data=reply))
        node.create_publisher = lambda *args: types.SimpleNamespace(get_subscription_count=lambda: 1, publish=publish)
        def spin(*args, **kwargs):
            clock[0] += 0.1
            if reaches and clock[0] >= 0.8:
                node.joint_states = dict(target)
        def wait(timeout, ready):
            for _ in range(60):
                if ready():
                    return True
                spin()
            return ready()
        node.wait_until = wait
        function, scope = METHODS["_return_to_mechanical_zero"]
        scope["rclpy"] = types.SimpleNamespace(ok=lambda: True, spin_once=spin)
        scope["load_named_positions"] = lambda *args: target
        message_module = types.ModuleType("std_msgs.msg")
        message_module.Float64MultiArray = type("Array", (), {})
        message_module.String = type("String", (), {})
        with patch.dict(sys.modules, {"std_msgs.msg": message_module}), patch("time.monotonic", side_effect=lambda: clock[0]):
            result = function(node)
        return result, sent, logs

    def test_protocol_and_measured_arrival(self):
        result, sent, logs = self.exercise()
        self.assertTrue(result)
        self.assertEqual(sent[0][:3], [1.0, 0.8, 1.0])
        self.assertEqual(len(sent[0]), 18)

    def test_accepted_but_stationary_is_failure(self):
        result, _, logs = self.exercise(reaches=False)
        self.assertFalse(result)
        self.assertTrue(any("未到位" in line for line in logs))

    def test_rejected_and_legacy_malformed_replies(self):
        for reply in ("1 fail rejected", "0 fail malformed"):
            with self.subTest(reply=reply):
                result, _, logs = self.exercise(reply=reply)
                self.assertFalse(result)
                self.assertTrue(any("被拒绝" in line for line in logs))

    def test_missing_reply(self):
        result, _, logs = self.exercise(reply=None)
        self.assertFalse(result)
        self.assertTrue(any("没有收到" in line for line in logs))

    def test_failed_zero_aborts_startup(self):
        function, scope = METHODS["run"]
        scope["hardware_faults"] = lambda states: []
        scope["drive_enable_faults"] = lambda *args: []
        node = types.SimpleNamespace(motor_states=[], joint_states={name: 0.2 for name in NS["JOINTS"]})
        node.get_parameter = lambda key: types.SimpleNamespace(value={
            "check_timeout_sec": 1.0, "drives_enabled": True,
            "motion_enabled": True, "use_fake_hardware": False,
        }[key])
        node.get_logger = lambda: types.SimpleNamespace(info=lambda *args: None, error=lambda *args: None)
        node.wait_until = lambda timeout, ready: ready()
        node._return_to_mechanical_zero = lambda: False
        self.assertEqual(function(node), 1)

    def run_without_initial_pose(self, motion=True, fake=False, controllers=True, armed=True, requested=True):
        function, scope = METHODS["run"]
        scope["hardware_faults"] = lambda states: []
        scope["drive_enable_faults"] = lambda *args: []
        events = []
        node = types.SimpleNamespace(motor_states=[], joint_states={name: 0.2 for name in NS["JOINTS"]})
        node.get_parameter = lambda key: types.SimpleNamespace(value={
            "check_timeout_sec": 1.0, "drives_enabled": True,
            "motion_enabled": motion, "use_fake_hardware": fake,
            "move_to_initial": False, "arm_for_moveit": requested,
        }[key])
        node.get_logger = lambda: types.SimpleNamespace(info=lambda *args: None, error=lambda *args: None)
        node.wait_until = lambda timeout, ready: ready()
        node._return_to_mechanical_zero = lambda: events.append("zero") or True
        node.controllers_active = lambda: events.append("controllers") or controllers
        node._start_motion = lambda: events.append("arm") or armed
        return function(node), events

    def test_real_moveit_ready_without_initial_pose(self):
        result, events = self.run_without_initial_pose()
        self.assertEqual(result, 0)
        self.assertEqual(events, ["zero", "controllers", "arm"])

    def test_readonly_and_fake_do_not_arm_real_hardware(self):
        for kwargs in ({"motion": False}, {"fake": True}, {"requested": False}):
            result, events = self.run_without_initial_pose(**kwargs)
            self.assertEqual(result, 0)
            self.assertEqual(events, ["zero"] if kwargs.get("requested") is False else [])

    def test_controller_or_arming_failure_aborts(self):
        result, events = self.run_without_initial_pose(controllers=False)
        self.assertEqual(result, 1)
        self.assertEqual(events, ["zero", "controllers"])
        result, events = self.run_without_initial_pose(armed=False)
        self.assertEqual(result, 1)
        self.assertEqual(events, ["zero", "controllers", "arm"])

    def test_controller_targets_align_before_arming(self):
        function, scope = METHODS["_start_motion"]
        for aligned in (True, False):
            with self.subTest(aligned=aligned):
                events = []
                result = types.SimpleNamespace(success=True, message="armed")
                future = types.SimpleNamespace(done=lambda: True, result=lambda: result)
                node = types.SimpleNamespace(joint_states={name: 0.2 for name in NS["JOINTS"]})
                node.get_parameter = lambda key: types.SimpleNamespace(value=1.0)
                node.get_logger = lambda: types.SimpleNamespace(info=lambda *args: None, error=lambda *args: None)
                def command(target, duration):
                    self.assertEqual(target, node.joint_states)
                    events.append("hold_current")
                    return aligned
                node._command_pose = command
                node.start_motion = types.SimpleNamespace(
                    wait_for_service=lambda **kwargs: True,
                    call_async=lambda request: events.append("arm") or future)
                scope["Trigger"] = types.SimpleNamespace(Request=lambda: None)
                scope["rclpy"] = types.SimpleNamespace(spin_until_future_complete=lambda *args, **kwargs: None)
                self.assertEqual(function(node), aligned)
                self.assertEqual(events, ["hold_current", "arm"] if aligned else ["hold_current"])

    def test_all_controllers_require_measured_arrival(self):
        sdk = ROOT.parent
        for relative in ("x1_coffee_integration/config/controllers.yaml", "x1_coffee_moveit_config/config/ros2_controllers.yaml"):
            config = yaml.safe_load((sdk / relative).read_text())
            for controller, joints in NS["GROUPS"].items():
                constraints = config[controller]["ros__parameters"]["constraints"]
                self.assertEqual(constraints["goal_time"], 2.0)
                for joint in joints:
                    self.assertEqual(constraints[joint]["goal"], 0.02)

if __name__ == "__main__":
    unittest.main()
