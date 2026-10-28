import math
import os
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "48")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ.setdefault("ROS_LOG_DIR", "/tmp/x1_python_device_sdk_contract_log")

import rclpy
from x1_coffee_device_sdk_py.clients import GripperClient, SafetyClient, WaistClient
from x1_coffee_interfaces.srv import SafetyCommand


class ClientContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)
        rclpy.init()
        cls.node = rclpy.create_node("python_device_sdk_contract_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gripper_side_must_be_left_or_right(self):
        for side in ("", "middle", "Left", "both"):
            with self.assertRaises(ValueError):
                GripperClient(None, side)

    def test_gripper_move_to_rejects_non_finite_values(self):
        gripper = GripperClient(self.node, "left")
        for position in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                gripper.move_to(position)
        for effort in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                gripper.move_to(0.0, effort)

    def test_waist_command_radians_rejects_non_finite_values(self):
        waist = WaistClient(self.node)
        for position in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                waist.command_radians(position)

    def test_safety_constants_match_safety_command(self):
        self.assertEqual(SafetyClient.PAUSE, 1)
        self.assertEqual(SafetyClient.RESUME, 2)
        self.assertEqual(SafetyClient.FAULT_RESET, 3)
        self.assertEqual(SafetyClient.EMERGENCY_STOP, 4)
        self.assertEqual(SafetyClient.PAUSE, SafetyCommand.Request.PAUSE)
        self.assertEqual(SafetyClient.RESUME, SafetyCommand.Request.RESUME)
        self.assertEqual(SafetyClient.FAULT_RESET, SafetyCommand.Request.FAULT_RESET)
        self.assertEqual(SafetyClient.EMERGENCY_STOP, SafetyCommand.Request.EMERGENCY_STOP)

    def test_gripper_names_and_finite_goal_are_not_rescaled(self):
        # 只替换发送函数，避免向动作服务器发出目标。
        left = GripperClient(self.node, "left")
        right = GripperClient(self.node, "right")
        self.assertEqual(left._client._action_name, "/left_gripper_controller/gripper_cmd")
        self.assertEqual(right._client._action_name, "/right_gripper_controller/gripper_cmd")
        for side in (None, "LEFT", "left ", " right"):
            with self.assertRaisesRegex(ValueError, "side must be 'left' or 'right'"):
                GripperClient(self.node, side)

        captured = {}

        def fake_send(goal, feedback_callback=None, goal_uuid=None):
            captured["goal"] = goal
            return "pending-future"

        left._client.send_goal_async = fake_send
        # 动作定义没有 0–1000。大正数和负值都应原样进入 goal，并直接返回 future。
        result = left.move_to(1000.0, max_effort=-3.5)
        self.assertEqual(result, "pending-future")
        self.assertEqual(captured["goal"].command.position, 1000.0)
        self.assertEqual(captured["goal"].command.max_effort, -3.5)
        self.assertIs(left.wait_for_server(0.01), False)

    def test_waist_publishes_radians_without_degree_conversion(self):
        waist = WaistClient(self.node)
        self.assertEqual(
            waist._publisher.topic_name,
            "/waist_forward_position_controller/commands")
        published = []
        waist._publisher.publish = lambda message: published.append(list(message.data))
        waist.command_radians(7.0)
        waist.command_radians(-0.4)
        self.assertEqual(published, [[7.0], [-0.4]])
        with self.assertRaisesRegex(ValueError, "position must be finite"):
            waist.command_radians(math.nan)
        self.assertEqual(published, [[7.0], [-0.4]])
        with self.assertRaisesRegex(ValueError, "command topic must not be empty"):
            WaistClient(self.node, "")

    def test_safety_command_returns_future_without_treating_it_as_arrived(self):
        safety = SafetyClient(self.node)
        self.assertEqual(safety._client.srv_name, "/x1/safety/command")
        captured = {}

        def fake_call(request):
            captured["request"] = request
            return "pending-future"

        safety._client.call_async = fake_call
        result = safety.command(SafetyClient.EMERGENCY_STOP, "contract")
        self.assertEqual(result, "pending-future")
        self.assertEqual(captured["request"].command, 4)
        self.assertEqual(captured["request"].reason, "contract")
        self.assertIs(safety.wait_for_service(0.01), False)


if __name__ == "__main__":
    unittest.main()
