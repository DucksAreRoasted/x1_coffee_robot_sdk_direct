"""Qt-free Python clients matching the public C++ device SDK."""

import math
from typing import Callable, Optional

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from x1_coffee_interfaces.msg import MotorStateArray
from x1_coffee_interfaces.srv import SafetyCommand


class _PositionPublisher:
    def __init__(self, node: Node, topic: str) -> None:
        if not topic:
            raise ValueError("command topic must not be empty")
        self._publisher = node.create_publisher(Float64MultiArray, topic, 10)

    def command(self, position: float) -> None:
        if not math.isfinite(position):
            raise ValueError("position must be finite")
        message = Float64MultiArray()
        message.data = [float(position)]
        self._publisher.publish(message)


class WaistClient(_PositionPublisher):
    def __init__(self, node: Node, topic: str = "/waist_forward_position_controller/commands") -> None:
        super().__init__(node, topic)

    def command_radians(self, position: float) -> None:
        self.command(position)


class GripperClient:
    def __init__(self, node: Node, side: str) -> None:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._client = ActionClient(
            node, GripperCommand, f"/{side}_gripper_controller/gripper_cmd"
        )

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def move_to(self, position: float, max_effort: float = 0.0):
        if not math.isfinite(position) or not math.isfinite(max_effort):
            raise ValueError("position and max_effort must be finite")
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self._client.send_goal_async(goal)


class SafetyClient:
    PAUSE = SafetyCommand.Request.PAUSE
    RESUME = SafetyCommand.Request.RESUME
    FAULT_RESET = SafetyCommand.Request.FAULT_RESET
    EMERGENCY_STOP = SafetyCommand.Request.EMERGENCY_STOP

    def __init__(self, node: Node, service: str = "/x1/safety/command") -> None:
        self._client = node.create_client(SafetyCommand, service)

    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def command(self, command: int, reason: str = ""):
        request = SafetyCommand.Request()
        request.command = command
        request.reason = reason
        return self._client.call_async(request)


class MotorStateReader:
    def __init__(self, node: Node, topic: str = "/x1/motor_states") -> None:
        self.latest: Optional[MotorStateArray] = None
        self._callback: Optional[Callable[[MotorStateArray], None]] = None
        self._subscription = node.create_subscription(
            MotorStateArray, topic, self._receive, 10
        )

    def set_callback(self, callback: Optional[Callable[[MotorStateArray], None]]) -> None:
        self._callback = callback

    def _receive(self, message: MotorStateArray) -> None:
        self.latest = message
        if self._callback is not None:
            self._callback(message)
