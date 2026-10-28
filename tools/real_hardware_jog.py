#!/usr/bin/env python3
"""Send one small, profile-aware real-hardware joint increment for commissioning."""

import argparse
import math
import sys
import time
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


LEFT = [f"Joint_l{i}" for i in range(1, 8)]
RIGHT = [f"Joint_r{i}" for i in range(1, 8)]
PROFILES = {
    "coffee_15_no_slide": ["Joint_Body", *LEFT, *RIGHT],
}

# Keep these commissioning limits deliberately identical to the robot model.
# They are a last local guard only; physical limits and the controller's own
# limits remain authoritative.
LIMITS = {
    "Joint_Body": (-2.0 * math.pi, 2.0 * math.pi),
    "Joint_l1": (-math.pi, math.pi),
    "Joint_l2": (-0.15, math.pi),
    "Joint_l3": (-math.pi, math.pi),
    "Joint_l4": (-1.5707963, 2.0943951),
    "Joint_l5": (-math.pi, math.pi),
    "Joint_l6": (-1.7453292, 1.7453292),
    "Joint_l7": (-2.0 * math.pi, 2.0 * math.pi),
    "Joint_r1": (-math.pi, math.pi),
    "Joint_r2": (-math.pi, 0.15),
    "Joint_r3": (-math.pi, math.pi),
    "Joint_r4": (-2.0943951, 1.5707963),
    "Joint_r5": (-math.pi, math.pi),
    "Joint_r6": (-1.7453292, 1.7453292),
    "Joint_r7": (-2.0 * math.pi, 2.0 * math.pi),
}


def controller_for_joint(joint: str) -> tuple[str, List[str]]:
    if joint == "Joint_Body":
        return "/waist_group_controller/follow_joint_trajectory", [joint]
    if joint in LEFT:
        return "/left_group_controller/follow_joint_trajectory", LEFT
    if joint in RIGHT:
        return "/right_group_controller/follow_joint_trajectory", RIGHT
    raise ValueError(f"unsupported joint: {joint}")


class JogNode(Node):
    def __init__(self) -> None:
        super().__init__("x1_real_hardware_jog")
        self.positions: Dict[str, float] = {}
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )

    def _on_joint_state(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                self.positions[name] = position


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read the current group state and jog exactly one physical joint."
    )
    parser.add_argument("--profile", required=True, choices=sorted(PROFILES))
    parser.add_argument("--joint", required=True)
    parser.add_argument("--delta", required=True, type=float,
                        help="Increment in radians")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Trajectory duration in seconds (2..20, default 5)")
    parser.add_argument(
        "--confirm",
        help="Must be exactly I_UNDERSTAND_REAL_MOTION",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Only read state and print the generated trajectory; never send a goal",
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.02,
        help="Post-motion position tolerance in radians",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.dry_run and args.confirm != "I_UNDERSTAND_REAL_MOTION":
        print("Refusing motion: invalid --confirm value", file=sys.stderr)
        return 2
    if args.joint not in PROFILES[args.profile]:
        print(
            f"Refusing motion: {args.joint} is not a physical joint in {args.profile}",
            file=sys.stderr,
        )
        return 2
    if not math.isfinite(args.delta) or not math.isfinite(args.duration):
        print("Refusing motion: delta/duration must be finite", file=sys.stderr)
        return 2
    maximum_delta = math.radians(1.0)
    if abs(args.delta) <= 0.0 or abs(args.delta) > maximum_delta:
        unit = "rad"
        print(
            f"Refusing motion: abs(delta) must be in (0, {maximum_delta}] {unit}",
            file=sys.stderr,
        )
        return 2
    if args.duration < 2.0 or args.duration > 20.0:
        print("Refusing motion: duration must be between 2 and 20 seconds", file=sys.stderr)
        return 2
    if not math.isfinite(args.tolerance) or args.tolerance <= 0.0:
        print("Refusing motion: tolerance must be finite and positive", file=sys.stderr)
        return 2

    action_name, group_joints = controller_for_joint(args.joint)
    rclpy.init()
    node = JogNode()
    try:
        deadline = node.get_clock().now() + Duration(seconds=5.0)
        while rclpy.ok() and any(name not in node.positions for name in group_joints):
            if node.get_clock().now() >= deadline:
                missing = [name for name in group_joints if name not in node.positions]
                print(f"No valid /joint_states sample for: {missing}", file=sys.stderr)
                return 3
            rclpy.spin_once(node, timeout_sec=0.1)

        current = [node.positions[name] for name in group_joints]
        start_position = current[group_joints.index(args.joint)]
        target = list(current)
        target[group_joints.index(args.joint)] += args.delta
        lower, upper = LIMITS[args.joint]
        target_position = target[group_joints.index(args.joint)]
        if not lower <= target_position <= upper:
            print(
                f"Refusing motion: target {target_position:.9f} for {args.joint} "
                f"is outside configured limits [{lower:.9f}, {upper:.9f}]",
                file=sys.stderr,
            )
            return 2
        print(f"profile: {args.profile}")
        print(f"action: {action_name}")
        print(f"joint: {args.joint}")
        print(f"current: {node.positions[args.joint]:.9f}")
        print(f"target:  {target[group_joints.index(args.joint)]:.9f}")
        print(f"duration: {args.duration:.3f} s")

        client = ActionClient(node, FollowJointTrajectory, action_name)
        if not client.wait_for_server(timeout_sec=5.0):
            print(f"Action server unavailable: {action_name}", file=sys.stderr)
            return 4

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = group_joints
        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(group_joints)
        point.time_from_start = Duration(seconds=args.duration).to_msg()
        goal.trajectory.points = [point]

        if args.dry_run:
            print("Action server is available")
            print("DRY RUN: no trajectory goal was sent")
            return 0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("Trajectory goal was rejected", file=sys.stderr)
            return 5
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.duration + 10.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
            print("Trajectory result timed out", file=sys.stderr)
            return 6
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(
                f"Trajectory failed: error_code={result.error_code} "
                f"error_string={result.error_string}", file=sys.stderr
            )
            return 7

        # The action result only confirms controller execution. Read back the
        # physical joint state as an independent end-to-end check.
        verify_deadline = time.monotonic() + 3.0
        measured = None
        while rclpy.ok() and time.monotonic() < verify_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            value = node.positions.get(args.joint)
            if value is not None:
                measured = value
                if abs(value - target_position) <= args.tolerance:
                    break
        if measured is None:
            print("Trajectory succeeded but no post-motion joint state was received", file=sys.stderr)
            return 8
        error = measured - target_position
        observed_delta = measured - start_position
        # Keep the original state before constructing the target.  A target
        # error alone is insufficient: with delta=0.01 and tolerance=0.02,
        # an unmoved joint would incorrectly pass (error=-0.01).
        expected_delta = args.delta
        delta_error = observed_delta - expected_delta
        print(f"measured: {measured:.9f}")
        print(f"position_error: {error:.9f}")
        print(f"observed_delta: {observed_delta:.9f}")
        print(f"delta_error: {delta_error:.9f}")
        delta_tolerance = max(0.002, abs(expected_delta) * 0.25)
        if abs(error) > args.tolerance or abs(delta_error) > delta_tolerance:
            print(
                f"Trajectory action succeeded but physical motion verification failed: "
                f"observed_delta={observed_delta:.6f}, expected_delta={expected_delta:.6f}, "
                f"delta_tolerance={delta_tolerance:.6f}, target_error={error:.6f}",
                file=sys.stderr,
            )
            return 9
        print("Trajectory succeeded and physical joint-state verification passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
