#!/usr/bin/env python3
"""验证 SDK 内部 MoveIt 语义副本的单臂、双臂和腰身规划组。"""

import json
import sys

import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node


ALL_JOINTS = [
    "Joint_Body",
    "Joint_l1", "Joint_l2", "Joint_l3", "Joint_l4", "Joint_l5", "Joint_l6", "Joint_l7",
    "Joint_r1", "Joint_r2", "Joint_r3", "Joint_r4", "Joint_r5", "Joint_r6", "Joint_r7",
]

CASES = {
    "left_arm": {"Joint_l1": 0.05, "Joint_l2": 0.05},
    "right_arm": {"Joint_r1": -0.05, "Joint_r2": -0.05},
    "both_arms": {"Joint_l1": 0.05, "Joint_r1": -0.05},
    "waist_group": {"Joint_Body": 0.05},
    "tri_group": {
        "Joint_Body": 0.05, "Joint_l1": 0.05, "Joint_r1": -0.05,
    },
}


def request_for(group_name: str, goals: dict[str, float]) -> GetMotionPlan.Request:
    request = GetMotionPlan.Request()
    plan = request.motion_plan_request
    plan.group_name = group_name
    plan.num_planning_attempts = 1
    plan.allowed_planning_time = 5.0
    plan.max_velocity_scaling_factor = 0.1
    plan.max_acceleration_scaling_factor = 0.1
    plan.start_state.joint_state.name = ALL_JOINTS
    plan.start_state.joint_state.position = [0.0] * len(ALL_JOINTS)
    plan.start_state.is_diff = False
    constraints = Constraints()
    constraints.name = f"{group_name}_acceptance_goal"
    for name, position in goals.items():
        constraints.joint_constraints.append(
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=1.0e-3,
                tolerance_below=1.0e-3,
                weight=1.0,
            )
        )
    plan.goal_constraints = [constraints]
    return request


def main() -> int:
    rclpy.init()
    node = Node("x1_coffee_sdk_moveit_planning_verifier")
    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=30.0):
        print(json.dumps({"success": False, "error": "规划服务不可用"}, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
        return 2

    results = []
    for group_name, goals in CASES.items():
        future = client.call_async(request_for(group_name, goals))
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            results.append({"group": group_name, "success": False, "error": "调用超时"})
            continue
        response = future.result().motion_plan_response
        trajectory = response.trajectory.joint_trajectory
        success = response.error_code.val == MoveItErrorCodes.SUCCESS and bool(trajectory.points)
        results.append({
            "group": group_name,
            "success": success,
            "error_code": response.error_code.val,
            "joint_count": len(trajectory.joint_names),
            "point_count": len(trajectory.points),
        })

    print(json.dumps(results, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(item["success"] for item in results) else 1


if __name__ == "__main__":
    sys.exit(main())
