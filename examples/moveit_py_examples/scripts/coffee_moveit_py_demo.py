#!/usr/bin/env python3
"""Coffee MoveItPy 验收示例；默认只规划，显式 --execute 才请求执行。"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.logging import get_logger
from rclpy.utilities import remove_ros_args
from moveit_configs_utils import MoveItConfigsBuilder

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters


def plan_group(robot: MoveItPy, group_name: str, execute: bool, logger) -> bool:
    component = robot.get_planning_component(group_name)
    component.set_start_state_to_current_state()

    state = RobotState(robot.get_robot_model())
    state.set_to_default_values()
    group = robot.get_robot_model().get_joint_model_group(group_name)
    if group is None:
        logger.error(f"规划组不存在: {group_name}")
        return False

    # 以零位附近的小幅安全目标作为软件验收目标，不复用文档中的越界示例数值。
    values = np.zeros(len(group.active_joint_model_names), dtype=float)
    if values.size:
        values[0] = 0.05
    state.set_joint_group_positions(
        joint_model_group_name=group_name,
        position_values=values,
    )
    state.update()
    component.set_goal_state(robot_state=state)
    plan_parameters = PlanRequestParameters(robot, "plan_request_params")
    # 直接覆盖默认空值，避免不同 MoveIt2 构建版本对参数命名空间处理差异。
    plan_parameters.planning_pipeline = "ompl"
    plan_parameters.planner_id = "RRTConnectkConfigDefault"
    plan_parameters.planning_attempts = 1
    plan_parameters.planning_time = 5.0
    plan_parameters.max_velocity_scaling_factor = 0.1
    plan_parameters.max_acceleration_scaling_factor = 0.1
    result = component.plan(single_plan_parameters=plan_parameters)
    if not result:
        logger.error(f"规划失败: {group_name}")
        return False

    trajectory = result.trajectory
    trajectory_message = trajectory.get_robot_trajectory_msg()
    logger.info(
        f"规划成功: {group_name}, joints={len(trajectory_message.joint_trajectory.joint_names)}, "
        f"points={len(trajectory_message.joint_trajectory.points)}"
    )
    if execute:
        logger.info("请求执行；真实硬件执行前必须完成台架验收")
        robot.execute(trajectory, controllers=[])
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("group", choices=["left", "right", "both"], default="left", nargs="?")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    group_name = {"left": "left_arm", "right": "right_arm", "both": "both_arms"}[args.group]
    rclpy.init()
    logger = get_logger("x1_coffee_moveit_py_demo")
    moveit_config = MoveItConfigsBuilder(
        "HRB3Q-LC", package_name="x1_coffee_moveit_config"
    ).to_moveit_configs().to_dict()
    # MoveItPy 直接构造 MoveItCpp，所需参数名与 move_group 的列表参数不同。
    pipeline_names = list(moveit_config.pop("planning_pipelines"))
    moveit_config["planning_pipelines"] = {
        "pipeline_names": pipeline_names,
        "namespace": "",
    }
    moveit_config["plan_request_params"] = {
        "planning_attempts": 1,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "planning_time": 5.0,
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
    }
    robot = MoveItPy(
        node_name="x1_coffee_moveit_py_demo",
        config_dict=moveit_config,
    )
    try:
        return 0 if plan_group(robot, group_name, args.execute, logger) else 1
    finally:
        # MoveItPy 自己负责关闭其 rclcpp 上下文；不要再次调用 rclpy.shutdown()，
        # 否则在部分 MoveIt2/Humble 组合中会触发重复释放。
        robot.shutdown()


if __name__ == "__main__":
    sys.exit(main())
