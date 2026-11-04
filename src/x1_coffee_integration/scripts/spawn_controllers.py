#!/usr/bin/env python3
"""按顺序加载控制器，并只激活指定的那一组。

并行的官方 spawner 会把同一次加载或切换送进 controller_manager 两遍。
第二遍因为控制器已经存在或已经激活而失败，spawner 随即退出，
同组里后面的控制器就不再处理。这里每次调用后都重新查询状态，
已经加载或已经激活视为成功。
"""

import sys

import rclpy
from rclpy.node import Node

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
)

MANAGER = "/controller_manager"
LABELS = {
    "joint_state_broadcaster": "关节状态",
    "openarm_compat_joint_state_broadcaster": "兼容关节状态",
    "motor_state_broadcaster": "电机状态",
    "left_group_controller": "左臂轨迹",
    "right_group_controller": "右臂轨迹",
    "waist_group_controller": "腰轨迹",
    "left_forward_position_controller": "左臂转发",
    "right_forward_position_controller": "右臂转发",
    "waist_forward_position_controller": "腰转发",
}


def label(name):
    return LABELS.get(name, name)


def controller_states(node):
    response = list_controllers(node, MANAGER, service_timeout=60.0, call_timeout=10.0)
    return {controller.name: controller.state for controller in response.controller}


def ensure_configured(node, name):
    for _ in range(6):
        state = controller_states(node).get(name)
        if state is None:
            result = load_controller(node, MANAGER, name, call_timeout=15.0)
            if not result.ok:
                node.get_logger().warn(f"! {label(name)} 加载返回失败，重新查询状态")
            continue
        if state == "unconfigured":
            result = configure_controller(node, MANAGER, name, call_timeout=15.0)
            if not result.ok:
                node.get_logger().warn(f"! {label(name)} 配置返回失败，重新查询状态")
            continue
        return state
    raise RuntimeError(f"{name} 未能加载并配置")


def ensure_active(node, name):
    if ensure_configured(node, name) == "active":
        node.get_logger().info(f"✓ {label(name)} 已激活")
        return
    for _ in range(4):
        result = switch_controllers(
            node, MANAGER, [], [name], True, True, 5.0, 15.0)
        state = controller_states(node).get(name)
        if state == "active":
            node.get_logger().info(f"✓ {label(name)} 已激活")
            return
        node.get_logger().warn(f"! {label(name)} 激活返回 {result.ok}，当前状态 {state}")
    raise RuntimeError(f"{name} 未能激活")


def ensure_inactive(node, name):
    state = ensure_configured(node, name)
    if state == "active":
        raise RuntimeError(f"{name} 应当保持未激活，但当前已激活")
    node.get_logger().info(f"● {label(name)} 已加载，未激活")


def main():
    rclpy.init()
    node = Node("spawn_controllers")
    node.declare_parameter("active_controllers", [""])
    node.declare_parameter("inactive_controllers", [""])
    active = [name for name in node.get_parameter("active_controllers").value if name]
    inactive = [name for name in node.get_parameter("inactive_controllers").value if name]
    try:
        for name in active:
            ensure_active(node, name)
        for name in inactive:
            ensure_inactive(node, name)
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info("✓ 控制器已按顺序就绪")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
