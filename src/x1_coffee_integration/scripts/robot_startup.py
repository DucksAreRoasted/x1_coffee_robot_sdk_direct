#!/usr/bin/env python3
"""Start the 15-axis control stack, enable the drives, then check hardware.

The control nodes have to be running before encoders and alarms can be read.
This program waits for that feedback, checks every physical axis, and when motion
is enabled returns to the mechanical zero with one PVT segment. That pose is the
URDF evaluated at initial_positions.yaml. Cyclic PT is armed only when
arm_for_moveit or move_to_initial is explicitly requested. The default startup
keeps PVT jogging available. Launching MoveIt is a separate step.
"""

import math
import os
import sys
import time
from typing import Dict, List, Mapping, Optional, Sequence

import yaml

JOINTS = ["Joint_Body"] + [f"Joint_{side}{index}" for side in ("l", "r") for index in range(1, 8)]
GROUPS = {
    "waist_group_controller": ["Joint_Body"],
    "left_group_controller": [f"Joint_l{index}" for index in range(1, 8)],
    "right_group_controller": [f"Joint_r{index}" for index in range(1, 8)],
}


def load_named_positions(path: str, key: str) -> Dict[str, float]:
    with open(path, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    values = document.get(key)
    if not isinstance(values, dict):
        raise ValueError(f"{path} 缺少 {key}")
    positions = {}
    for name in JOINTS:
        if name not in values:
            raise ValueError(f"{path} 缺少关节 {name}")
        position = float(values[name])
        if not math.isfinite(position):
            raise ValueError(f"{path} 中 {name} 不是有限数值")
        positions[name] = position
    return positions


def zero_offsets_uncalibrated(offsets: Mapping[str, float]) -> bool:
    return all(float(offsets[name]) == 0.0 for name in JOINTS)


def hardware_faults(states: Sequence[object]) -> List[str]:
    faults = []
    seen = {}
    for state in states:
        name = state.joint_name
        if name in seen:
            faults.append(f"{name} 重复出现")
        seen[name] = state
    missing = [name for name in JOINTS if name not in seen]
    if missing:
        faults.append("缺少关节反馈: " + ", ".join(missing))
    unexpected = sorted(set(seen) - set(JOINTS))
    if unexpected:
        faults.append("存在未知关节反馈: " + ", ".join(unexpected))
    for index, name in enumerate(JOINTS):
        state = seen.get(name)
        if state is None:
            continue
        expected_axis = index + 1
        if int(state.hardware_axis) != expected_axis:
            faults.append(f"{name} 轴号为 {state.hardware_axis}，期望 {expected_axis}")
        if not state.encoder_valid or not state.position_valid or not math.isfinite(state.position):
            faults.append(f"{name} 编码器或位置无效")
        if state.alarm:
            faults.append(f"{name} 报警，status=0x{int(state.status_bits):x}")
        if state.follow_error:
            faults.append(f"{name} 跟随误差")
        if state.emergency_input_active:
            faults.append(f"{name} 急停输入有效")
    return faults


def cia402_state_name(status_bits: int) -> str:
    return {
        0x0000: "未就绪",
        0x0040: "接通禁止",
        0x0021: "准备接通",
        0x0023: "已接通",
        0x0027: "已使能",
        0x0007: "快速停机",
        0x0008: "故障",
        0x000F: "故障反应",
    }.get(int(status_bits) & 0x006F, "未知")


def drive_enable_faults(states: Sequence[object], require_enabled: bool) -> List[str]:
    if not require_enabled:
        return []
    faults = []
    by_name = {state.joint_name: state for state in states}
    for name in JOINTS:
        state = by_name.get(name)
        if state is None:
            continue
        if not state.enabled:
            status_bits = int(state.status_bits)
            faults.append(
                f"{name} 轴{int(state.hardware_axis)} 未使能，"
                f"status=0x{status_bits:x}（{cia402_state_name(status_bits)}）")
        if state.moving:
            faults.append(f"{name} 上电后仍在运动")
    return faults


def should_arm_motion(
    move_to_initial: bool,
    trajectory_controllers_active: bool,
    use_fake_hardware: bool,
) -> bool:
    """Real PT starts only after a requested initial pose and active trajectory controllers.

    Fake hardware has no GTN transport, so it never calls /x1/start_motion.
    """
    return bool(move_to_initial) and bool(trajectory_controllers_active) and not bool(use_fake_hardware)


def motion_block_reason(
    current: Mapping[str, float],
    target: Mapping[str, float],
    max_joint_delta_rad: float,
    allow_large_motion: bool,
    use_fake_hardware: bool,
    motion_enabled: bool,
    offsets_uncalibrated: bool,
) -> Optional[str]:
    if not use_fake_hardware and not motion_enabled:
        return "实机运动未打开：需要 motion_enabled:=true"
    if not use_fake_hardware and offsets_uncalibrated:
        return "zero_offsets.yaml 仍是全 0，实机零位尚未写入，拒绝回初始姿态"
    oversized = []
    for name in JOINTS:
        delta = abs(float(target[name]) - float(current[name]))
        if math.isfinite(delta) and delta > max_joint_delta_rad:
            oversized.append(f"{name} 偏差 {delta:.4f} rad")
    if oversized and not allow_large_motion:
        return "初始姿态偏差超过 {:.3f} rad（{}）。确认路径安全后加 allow_large_motion:=true".format(
            max_joint_delta_rad, ", ".join(oversized))
    return None


def _self_check() -> None:
    class State:
        def __init__(self, name, axis, **flags):
            self.joint_name = name
            self.hardware_axis = axis
            self.position = 0.0
            self.status_bits = 0
            self.encoder_valid = True
            self.position_valid = True
            self.alarm = False
            self.follow_error = False
            self.emergency_input_active = False
            self.enabled = True
            self.moving = False
            for key, value in flags.items():
                setattr(self, key, value)

    healthy = [State(name, index + 1) for index, name in enumerate(JOINTS)]
    assert hardware_faults(healthy) == []
    alarmed = healthy[:]
    alarmed[0] = State("Joint_Body", 1, alarm=True, status_bits=0x2)
    assert any("报警" in item for item in hardware_faults(alarmed))
    assert zero_offsets_uncalibrated({name: 0.0 for name in JOINTS})
    current = {name: 0.0 for name in JOINTS}
    target = dict(current)
    target["Joint_l1"] = 1.0
    assert motion_block_reason(current, target, 0.5, False, False, True, True).startswith("zero_offsets")
    assert "偏差" in motion_block_reason(current, target, 0.5, False, True, False, False)
    assert motion_block_reason(current, current, 0.5, False, True, False, False) is None
    assert drive_enable_faults(healthy, False) == []
    assert drive_enable_faults(healthy, True) == []
    disabled = healthy[:]
    disabled[1] = State("Joint_l1", 2, enabled=False, status_bits=0)
    assert any("未使能" in item for item in drive_enable_faults(disabled, True))
    moving = healthy[:]
    moving[1] = State("Joint_l1", 2, enabled=True, moving=True, status_bits=0x200)
    assert any("仍在运动" in item for item in drive_enable_faults(moving, True))
    assert should_arm_motion(False, True, False) is False
    assert should_arm_motion(True, False, False) is False
    assert should_arm_motion(True, True, True) is False
    assert should_arm_motion(True, True, False) is True
    print("robot_startup self-check passed")


def main() -> int:
    if "--self-check" in sys.argv:
        _self_check()
        return 0

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from builtin_interfaces.msg import Duration as TimeDuration
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint
    from x1_coffee_interfaces.msg import MotorStateArray

    class Startup(Node):
        def __init__(self):
            super().__init__("robot_startup")
            share = get_package_share_directory("x1_coffee_integration")
            self.declare_parameter("use_fake_hardware", False)
            self.declare_parameter("motion_enabled", False)
            self.declare_parameter("drives_enabled", False)
            self.declare_parameter("move_to_initial", False)
            self.declare_parameter("arm_for_moveit", False)
            self.declare_parameter("allow_large_motion", False)
            self.declare_parameter("pose_duration_sec", 10.0)
            self.declare_parameter("pose_tolerance_rad", 0.02)
            self.declare_parameter("max_joint_delta_rad", 0.5)
            self.declare_parameter("check_timeout_sec", 30.0)
            self.declare_parameter("startup_pose_file", os.path.join(share, "config", "startup_pose.yaml"))
            self.declare_parameter("zero_offsets_file", os.path.join(share, "config", "zero_offsets.yaml"))
            self.declare_parameter(
                "initial_positions_file", os.path.join(share, "config", "initial_positions.yaml"))
            self.motor_states = None
            self.joint_states = {}
            self.create_subscription(MotorStateArray, "/x1/motor_states", self._on_motors, qos_profile_sensor_data)
            self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
            self.list_controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
            self.start_motion = self.create_client(Trigger, "/x1/start_motion")
            self.action_clients = {
                name: ActionClient(self, FollowJointTrajectory, f"/{name}/follow_joint_trajectory")
                for name in GROUPS
            }

        def _on_motors(self, message):
            self.motor_states = list(message.states)

        def _on_joints(self, message):
            for name, position in zip(message.name, message.position):
                if math.isfinite(position):
                    self.joint_states[name] = float(position)

        def wait_until(self, timeout_sec: float, ready) -> bool:
            deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
            while rclpy.ok() and self.get_clock().now() < deadline:
                if ready():
                    return True
                rclpy.spin_once(self, timeout_sec=0.1)
            return ready()

        def controllers_active(self) -> bool:
            if not self.list_controllers.service_is_ready():
                return False
            future = self.list_controllers.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done() or future.result() is None:
                return False
            active = {item.name for item in future.result().controller if item.state == "active"}
            return set(GROUPS).issubset(active)

        def run(self) -> int:
            timeout = float(self.get_parameter("check_timeout_sec").value)
            self.get_logger().info("控制栈已由启动程序拉起，开始等待 15 轴反馈")
            if not self.wait_until(timeout, lambda: self.motor_states is not None and all(name in self.joint_states for name in JOINTS)):
                self.get_logger().error("超时：没有收齐 /x1/motor_states 和 /joint_states")
                return 1
            faults = hardware_faults(self.motor_states)
            if faults:
                for fault in faults:
                    self.get_logger().error(fault)
                return 1
            self.get_logger().info("硬件检查通过：15 轴反馈有效，无报警、跟随误差或急停输入")
            require_enabled = bool(self.get_parameter("drives_enabled").value)
            if require_enabled:
                self.get_logger().info("等待 15 轴状态字进入使能，未使能的轴会每 2 秒列出来")
                next_report = 0.0
                def drives_ready() -> bool:
                    nonlocal next_report
                    faults = drive_enable_faults(self.motor_states, True)
                    now = time.monotonic()
                    if faults and now >= next_report:
                        self.get_logger().info(
                            "尚未使能 " + str(len(faults)) + " 轴: " + "；".join(faults))
                        next_report = now + 2.0
                    return not faults
                if not self.wait_until(timeout, drives_ready):
                    for fault in drive_enable_faults(self.motor_states, True):
                        self.get_logger().error(fault)
                    return 1
            if bool(self.get_parameter("drives_enabled").value) and not (
                bool(self.get_parameter("motion_enabled").value)
                and not bool(self.get_parameter("use_fake_hardware").value)
            ):
                self.get_logger().info("15 轴已使能并保持，未下发运动")
            use_fake_hardware = bool(self.get_parameter("use_fake_hardware").value)
            motion_enabled = bool(self.get_parameter("motion_enabled").value)
            if motion_enabled and not use_fake_hardware:
                # MoveIt 先对齐 URDF 零位（关节角全 0）。其它实机启动仍回机械零位。
                if bool(self.get_parameter("arm_for_moveit").value):
                    if not self._return_to_urdf_zero():
                        return 1
                elif not self._return_to_mechanical_zero():
                    return 1
            move_to_initial = bool(self.get_parameter("move_to_initial").value)
            if not move_to_initial:
                if (bool(self.get_parameter("arm_for_moveit").value)
                        and motion_enabled and not use_fake_hardware):
                    # Preparing real MoveIt execution does not require a move to
                    # startup_pose. Mechanical zero and hardware checks passed.
                    if not self.wait_until(timeout, self.controllers_active):
                        self.get_logger().error("轨迹控制器未全部激活: " + ", ".join(GROUPS))
                        return 1
                    if not self._start_motion():
                        return 1
                    self.get_logger().info("已对齐 URDF 零位，运动已允许。可以打开 RViz 规划执行")
                else:
                    self.get_logger().info("控制节点保持运行。未允许运动，MoveIt 轨迹不会写到驱动器")
                return 0
            controllers_ready = self.wait_until(timeout, self.controllers_active)
            if not controllers_ready:
                self.get_logger().error("轨迹控制器未全部激活: " + ", ".join(GROUPS))
                return 1
            pose_file = str(self.get_parameter("startup_pose_file").value)
            offset_file = str(self.get_parameter("zero_offsets_file").value)
            try:
                target = load_named_positions(pose_file, "startup_pose")
                offsets = load_named_positions(offset_file, "zero_offset_pulse")
            except (OSError, ValueError, yaml.YAMLError) as error:
                self.get_logger().error(str(error))
                return 1
            reason = motion_block_reason(
                self.joint_states, target,
                float(self.get_parameter("max_joint_delta_rad").value),
                bool(self.get_parameter("allow_large_motion").value),
                use_fake_hardware,
                bool(self.get_parameter("motion_enabled").value),
                zero_offsets_uncalibrated(offsets),
            )
            if reason:
                self.get_logger().error(reason)
                return 1
            if should_arm_motion(move_to_initial, controllers_ready, use_fake_hardware):
                if not self._start_motion():
                    return 1
            duration = max(1.0, float(self.get_parameter("pose_duration_sec").value))
            if not self._command_pose(target, duration):
                return 1
            tolerance = float(self.get_parameter("pose_tolerance_rad").value)
            reached = self.wait_until(
                duration + 5.0,
                lambda: all(abs(self.joint_states.get(name, math.inf) - target[name]) <= tolerance for name in JOINTS),
            )
            if not reached:
                missed = [name for name in JOINTS if abs(self.joint_states.get(name, math.inf) - target[name]) > tolerance]
                self.get_logger().error("初始姿态未到位: " + ", ".join(missed))
                return 1
            self.get_logger().info("已到达初始姿态。控制节点保持运行，MoveIt 未启动")
            return 0

        def _start_motion(self) -> bool:
            timeout = float(self.get_parameter("check_timeout_sec").value)
            if not self.start_motion.wait_for_service(timeout_sec=timeout):
                self.get_logger().error("运动启动服务 /x1/start_motion 不可用")
                return False
            # PVT can move the hardware while the active trajectory controllers
            # still hold their pre-PVT targets. Align all controller targets to
            # measured joints before arming, so idle control cannot move back.
            if not self._command_pose(dict(self.joint_states), 1.0):
                self.get_logger().error("运动武装前的控制器保持位置对齐失败")
                return False
            future = self.start_motion.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            result = future.result() if future.done() else None
            if result is None or not result.success:
                detail = result.message if result is not None else "无响应"
                self.get_logger().error(f"未能允许运动: {detail}")
                return False
            self.get_logger().info("轨迹控制器已激活，已允许运动")
            return True

        def _return_to_urdf_zero(self) -> bool:
            """15 个关节角回到 0。这是 RViz 里的 URDF 零位，不是出厂机械零位。"""
            return self._return_to_mechanical_zero(
                "startup_pose_file", "startup_pose", "URDF 零位")

        def _return_to_mechanical_zero(
            self,
            positions_parameter: str = "initial_positions_file",
            document_key: str = "initial_positions",
            label: str = "机械零位",
        ) -> bool:
            """Move to one recorded pose with a single PVT segment."""
            import time
            from std_msgs.msg import Float64MultiArray, String

            pose_file = str(self.get_parameter(positions_parameter).value)
            try:
                target = load_named_positions(pose_file, document_key)
            except (OSError, ValueError, yaml.YAMLError) as error:
                self.get_logger().error(str(error))
                return False
            farthest = max(abs(self.joint_states.get(name, 0.0) - target[name]) for name in JOINTS)
            if farthest < 0.01:
                if label == "机械零位":
                    self.get_logger().info("已在机械零位。末端计算从这组 URDF 关节角开始")
                else:
                    self.get_logger().info(f"已在{label}")
                return True
            if farthest > 2.0:
                self.get_logger().error(f"离{label} {farthest:.3f} rad，超过 2 rad，未自动返回")
                return False
            duration = max(0.8, farthest / 0.4)
            self.pvt_result = None

            def on_result(message):
                self.pvt_result = message.data

            self.create_subscription(String, "/x1/pvt_result", on_result, 10)
            publisher = self.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if publisher.get_subscription_count() == 0:
                self.get_logger().error(f"没有 /x1/pvt_move，不能回{label}")
                return False
            message = Float64MultiArray()
            message.data = [1.0, duration, 1.0, *[target[name] for name in JOINTS]]
            publisher.publish(message)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.pvt_result:
                    continue
                parts = self.pvt_result.split(" ", 2)
                try:
                    sequence = float(parts[0])
                    accepted = abs(sequence - 1.0) < 0.1 or (
                        sequence == 0.0 and len(parts) > 1 and parts[1] == "fail")
                except (ValueError, IndexError):
                    continue
                if not accepted:
                    continue
                if len(parts) > 1 and parts[1] == "ok":
                    break
                detail = parts[2] if len(parts) > 2 else self.pvt_result
                self.get_logger().error(f"回{label}被拒绝: {detail}")
                return False
            else:
                self.get_logger().error(f"回{label}没有收到 PVT 结果")
                return False
            # PVT ok only acknowledges acceptance. Require feedback after the
            # segment duration, then check actual joints before reporting arrival.
            deadline = time.monotonic() + duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            tolerance = float(self.get_parameter("pose_tolerance_rad").value)
            if not rclpy.ok() or not self.wait_until(
                5.0,
                lambda: all(
                    abs(self.joint_states.get(name, math.inf) - target[name]) <= tolerance
                    for name in JOINTS),
            ):
                self.get_logger().error(f"回{label}未到位，停止启动流程")
                return False
            if label == "机械零位":
                self.get_logger().info(
                    f"已回到机械零位，最大行程 {farthest:.3f} rad，约 {duration:.1f} s。"
                    "URDF 使用这组关节角，末端计算从这里开始")
            else:
                self.get_logger().info(
                    f"已回到{label}，最大行程 {farthest:.3f} rad，约 {duration:.1f} s。"
                    "15 个关节角为 0")
            return True

        def _command_pose(self, target: Mapping[str, float], duration: float) -> bool:
            seconds = int(duration)
            nanosec = int(round((duration - seconds) * 1e9))
            goals = []
            for controller, names in GROUPS.items():
                client = self.action_clients[controller]
                if not client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error(f"{controller} 动作服务不可用")
                    return False
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = list(names)
                point = JointTrajectoryPoint()
                point.positions = [target[name] for name in names]
                point.time_from_start = TimeDuration(sec=seconds, nanosec=nanosec)
                goal.trajectory.points = [point]
                pending = client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, pending, timeout_sec=5.0)
                handle = pending.result() if pending.done() else None
                if handle is None or not handle.accepted:
                    self.get_logger().error(f"{controller} 拒绝初始姿态")
                    return False
                goals.append((controller, handle))
            for controller, handle in goals:
                result = handle.get_result_async()
                rclpy.spin_until_future_complete(self, result, timeout_sec=duration + 5.0)
                code = result.result().status if result.done() and result.result() is not None else None
                if code != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error(f"{controller} 轨迹执行失败，status={code}")
                    return False
            return True

    rclpy.init()
    node = Startup()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
