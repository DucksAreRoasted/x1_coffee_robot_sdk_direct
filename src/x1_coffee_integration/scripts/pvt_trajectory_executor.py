#!/usr/bin/env python3
"""把 MoveIt 的 FollowJointTrajectory 收成一段一段的 PVT。

直连主站的 /x1/pvt_move 每次只走向最后一个路点。这里按时间顺序逐点下发，
并等实测关节靠近该点再发下一点。动态 PT 不经过这条路径。
"""

import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

COFFEE_JOINTS = [
    "Joint_Body",
    "Joint_l1", "Joint_l2", "Joint_l3", "Joint_l4", "Joint_l5", "Joint_l6", "Joint_l7",
    "Joint_r1", "Joint_r2", "Joint_r3", "Joint_r4", "Joint_r5", "Joint_r6", "Joint_r7",
]
MAX_JOINT_SPEED = 0.4
MIN_SEGMENT_SEC = 0.2
ARRIVAL_TOLERANCE = 0.05


def segment_duration(previous: Sequence[float], goal: Sequence[float], planned_dt: float) -> float:
    """沿用 MoveIt 给出的时间。只有这一段比速度上限更快时才拉长。

    每个路点都再加一截最短时间的话，实际运动会比 MoveIt 预计的久。
    MoveIt 到点就取消动作，手臂就会停在半路，得点多次执行才到。
    """
    delta = 0.0
    for start, target in zip(previous, goal):
        delta = max(delta, abs(target - start))
    needed = 0.0 if delta <= 1.0e-9 else delta / MAX_JOINT_SPEED
    if planned_dt > 0.0:
        return max(planned_dt, needed)
    return max(needed, 0.05)


def full_positions(
    joint_names: Sequence[str],
    positions: Sequence[float],
    measured: Dict[str, float],
) -> List[float]:
    """轨迹里没写到的轴保持当前实测角，避免把另一条臂拉向 0。"""
    if len(joint_names) != len(positions):
        raise ValueError("关节名和位置数量不一致")
    goal = dict(measured)
    unknown = [name for name in joint_names if name not in COFFEE_JOINTS]
    if unknown:
        raise ValueError("轨迹含有未知关节: " + ",".join(unknown))
    for name, value in zip(joint_names, positions):
        goal[name] = float(value)
    missing = [name for name in COFFEE_JOINTS if name not in goal]
    if missing:
        raise ValueError("还没有这些关节的实测角: " + ",".join(missing))
    return [goal[name] for name in COFFEE_JOINTS]


def stamp_seconds(stamp) -> float:
    """轨迹点的 time_from_start 是消息里的 Duration，字段是 sec 和 nanosec。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def pvt_message(sequence: float, duration: float, positions: Sequence[float]) -> Float64MultiArray:
    message = Float64MultiArray()
    message.data = [float(sequence), float(duration), 1.0, *[float(item) for item in positions]]
    return message


def pvt_path_message(
    sequence: float,
    segment_sec: Sequence[float],
    rows: Sequence[Sequence[float]],
) -> Float64MultiArray:
    """整段轨迹一次下发。每个路点前面是这一段的秒数，15 轴按同一时刻插补。"""
    if len(segment_sec) != len(rows):
        raise ValueError("段时间和路点数量不一致")
    data = [float(sequence), float(sum(segment_sec)), float(len(rows))]
    for duration, row in zip(segment_sec, rows):
        data.append(float(duration))
        data.extend(float(item) for item in row)
    message = Float64MultiArray()
    message.data = data
    return message


class PvtTrajectoryExecutor(Node):
    def __init__(self) -> None:
        super().__init__("pvt_trajectory_executor")
        self._lock = threading.Lock()
        self._measured: Dict[str, float] = {}
        self._pvt_result: Optional[str] = None
        self._sequence = 0
        self._busy = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos, callback_group=group)
        # /x1/pvt_result 是普通可靠发布。订阅改成持久化会和它不兼容，结果一条也收不到。
        self.create_subscription(String, "/x1/pvt_result", self._on_result, qos, callback_group=group)
        self._pvt_pub = self.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
        self._action = ActionServer(
            self,
            FollowJointTrajectory,
            "/pvt_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=group,
        )

    def _on_joints(self, message: JointState) -> None:
        with self._lock:
            for name, position in zip(message.name, message.position):
                self._measured[name] = float(position)

    def _on_result(self, message: String) -> None:
        with self._lock:
            self._pvt_result = message.data

    def _snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._measured)

    def _goal(self, _goal) -> GoalResponse:
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _wait_result(self, sequence: float) -> Tuple[bool, str]:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self._lock:
                result = self._pvt_result
            if result:
                parts = result.split(" ", 2)
                try:
                    got = float(parts[0])
                except (ValueError, IndexError):
                    got = -1.0
                if abs(got - sequence) <= 0.1:
                    if len(parts) > 1 and parts[1] == "ok":
                        return True, ""
                    detail = parts[2] if len(parts) > 2 else result
                    return False, detail
            time.sleep(0.02)
        return False, "没有收到 PVT 结果"

    def _publish_and_wait(self, positions: Sequence[float], duration: float) -> Tuple[bool, str]:
        with self._lock:
            self._sequence += 1
            sequence = float(self._sequence)
            self._pvt_result = None
        self._pvt_pub.publish(pvt_message(sequence, duration, positions))
        return self._wait_result(sequence)

    def _wait_arrival(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        duration: float,
        goal_handle,
    ) -> Tuple[bool, str]:
        targets = dict(zip(joint_names, positions))
        deadline = time.monotonic() + duration + 2.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "已取消"
            measured = self._snapshot()
            if targets and all(
                name in measured and abs(measured[name] - target) <= ARRIVAL_TOLERANCE
                for name, target in targets.items()
            ):
                return True, ""
            time.sleep(0.05)
        return False, "关节没有在时限内靠近路点"

    def _hold_here(self) -> None:
        measured = self._snapshot()
        if any(name not in measured for name in COFFEE_JOINTS):
            return
        positions = [measured[name] for name in COFFEE_JOINTS]
        self._publish_and_wait(positions, MIN_SEGMENT_SEC)

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        try:
            if not trajectory.points:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            # 起点用实测角。轨迹没覆盖的轴保持不动。
            measured = self._snapshot()
            previous = full_positions(names, [
                measured.get(name, trajectory.points[0].positions[index])
                for index, name in enumerate(names)
            ], measured)
            rows = []
            durations = []
            last_time = 0.0
            for point in trajectory.points:
                # 没出现在这条轨迹里的轴锁在起点，避免中途用新的实测角把它们带跑。
                goal = full_positions(names, point.positions, measured)
                planned_dt = max(0.0, stamp_seconds(point.time_from_start) - last_time)
                last_time = stamp_seconds(point.time_from_start)
                durations.append(segment_duration(previous, goal, planned_dt))
                rows.append(goal)
                previous = goal
            with self._lock:
                self._sequence += 1
                sequence = float(self._sequence)
                self._pvt_result = None
            self._pvt_pub.publish(pvt_path_message(sequence, durations, rows))
            accepted, detail = self._wait_result(sequence)
            if not accepted:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = detail
                return result
            if goal_handle.is_cancel_requested:
                self._hold_here()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "已取消，并按当前实测角停下"
                return result
            arrived, detail = self._wait_arrival(
                names, trajectory.points[-1].positions, sum(durations), goal_handle)
            if not arrived:
                self._hold_here()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = detail
                return result
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        except ValueError as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = str(exc)
            return result
        finally:
            with self._lock:
                self._busy = False


def main() -> None:
    rclpy.init()
    node = PvtTrajectoryExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
