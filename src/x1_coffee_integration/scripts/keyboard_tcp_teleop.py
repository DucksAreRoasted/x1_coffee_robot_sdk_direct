#!/usr/bin/env python3
"""Keyboard jogging of the left or right gripper TCP.

Each key adds 5 mm or 0.03 rad in the fixed root frame, starting from the
current measured gripper pose, then sends one profile-position target on
/x1/pvt_move. The direct EtherCAT master writes 0x607A. This script does not
enable drives and does not release brakes.
"""

import math
import os
import select
import sys
import termios
import time
import tty
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

Vector = List[float]
Matrix = List[List[float]]

LEFT_JOINTS = [f"Joint_l{index}" for index in range(1, 8)]
RIGHT_JOINTS = [f"Joint_r{index}" for index in range(1, 8)]
COFFEE_JOINTS = ["Joint_Body", *LEFT_JOINTS, *RIGHT_JOINTS]
ARMS = {
    "left": ("left_group_controller", LEFT_JOINTS, "left_gripper_tcp"),
    "right": ("right_group_controller", RIGHT_JOINTS, "right_gripper_tcp"),
}
LIMITS = {
    "Joint_Body": (-6.2831852, 6.2831852),
    "Joint_l1": (-3.1415926, 3.1415926),
    "Joint_l2": (-0.15, 3.1415926),
    "Joint_l3": (-3.1415926, 3.1415926),
    "Joint_l4": (-1.5707963, 2.0943951),
    "Joint_l5": (-3.1415926, 3.1415926),
    "Joint_l6": (-1.7453292, 1.7453292),
    "Joint_l7": (-6.2831853, 6.2831853),
    "Joint_r1": (-3.1415926, 3.1415926),
    "Joint_r2": (-3.1415926, 0.15),
    "Joint_r3": (-3.1415926, 3.1415926),
    "Joint_r4": (-2.0943951, 1.5707963),
    "Joint_r5": (-3.1415926, 3.1415926),
    "Joint_r6": (-1.7453292, 1.7453292),
    "Joint_r7": (-6.2831853, 6.2831853),
}

STEP_METERS = 0.005
STEP_RADIANS = 0.03
# 平移方向和某一关节的位移至少这么对齐时，只转该关节，避免和其他关节对消。
DIRECT_PURITY = 0.85
DIRECT_OFF_AXIS = 0.002
DIRECT_ROTATION = 0.02
WAIST_STEP = 0.02
MAX_JOINT_STEP = 0.10
# 零位附近抬高末端必须同时弯肘和腕，单步大约 0.2 rad。
COUPLED_JOINT_STEP = 0.28
# 6、7 关节可以大幅转动来解开抬升和俯仰的耦合。
WRIST_JOINT_STEP = 1.2
SEGMENT_SEC = 0.8
# 轨迹控制器用五次样条时，峰值速度大约是平均速度的 1.875 倍。
# 平均速度压到 0.4 rad/s，峰值约 0.75，低于硬件 1 rad/s 的保护。
MAX_JOINT_VELOCITY = 0.4
POSITION_TOLERANCE = 0.0004
ROTATION_TOLERANCE = 0.02
# 平移时姿态误差超过约 0.3° 就放弃这一步，避免手腕为了凑位置而转动。
ORIENTATION_HOLD = 0.005
TRANSLATION_SAMPLES = 5


def _zeros(rows: int, cols: int) -> Matrix:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def _matvec(matrix: Matrix, vector: Vector) -> Vector:
    return [sum(row[col] * vector[col] for col in range(len(vector))) for row in matrix]


def _transpose(matrix: Matrix) -> Matrix:
    return [list(column) for column in zip(*matrix)]


def _mul(left: Matrix, right: Matrix) -> Matrix:
    return [
        [sum(left[row][k] * right[k][col] for k in range(len(right))) for col in range(len(right[0]))]
        for row in range(len(left))
    ]


def _rpy(roll: float, pitch: float, yaw: float) -> Matrix:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rot_axis(axis: Sequence[float], angle: float) -> Matrix:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return [
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ]


def _identity() -> Matrix:
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _transform(xyz: Sequence[float], rotation: Matrix) -> Matrix:
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_of(transform: Matrix) -> Matrix:
    return [row[:3] for row in transform[:3]]


def _origin_of(transform: Matrix) -> Vector:
    return [transform[0][3], transform[1][3], transform[2][3]]


def _vee(rotation: Matrix) -> Vector:
    return [
        0.5 * (rotation[2][1] - rotation[1][2]),
        0.5 * (rotation[0][2] - rotation[2][0]),
        0.5 * (rotation[1][0] - rotation[0][1]),
    ]


def _solve(matrix: Matrix, target: Vector) -> Optional[Vector]:
    size = len(target)
    rows = [matrix[row][:] + [target[row]] for row in range(size)]
    for col in range(size):
        pivot = max(range(col, size), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-10:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        for col_index in range(col, size + 1):
            rows[col][col_index] /= scale
        for row in range(size):
            if row == col:
                continue
            factor = rows[row][col]
            for col_index in range(col, size + 1):
                rows[row][col_index] -= factor * rows[col][col_index]
    return [rows[row][size] for row in range(size)]


class Joint:
    def __init__(self, name: str, joint_type: str, xyz: Vector, rpy: Vector, axis: Vector) -> None:
        self.name = name
        self.joint_type = joint_type
        self.xyz = xyz
        self.rpy = rpy
        self.axis = axis


class ArmChain:
    def __init__(self, joints: Sequence[Joint], actuated: Sequence[str]) -> None:
        self.joints = list(joints)
        self.actuated = list(actuated)

    def fk(self, positions: Dict[str, float]) -> Matrix:
        transform = _identity()
        for joint in self.joints:
            rotation = _rpy(*joint.rpy)
            origin = _transform(joint.xyz, rotation)
            if joint.joint_type == "revolute":
                origin = _mul(origin, _transform([0.0, 0.0, 0.0], _rot_axis(joint.axis, positions.get(joint.name, 0.0))))
            transform = _mul(transform, origin)
        return transform

    def jacobian(self, positions: Dict[str, float]) -> Matrix:
        base = self.fk(positions)
        base_position = _origin_of(base)
        base_rotation = _rotation_of(base)
        columns = []
        for name in self.actuated:
            nudged = dict(positions)
            nudged[name] = positions.get(name, 0.0) + 1e-6
            moved = self.fk(nudged)
            delta_position = [(_origin_of(moved)[axis] - base_position[axis]) / 1e-6 for axis in range(3)]
            delta_rotation = _mul(_rotation_of(moved), _transpose(base_rotation))
            delta_angle = [component / 1e-6 for component in _vee(delta_rotation)]
            columns.append(delta_position + delta_angle)
        return _transpose(columns)

    def step(
        self,
        positions: Dict[str, float],
        linear: Vector,
        angular: Vector,
    ) -> Optional[Dict[str, float]]:
        current = self.fk(positions)
        current_rotation = _rotation_of(current)
        current_position = _origin_of(current)
        # 当前 TCP 在 URDF 根坐标系 dummy_root 里的位置，增量也加在这个全局坐标系上。
        target_position = [current_position[axis] + linear[axis] for axis in range(3)]
        target_rotation = _mul(_rpy(*angular), current_rotation)
        return self._reach(positions, target_position, target_rotation, ROTATION_TOLERANCE)

    def move_line(
        self,
        positions: Dict[str, float],
        linear: Vector,
    ) -> Optional[List[Dict[str, float]]]:
        """Straight global move. Every sample keeps the orientation at the start of the key."""
        current = self.fk(positions)
        start_position = _origin_of(current)
        start_rotation = _rotation_of(current)
        samples: List[Dict[str, float]] = []
        seed = dict(positions)
        for index in range(1, TRANSLATION_SAMPLES + 1):
            scale = index / TRANSLATION_SAMPLES
            target_position = [start_position[axis] + linear[axis] * scale for axis in range(3)]
            solved = self._reach(seed, target_position, start_rotation, ORIENTATION_HOLD)
            if solved is None:
                return None
            displacement = _world_displacement(self, positions, solved)
            if not _axis_aligned(displacement, linear, scale):
                return None
            samples.append(solved)
            seed.update(solved)
        if any(abs(samples[-1][name] - positions.get(name, 0.0)) > MAX_JOINT_STEP for name in self.actuated):
            return None
        return samples

    def _reach(
        self,
        positions: Dict[str, float],
        target_position: Vector,
        target_rotation: Matrix,
        rotation_tolerance: float,
    ) -> Optional[Dict[str, float]]:
        candidate = dict(positions)
        for _ in range(40):
            pose = self.fk(candidate)
            error = [target_position[axis] - _origin_of(pose)[axis] for axis in range(3)]
            error.extend(_vee(_mul(target_rotation, _transpose(_rotation_of(pose)))))
            if (
                math.sqrt(sum(item * item for item in error[:3])) < POSITION_TOLERANCE
                and math.sqrt(sum(item * item for item in error[3:])) < rotation_tolerance
            ):
                break
            jacobian = self.jacobian(candidate)
            rows = len(jacobian)
            normal = _zeros(rows, rows)
            for row in range(rows):
                for col in range(rows):
                    normal[row][col] = sum(
                        jacobian[row][joint] * jacobian[col][joint] for joint in range(len(jacobian[0]))
                    )
                normal[row][row] += 0.02 * 0.02
            task = _solve(normal, error)
            if task is None:
                return None
            delta = _matvec(_transpose(jacobian), task)
            for index, name in enumerate(self.actuated):
                lower, upper = LIMITS[name]
                candidate[name] = min(upper, max(lower, candidate.get(name, 0.0) + delta[index]))
        pose = self.fk(candidate)
        position_error = math.sqrt(sum((target_position[axis] - _origin_of(pose)[axis]) ** 2 for axis in range(3)))
        rotation_error = math.sqrt(sum(
            component * component
            for component in _vee(_mul(target_rotation, _transpose(_rotation_of(pose))))
        ))
        if position_error > POSITION_TOLERANCE or rotation_error > rotation_tolerance:
            return None
        if any(abs(candidate[name] - positions.get(name, 0.0)) > MAX_JOINT_STEP for name in self.actuated):
            return None
        return {name: candidate[name] for name in self.actuated}


def _dls_towards(
    chain: ArmChain,
    start: Dict[str, float],
    target_position: Vector,
    target_rotation: Matrix,
    iterations: int = 25,
) -> Optional[Dict[str, float]]:
    """Damped least squares without the small-step limit used by keyboard jogs."""
    candidate = dict(start)
    for _ in range(iterations):
        pose = chain.fk(candidate)
        error = [target_position[axis] - _origin_of(pose)[axis] for axis in range(3)]
        error.extend(_vee(_mul(target_rotation, _transpose(_rotation_of(pose)))))
        if (
            math.sqrt(sum(item * item for item in error[:3])) < POSITION_TOLERANCE
            and math.sqrt(sum(item * item for item in error[3:])) < ORIENTATION_HOLD
        ):
            return {name: candidate[name] for name in chain.actuated}
        jacobian = chain.jacobian(candidate)
        rows = len(jacobian)
        normal = _zeros(rows, rows)
        for row in range(rows):
            for col in range(rows):
                normal[row][col] = sum(
                    jacobian[row][joint] * jacobian[col][joint] for joint in range(len(jacobian[0]))
                )
            normal[row][row] += 0.02 * 0.02
        task = _solve(normal, error)
        if task is None:
            return None
        delta = _matvec(_transpose(jacobian), task)
        peak = max(abs(item) for item in delta)
        if peak > 0.08:
            delta = [item * 0.08 / peak for item in delta]
        for index, name in enumerate(chain.actuated):
            lower, upper = LIMITS[name]
            candidate[name] = min(upper, max(lower, candidate.get(name, 0.0) + delta[index]))
    pose = chain.fk(candidate)
    position_error = math.sqrt(sum((target_position[axis] - _origin_of(pose)[axis]) ** 2 for axis in range(3)))
    rotation_error = math.sqrt(sum(
        component * component
        for component in _vee(_mul(target_rotation, _transpose(_rotation_of(pose))))
    ))
    if position_error > POSITION_TOLERANCE or rotation_error > ORIENTATION_HOLD:
        return None
    return {name: candidate[name] for name in chain.actuated}


def _off_axis(actual: Vector, linear: Vector) -> float:
    """Distance from the requested axis. Progress along that axis is ignored."""
    error = [actual[axis] - linear[axis] for axis in range(3)]
    norm = sum(item * item for item in linear)
    if norm > 1.0e-12:
        scale = sum(error[axis] * linear[axis] for axis in range(3)) / norm
        error = [error[axis] - scale * linear[axis] for axis in range(3)]
    return math.sqrt(sum(item * item for item in error))


def _joint_lerp_stays_on_axis(
    chain: ArmChain,
    start: Dict[str, float],
    goal: Dict[str, float],
    linear: Vector,
) -> bool:
    """The card interpolates joints, so that path has to stay near the requested line."""
    start_pose = chain.fk(start)
    start_position = _origin_of(start_pose)
    start_rotation = _rotation_of(start_pose)
    for index in range(1, 9):
        scale = index / 8
        sample = {
            name: start.get(name, 0.0) + (goal[name] - start.get(name, 0.0)) * scale
            for name in chain.actuated
        }
        pose = chain.fk({**start, **sample})
        actual = [_origin_of(pose)[axis] - start_position[axis] for axis in range(3)]
        if _off_axis(actual, [item * scale for item in linear]) > 0.002:
            return False
        rotation_error = math.sqrt(sum(
            component * component
            for component in _vee(_mul(start_rotation, _transpose(_rotation_of(pose))))
        ))
        if rotation_error > 0.05:
            return False
    return True


def coupled_translation(
    chain: ArmChain,
    positions: Dict[str, float],
    linear: Vector,
) -> Optional[Dict[str, float]]:
    """Reach a translation when shoulder pitch and lift are the same joint motion.

    Bending the elbow and wrist makes those two directions independent. The
    endpoint keeps the original orientation. A solution is kept only when the
    joint interpolation stays within 2 mm of the requested axis.
    """
    current = chain.fk(positions)
    target_position = [_origin_of(current)[axis] + linear[axis] for axis in range(3)]
    target_rotation = _rotation_of(current)
    indexes = (1, 2, 4, 5)
    pattern = (1.0, -1.0, 1.0, -1.0)
    seeds = [dict(positions)]
    for scale in (0.16, 0.22):
        for flip in (1.0, -1.0):
            nudged = dict(positions)
            for slot, index in enumerate(indexes):
                name = chain.actuated[index]
                lower, upper = LIMITS[name]
                nudged[name] = min(
                    upper, max(lower, positions.get(name, 0.0) + flip * pattern[slot] * scale)
                )
            seeds.append(nudged)
    for wrist in (0.8, -0.8, 1.2, -1.2):
        nudged = dict(positions)
        for index in (5, 6):
            name = chain.actuated[index]
            lower, upper = LIMITS[name]
            nudged[name] = min(upper, max(lower, positions.get(name, 0.0) + wrist))
        seeds.append(nudged)
    best = None
    best_rank = (COUPLED_JOINT_STEP + 1.0, WRIST_JOINT_STEP + 1.0)
    for seed in seeds:
        solved = _dls_towards(chain, seed, target_position, target_rotation)
        if solved is None:
            continue
        proximal = 0.0
        wrist_motion = 0.0
        allowed = True
        for name in chain.actuated:
            delta = abs(solved[name] - positions.get(name, 0.0))
            if name.endswith(("6", "7")):
                wrist_motion = max(wrist_motion, delta)
                if delta > WRIST_JOINT_STEP:
                    allowed = False
            else:
                proximal = max(proximal, delta)
                if delta > COUPLED_JOINT_STEP:
                    allowed = False
        if not allowed or (proximal, wrist_motion) >= best_rank:
            continue
        if not _joint_lerp_stays_on_axis(chain, positions, solved, linear):
            continue
        best = solved
        best_rank = (proximal, wrist_motion)
    return best


def _axis_aligned(displacement: Vector, linear: Vector, scale: float) -> bool:
    """True when the move stays on the requested base axis. Off-axis motion stays under 0.5 mm."""
    for axis in range(3):
        expected = linear[axis] * scale
        error = displacement[axis] - expected
        if abs(error) > 0.0005:
            return False
    return True


def direct_translation(
    chain: ArmChain,
    positions: Dict[str, float],
    linear: Vector,
) -> Optional[Dict[str, float]]:
    """Move one joint when its Cartesian direction already matches the key.

    At the mechanical pose, joint 4 lifts and joint 3 moves sideways. Combining
    them with the shoulder cancels that motion on the real arm.
    """
    length = math.sqrt(sum(component * component for component in linear))
    if length < 1.0e-9:
        return None
    direction = [component / length for component in linear]
    jacobian = chain.jacobian(positions)
    best_align = 0.0
    best_name = None
    for index, name in enumerate(chain.actuated):
        column = [jacobian[row][index] for row in range(3)]
        magnitude = math.sqrt(sum(component * component for component in column))
        if magnitude < 0.05:
            continue
        align = sum(column[axis] * direction[axis] for axis in range(3))
        if abs(align) / magnitude < DIRECT_PURITY or abs(align) <= abs(best_align):
            continue
        best_align = align
        best_name = name
    if best_name is None:
        return None
    delta = length / best_align
    if abs(delta) > MAX_JOINT_STEP:
        return None
    lower, upper = LIMITS[best_name]
    current = positions.get(best_name, 0.0)
    target = min(upper, max(lower, current + delta))
    if abs(target - (current + delta)) > 1.0e-6:
        return None
    goal = dict(positions)
    goal[best_name] = target
    displacement = _world_displacement(chain, positions, goal)
    progress = sum(displacement[axis] * direction[axis] for axis in range(3))
    if abs(progress - length) > 0.0015 or _off_axis(displacement, linear) > DIRECT_OFF_AXIS:
        return None
    start_rotation = _rotation_of(chain.fk(positions))
    end_rotation = _rotation_of(chain.fk(goal))
    rotation_error = math.sqrt(sum(
        component * component
        for component in _vee(_mul(end_rotation, _transpose(start_rotation)))
    ))
    if rotation_error > DIRECT_ROTATION:
        return None
    return {name: goal[name] for name in chain.actuated}


def _world_displacement(chain: ArmChain, start: Dict[str, float], goal: Dict[str, float]) -> Vector:
    """TCP translation from start to goal in the URDF root frame dummy_root."""
    start_pose = chain.fk(start)
    end_pose = chain.fk({**start, **goal})
    return [_origin_of(end_pose)[axis] - _origin_of(start_pose)[axis] for axis in range(3)]


def segment_seconds(
    start: Dict[str, float],
    names: Sequence[str],
    goal: Dict[str, float],
    previous: Optional[Dict[str, float]] = None,
) -> float:
    """Stretch the segment so average joint speed stays at or below MAX_JOINT_VELOCITY.

    previous is the last goal already given to the trajectory controller. A new goal
    can be close to the measured arm and still far from that command.
    """
    delta = 0.0
    for name in names:
        goal_position = goal.get(name, 0.0)
        delta = max(delta, abs(goal_position - start.get(name, 0.0)))
        if previous is not None:
            delta = max(delta, abs(goal_position - previous.get(name, goal_position)))
    if delta <= 1.0e-9 or MAX_JOINT_VELOCITY <= 0.0:
        return SEGMENT_SEC
    return max(SEGMENT_SEC, delta / MAX_JOINT_VELOCITY)


def _numbers(text: Optional[str], fallback: Sequence[float]) -> Vector:
    if not text:
        return list(fallback)
    return [float(item) for item in text.split()]


def load_chain(urdf_path: str, tip_link: str, actuated: Sequence[str]) -> ArmChain:
    root = ET.parse(urdf_path).getroot()
    joints = {}
    parent_of = {}
    incoming = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        joints[name] = joint
        parent_of[child] = parent
        incoming[child] = name
    ordered = []
    link = tip_link
    while link != "dummy_root":
        joint = joints[incoming[link]]
        ordered.append(Joint(
            joint.get("name"),
            joint.get("type"),
            _numbers(None if joint.find("origin") is None else joint.find("origin").get("xyz"), (0.0, 0.0, 0.0)),
            _numbers(None if joint.find("origin") is None else joint.find("origin").get("rpy"), (0.0, 0.0, 0.0)),
            _numbers(None if joint.find("axis") is None else joint.find("axis").get("xyz"), (0.0, 0.0, 1.0)),
        ))
        link = parent_of[link]
    ordered.reverse()
    return ArmChain(ordered, actuated)


def urdf_zero_pose() -> Dict[str, float]:
    """Every joint at the URDF geometric zero."""
    return {name: 0.0 for name in COFFEE_JOINTS}


def mechanical_pose() -> Dict[str, float]:
    """URDF joint angles at the factory encoder pulses. That pose is the mechanical zero."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("x1_coffee_integration"), "config", "initial_positions.yaml"))
    except Exception:
        pass
    candidates.append(os.path.join(os.path.dirname(__file__), "..", "config", "initial_positions.yaml"))
    for path in candidates:
        if not os.path.isfile(path):
            continue
        import yaml
        values = (yaml.safe_load(open(path, encoding="utf-8")) or {}).get("initial_positions") or {}
        if all(name in values for name in COFFEE_JOINTS):
            return {name: float(values[name]) for name in COFFEE_JOINTS}
    return {name: 0.0 for name in COFFEE_JOINTS}


def default_urdf() -> str:
    override = os.environ.get("X1_URDF")
    if override:
        return override
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("arm_describle"), "urdf", "HRB3Q-LC.urdf")
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), "..", "..", "arm_describle", "urdf", "HRB3Q-LC.urdf")


def self_check(urdf_path: str) -> int:
    chain = load_chain(urdf_path, "left_gripper_tcp", LEFT_JOINTS)
    seed = {
        "Joint_l1": 0.4,
        "Joint_l2": 0.6,
        "Joint_l3": 0.3,
        "Joint_l4": 0.8,
        "Joint_l5": -0.4,
        "Joint_l6": 0.5,
        "Joint_l7": 0.2,
    }
    samples = chain.move_line(seed, [0.0, 0.0, STEP_METERS])
    if samples is None:
        print("self-check failed: 5 mm step was rejected")
        return 1
    start_rotation = _rotation_of(chain.fk(seed))
    for sample in samples:
        pose = chain.fk({**seed, **sample})
        rotation_error = math.sqrt(sum(
            component * component
            for component in _vee(_mul(start_rotation, _transpose(_rotation_of(pose))))
        ))
        if rotation_error > ORIENTATION_HOLD:
            print(f"self-check failed: translation changed orientation by {rotation_error} rad")
            return 1
    world_delta = _world_displacement(chain, seed, samples[-1])
    if abs(world_delta[2] - STEP_METERS) > 0.002 or max(abs(world_delta[0]), abs(world_delta[1])) > 0.002:
        print(f"self-check failed: world-frame step was {world_delta}, expected Z {STEP_METERS}")
        return 1
    turned = dict(seed)
    turned["Joint_Body"] = 1.0
    turned_move = chain.step(turned, [STEP_METERS, 0.0, 0.0], [0.0, 0.0, 0.0])
    if turned_move is None:
        print("self-check failed: world X step with the waist turned was rejected")
        return 1
    turned_delta = _world_displacement(chain, turned, turned_move)
    if abs(turned_delta[0] - STEP_METERS) > 0.002 or max(abs(turned_delta[1]), abs(turned_delta[2])) > 0.002:
        print(f"self-check failed: waist-turned world X step was {turned_delta}")
        return 1
    second_seed = {**seed, **samples[-1]}
    second = chain.move_line(second_seed, [0.0, 0.0, STEP_METERS])
    if second is None:
        print("self-check failed: second step from the new pose was rejected")
        return 1
    second_delta = _world_displacement(chain, second_seed, second[-1])
    if abs(second_delta[2] - STEP_METERS) > 0.002:
        print(f"self-check failed: second world-frame step was {second_delta}")
        return 1
    if chain.step(seed, [0.5, 0.0, 0.0], [0.0, 0.0, 0.0]) is not None:
        print("self-check failed: a 0.5 m step was accepted")
        return 1
    duration = segment_seconds({"Joint_l1": 0.0}, ["Joint_l1"], {"Joint_l1": 0.64})
    if duration < 0.64 / MAX_JOINT_VELOCITY:
        print(f"self-check failed: 0.64 rad segment is {duration} s")
        return 1
    home_duration = segment_seconds({"Joint_l4": 2.0}, COFFEE_JOINTS, {name: 0.0 for name in COFFEE_JOINTS})
    if home_duration < 2.0 / MAX_JOINT_VELOCITY:
        print(f"self-check failed: home segment is {home_duration} s")
        return 1
    zero_chain = load_chain(urdf_path, "left_gripper_tcp", LEFT_JOINTS)
    zero_pose = {"Joint_Body": 0.0}
    lifted = coupled_translation(zero_chain, zero_pose, [0.0, 0.0, STEP_METERS])
    if lifted is None:
        print("self-check failed: coupled vertical step at URDF zero was rejected")
        return 1
    lifted_delta = _world_displacement(zero_chain, zero_pose, lifted)
    if abs(lifted_delta[2] - STEP_METERS) > 0.001 or max(abs(lifted_delta[0]), abs(lifted_delta[1])) > 0.001:
        print(f"self-check failed: coupled vertical step was {lifted_delta}")
        return 1
    lifted_rotation = math.sqrt(sum(
        component * component
        for component in _vee(_mul(
            _rotation_of(zero_chain.fk(zero_pose)),
            _transpose(_rotation_of(zero_chain.fk({**zero_pose, **lifted}))),
        ))
    ))
    if lifted_rotation > ORIENTATION_HOLD:
        print(f"self-check failed: coupled step changed orientation by {lifted_rotation} rad")
        return 1
    right_chain = load_chain(urdf_path, "right_gripper_tcp", RIGHT_JOINTS)
    lowered = coupled_translation(right_chain, zero_pose, [0.0, 0.0, -STEP_METERS])
    if lowered is None:
        print("self-check failed: right arm coupled downward step was rejected")
        return 1
    mechanical = {
        "Joint_Body": -0.279592,
        "Joint_l1": 0.980322,
        "Joint_l2": 0.010067,
        "Joint_l3": 0.010175,
        "Joint_l4": -0.190022,
        "Joint_l5": -1.620459,
        "Joint_l6": -1.189866,
        "Joint_l7": 1.630046,
        "Joint_r1": -1.000191,
        "Joint_r2": -0.000144,
        "Joint_r3": -0.040147,
        "Joint_r4": 0.180135,
        "Joint_r5": 1.630658,
        "Joint_r6": 1.179811,
        "Joint_r7": 1.489615,
    }
    for tip, joints, vertical, lateral in (
        ("left_gripper_tcp", LEFT_JOINTS, [0.0, 0.0, STEP_METERS], [0.0, STEP_METERS, 0.0]),
        ("right_gripper_tcp", RIGHT_JOINTS, [0.0, 0.0, STEP_METERS], [0.0, STEP_METERS, 0.0]),
    ):
        arm = load_chain(urdf_path, tip, joints)
        for linear in (vertical, lateral, [-STEP_METERS, 0.0, 0.0]):
            samples = arm.move_line(mechanical, linear)
            if samples is None:
                print(f"self-check failed: translation {linear} was rejected")
                return 1
            displacement = _world_displacement(arm, mechanical, samples[-1])
            if any(abs(displacement[axis] - linear[axis]) > 0.001 for axis in range(3)):
                print(f"self-check failed: translation landed at {displacement}")
                return 1
            rotation_error = math.sqrt(sum(
                component * component
                for component in _vee(_mul(
                    _rotation_of(arm.fk(mechanical)),
                    _transpose(_rotation_of(arm.fk({**mechanical, **samples[-1]}))),
                ))
            ))
            if rotation_error > ORIENTATION_HOLD:
                print(f"self-check failed: translation changed orientation by {rotation_error} rad")
                return 1
    print("keyboard tcp self-check passed")
    return 0


HELP = """
末端键盘。先用 URDF 计算当前 TCP 在 dummy_root 里的全局坐标，再在这个坐标系里沿直线加 5 mm，并保持这一步开始时的姿态。
  1/2     左臂 / 右臂
  w/s     全局 X 正 / 负
  a/d     全局 Y 正 / 负
  q/e     全局 Z 正 / 负
  i/k j/l u/o   绕全局 X / Y / Z
  [ / ]   腰关节小步转动
  点动    一次把 15 轴目标写成轮廓位置。驱动器用已配置的轮廓速度走向新的 0x607A
  h h     连按两次：回到机械零位（出厂脉冲对应的 URDF 角度，不是全零）
  0 0     连按两次：回到 URDF 零位（15 个关节角全部为 0）
  z z     连按两次：请求零力矩。进 OP 之后不能改过程数据映射，这一步不会松开抱闸
  m m     连按两次：MIT 保持当前位置。关节变软，请扶着
  n       退出 MIT，回到位置保持
  b       关断并重新抱闸
  空格    停在当前目标，不再追加
  Esc     退出
手臂要先上电。控制栈这样启动后，再开这个程序：
  ros2 launch x1_coffee_integration robot_startup.launch.py motion_enabled:=false drives_enabled:=true ethercat_ifname:=eno1
"""


def run_teleop() -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
    from std_srvs.srv import Trigger

    urdf_path = default_urdf()
    chains = {
        side: load_chain(urdf_path, tip, joints)
        for side, (_, joints, tip) in ARMS.items()
    }

    class Teleop(Node):
        def __init__(self) -> None:
            super().__init__("keyboard_tcp_teleop")
            self.measured: Dict[str, float] = {}
            self.commanded: Dict[str, float] = {}
            self.side = "left"
            self.warned = False
            self.motion_requested = False
            self.zero_torque = False
            self.mit_hold = False
            self.zero_confirm_until = 0.0
            self.mit_confirm_until = 0.0
            self.home_confirm_until = 0.0
            self.urdf_confirm_until = 0.0
            self.last_segment_sec = SEGMENT_SEC
            self.pvt_seq = 0
            self.pvt_result = None
            self.joint_sub = None
            self.joint_qos_logged = False
            self.create_subscription(String, "/x1/pvt_result", self._on_pvt_result, 10)
            self.pvt_pub = self.create_publisher(Float64MultiArray, "/x1/pvt_move", 10)
            self.start_motion = self.create_client(Trigger, "/x1/start_motion")
            self.zero_torque_client = self.create_client(Trigger, "/x1/zero_torque")
            self.brake_client = self.create_client(Trigger, "/x1/brake_hold")
            self.mit_hold_client = self.create_client(Trigger, "/x1/mit_hold")
            self.mit_exit_client = self.create_client(Trigger, "/x1/mit_exit")
            self.trajectory_clients = {}

        def attach_joint_states(self) -> None:
            if self.joint_sub is not None:
                return
            infos = self.get_publishers_info_by_topic("/joint_states")
            if not infos:
                return
            qos = infos[0].qos_profile
            if not self.joint_qos_logged:
                names = ",".join(item.node_name for item in infos)
                print(
                    f"[x1-debug] /joint_states publisher={names} "
                    f"reliability={qos.reliability} durability={qos.durability} depth={qos.depth}",
                    flush=True,
                )
                self.joint_qos_logged = True
            from rclpy.qos import HistoryPolicy, QoSProfile
            # 发现到的发布者 QoS 里 history 是 UNKNOWN、depth 为 0，原样拷贝会创建订阅失败。
            # 只沿用可靠性与持久性，历史策略固定 KEEP_LAST。
            profile = QoSProfile(
                reliability=qos.reliability,
                durability=qos.durability,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.joint_sub = self.create_subscription(
                JointState, "/joint_states", self._on_joints, profile)

        def _on_joints(self, message: JointState) -> None:
            for name, position in zip(message.name, message.position):
                self.measured[name] = float(position)

        def ready(self) -> bool:
            needed = LEFT_JOINTS + RIGHT_JOINTS + ["Joint_Body"]
            return all(name in self.measured for name in needed)

        def arm_motion(self) -> bool:
            if not self.start_motion.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("没有 /x1/start_motion。请先启动 robot_startup.launch.py。")
                return False
            future = self.start_motion.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result() if future.done() else None
            if result is None or not result.success:
                detail = result.message if result is not None else "无响应"
                self.get_logger().error(f"未能允许运动: {detail}")
                return False
            self.get_logger().info("已调用 /x1/start_motion")
            return True

        def call_trigger(self, client, label: str) -> bool:
            if not client.wait_for_service(timeout_sec=3.0):
                print(f"\n没有 {label}")
                return False
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result() if future.done() else None
            if result is None or not result.success:
                detail = result.message if result is not None else "无响应"
                print(f"\n{label} 失败: {detail}")
                return False
            print(f"\n{result.message}")
            return True

        def _on_pvt_result(self, message: String) -> None:
            self.pvt_result = message.data

        def wait_controllers(self) -> bool:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.pvt_pub.get_subscription_count() > 0:
                    return True
            self.get_logger().error("没有 /x1/pvt_move。请先启动 robot_startup.launch.py。")
            return False

        def send(
            self,
            _controller_key: str,
            names: Sequence[str],
            waypoints: Sequence[Dict[str, float]],
        ) -> bool:
            if not waypoints:
                return False
            rows = []
            for positions in waypoints:
                goal = {
                    name: self.measured.get(name, self.commanded.get(name, 0.0))
                    for name in COFFEE_JOINTS
                }
                for name in names:
                    goal[name] = positions[name]
                rows.append([goal[name] for name in COFFEE_JOINTS])
            start = {name: self.measured.get(name, rows[-1][index]) for index, name in enumerate(COFFEE_JOINTS)}
            final_goal = {name: rows[-1][index] for index, name in enumerate(COFFEE_JOINTS)}
            duration = segment_seconds(start, COFFEE_JOINTS, final_goal, self.commanded)
            self.last_segment_sec = duration
            self.pvt_seq += 1
            sequence = float(self.pvt_seq)
            self.pvt_result = None
            message = Float64MultiArray()
            # 当前控制进程只收一段：序号、秒、15 个关节。路点用来确认轨迹仍在同一条轴上。
            message.data = [sequence, duration, 1.0, *rows[-1]]
            print(
                f"[x1-debug] key-send seq={sequence:.0f} sec={duration:.2f} "
                f"joints={list(zip(COFFEE_JOINTS, rows[-1]))}",
                flush=True,
            )
            self.pvt_pub.publish(message)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.pvt_result:
                    continue
                parts = self.pvt_result.split(" ", 2)
                try:
                    got = float(parts[0])
                except (ValueError, IndexError):
                    continue
                if abs(got - sequence) > 0.1:
                    continue
                if len(parts) > 1 and parts[1] == "ok":
                    before = {name: start[name] for name in names}
                    self._watch = (names, before, {name: final_goal[name] for name in names})
                    return True
                detail = parts[2] if len(parts) > 2 else self.pvt_result
                print(f"\n位置目标失败: {detail}")
                return False
            print("\n位置目标没有回应。控制栈可能还没起来。")
            return False

        def note_if_ignored(self) -> None:
            if self.warned or not hasattr(self, "_watch"):
                return
            names, before, target = self._watch
            moved = max(abs(self.measured.get(name, before[name]) - before[name]) for name in names)
            asked = max(abs(target[name] - before[name]) for name in names)
            if asked > 0.01 and moved < asked * 0.25:
                self.warned = True
                print("\n关节几乎没动。请确认启动时 drives_enabled:=true，并且网口主站已经进 OP。")

    rclpy.init()
    node = Teleop()
    status = 0
    terminal = termios.tcgetattr(sys.stdin)
    try:
        needed = LEFT_JOINTS + RIGHT_JOINTS + ["Joint_Body"]
        print(
            f"[x1-debug] keyboard ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '')} "
            f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '')}",
            flush=True,
        )
        deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=10.0)
        next_report = time.monotonic()
        while rclpy.ok() and node.get_clock().now() < deadline and not node.ready():
            node.attach_joint_states()
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_report:
                missing = [name for name in needed if name not in node.measured]
                print(
                    f"[x1-debug] /joint_states publishers={node.count_publishers('/joint_states')} "
                    f"received={len(node.measured)} missing={missing} "
                    f"/x1/pvt_move subscribers={node.pvt_pub.get_subscription_count()}",
                    flush=True,
                )
                next_report = time.monotonic() + 1.0
        if not node.ready():
            missing = [name for name in needed if name not in node.measured]
            node.get_logger().error(
                "没有收齐 /joint_states"
                f" publishers={node.count_publishers('/joint_states')} missing={missing}"
            )
            return 1
        if not node.wait_controllers():
            return 1
        node.commanded = dict(node.measured)
        print(HELP)
        print(f"当前手臂: {node.side}")
        mechanical = mechanical_pose()
        away = max(abs(node.measured.get(name, 0.0) - mechanical[name]) for name in COFFEE_JOINTS)
        if away < 0.05:
            print("当前是机械零位。URDF 用这组关节角，平移从这里计算。")
        else:
            print(f"当前离机械零位 {away:.3f} rad。平移仍从实测角度计算。")
        tty.setcbreak(sys.stdin.fileno())
        pending_since = None

        def return_to(label: str, target: Dict[str, float]) -> None:
            nonlocal pending_since
            if node.zero_torque:
                print("\n当前是零力矩。先按 b 抱闸，才能回零。")
                return
            if any(name not in node.measured for name in COFFEE_JOINTS):
                print("\n还没有全部关节位置，不能回零")
                return
            farthest = max(abs(node.measured[name] - target[name]) for name in COFFEE_JOINTS)
            if farthest < 0.01:
                print(f"\n已经在{label}附近")
                node.commanded = dict(target)
                return
            if not node.send("home", COFFEE_JOINTS, [target]):
                return
            node.commanded = dict(target)
            pending_since = node.get_clock().now()
            print(f"\n回{label}，最大关节行程 {farthest:.3f} rad，约 {node.last_segment_sec:.1f} s")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if pending_since is not None and (node.get_clock().now() - pending_since).nanoseconds > int(node.last_segment_sec * 1e9):
                node.note_if_ignored()
                pending_since = None
            if not select.select([sys.stdin], [], [], 0.0)[0]:
                continue
            key = sys.stdin.read(1)
            print(f"[x1-debug] key={key!r}", flush=True)
            if key != "h":
                node.home_confirm_until = 0.0
            if key != "0":
                node.urdf_confirm_until = 0.0
            if key == "\x1b":
                break
            if key == "1":
                node.side = "left"
                print("\n左臂")
                continue
            if key == "2":
                node.side = "right"
                print("\n右臂")
                continue
            if key == " ":
                print("\n不再追加目标")
                continue
            if key == "h":
                now = time.monotonic()
                if now > node.home_confirm_until:
                    node.home_confirm_until = now + 3.0
                    node.zero_confirm_until = 0.0
                    print("\n再按一次 h：15 轴回到机械零位。请让开手臂。")
                    continue
                node.home_confirm_until = 0.0
                node.zero_confirm_until = 0.0
                return_to("机械零位", mechanical_pose())
                continue
            if key == "0":
                now = time.monotonic()
                if now > node.urdf_confirm_until:
                    node.urdf_confirm_until = now + 3.0
                    node.zero_confirm_until = 0.0
                    print("\n再按一次 0：15 轴回到 URDF 零位，关节角全部为 0。请让开手臂。")
                    continue
                node.urdf_confirm_until = 0.0
                node.zero_confirm_until = 0.0
                return_to("URDF 零位", urdf_zero_pose())
                continue
            if key == "z":
                now = time.monotonic()
                if now > node.zero_confirm_until:
                    node.zero_confirm_until = now + 3.0
                    print("\n再按一次 z：请求零力矩。当前若已是轮廓位置，这一步不会改成 0 力矩。")
                    continue
                node.zero_confirm_until = 0.0
                if node.call_trigger(node.zero_torque_client, "/x1/zero_torque"):
                    node.zero_torque = True
                    node.motion_requested = False
                continue
            if key == "m":
                now = time.monotonic()
                if now > node.mit_confirm_until:
                    node.mit_confirm_until = now + 3.0
                    print("\n再按一次 m：MIT 保持当前位置。关节会变软，请扶着手臂。")
                    continue
                node.mit_confirm_until = 0.0
                if node.call_trigger(node.mit_hold_client, "/x1/mit_hold"):
                    node.mit_hold = True
                    node.motion_requested = False
                continue
            if key == "n":
                node.mit_confirm_until = 0.0
                if node.call_trigger(node.mit_exit_client, "/x1/mit_exit"):
                    node.mit_hold = False
                continue
            if key == "b":
                node.zero_confirm_until = 0.0
                node.mit_confirm_until = 0.0
                if node.mit_hold:
                    node.call_trigger(node.mit_exit_client, "/x1/mit_exit")
                    node.mit_hold = False
                if node.call_trigger(node.brake_client, "/x1/brake_hold"):
                    node.zero_torque = False
                continue
            if node.zero_torque:
                print("\n当前是零力矩。先按 b 抱闸，才能位置控制。")
                continue
            if node.mit_hold:
                print("\n当前是 MIT。按 n 回到位置保持，才能点动。")
                continue
            linear = {
                "w": (STEP_METERS, 0.0, 0.0),
                "s": (-STEP_METERS, 0.0, 0.0),
                "a": (0.0, STEP_METERS, 0.0),
                "d": (0.0, -STEP_METERS, 0.0),
                "q": (0.0, 0.0, STEP_METERS),
                "e": (0.0, 0.0, -STEP_METERS),
            }
            angular = {
                "i": (STEP_RADIANS, 0.0, 0.0),
                "k": (-STEP_RADIANS, 0.0, 0.0),
                "j": (0.0, STEP_RADIANS, 0.0),
                "l": (0.0, -STEP_RADIANS, 0.0),
                "u": (0.0, 0.0, STEP_RADIANS),
                "o": (0.0, 0.0, -STEP_RADIANS),
            }
            if key in ("[", "]"):
                name = "Joint_Body"
                lower, upper = LIMITS[name]
                delta = WAIST_STEP if key == "]" else -WAIST_STEP
                if name not in node.measured:
                    print("\n还没有当前腰关节位置")
                    continue
                current = node.measured[name]
                target = min(upper, max(lower, current + delta))
                if abs(target - current) < 1e-6:
                    print("\n腰已到限位")
                    continue
                if not node.send("waist", [name], [{name: target}]):
                    continue
                node.commanded[name] = target
                pending_since = node.get_clock().now()
                print(f"\n腰 {current:.3f} -> {target:.3f} rad")
                continue
            if key not in linear and key not in angular:
                continue
            _, joint_names, _ = ARMS[node.side]
            if any(name not in node.measured for name in [*joint_names, "Joint_Body"]):
                print("\n还没有当前手臂关节位置")
                continue
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.0)
            seed = {name: node.measured[name] for name in [*joint_names, "Joint_Body"]}
            if key in linear:
                solved_path = chains[node.side].move_line(seed, list(linear[key]))
                if not solved_path:
                    solved = coupled_translation(chains[node.side], seed, list(linear[key]))
                    solved_path = None if solved is None else [solved]
            else:
                solved = chains[node.side].step(seed, [0.0, 0.0, 0.0], list(angular[key]))
                solved_path = None if solved is None else [solved]
            if not solved_path:
                print("\n这一步到不了，或平移会改变姿态，已保持原位")
                continue
            if not node.send(node.side, joint_names, solved_path):
                continue
            node.commanded.update(solved_path[-1])
            pending_since = node.get_clock().now()
            current_pose = _origin_of(chains[node.side].fk(seed))
            target_pose = _origin_of(chains[node.side].fk({**seed, **solved_path[-1]}))
            print(
                f"\n{node.side} 全局 TCP {current_pose[0]:.3f} {current_pose[1]:.3f} {current_pose[2]:.3f}"
                f" -> {target_pose[0]:.3f} {target_pose[1]:.3f} {target_pose[2]:.3f}"
            )
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        node.destroy_node()
        rclpy.shutdown()
    return status


def main() -> int:
    if "--self-check" in sys.argv:
        return self_check(sys.argv[2] if len(sys.argv) > 2 else os.path.join(
            os.path.dirname(__file__), "..", "..", "arm_describle", "urdf", "HRB3Q-LC.urdf"))
    return run_teleop()


if __name__ == "__main__":
    sys.exit(main())
