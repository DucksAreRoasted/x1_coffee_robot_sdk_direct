#!/usr/bin/env bash
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

if [[ "${X1_PRESERVE_UNDERLAY:-0}" != "1" ]]; then
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
fi

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="${1:-${SDK_ROOT}/environment/moveit_source_ws}"

source /opt/ros/humble/setup.bash

if [[ ! -d "${WORKSPACE}/src/moveit2" || ! -d "${WORKSPACE}/src/moveit_msgs" ]]; then
  echo "MoveIt2 源码快照不完整：${WORKSPACE}/src" >&2
  exit 1
fi

cd "${WORKSPACE}"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --packages-up-to \
    moveit_py \
    moveit_ros_move_group \
    moveit_ros_visualization \
    moveit_configs_utils \
    moveit_planners

echo "MoveIt2 源码环境构建完成：${WORKSPACE}"
echo "请执行：source ${WORKSPACE}/install/setup.bash"
