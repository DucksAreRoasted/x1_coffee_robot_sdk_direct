#!/usr/bin/env bash
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

if [[ "${X1_PRESERVE_UNDERLAY:-0}" != "1" ]]; then
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
fi

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash

MOVEIT_WS="${X1_MOVEIT_WS:-${SDK_ROOT}/environment/moveit_source_ws}"
if [[ -f "${MOVEIT_WS}/install/setup.bash" ]]; then
  source "${MOVEIT_WS}/install/setup.bash"
fi

if ! ros2 pkg prefix moveit_core >/dev/null 2>&1; then
  echo "未找到 MoveIt2 运行环境。请先执行：" >&2
  echo "  ${SDK_ROOT}/environment/scripts/build_moveit_source.sh" >&2
  exit 1
fi

cd "${SDK_ROOT}"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build \
  --base-paths "${SDK_ROOT}/src" "${SDK_ROOT}/examples" \
  --symlink-install \
  --event-handlers console_direct+

echo "SDK 构建完成。请执行："
echo "  source ${SDK_ROOT}/install/setup.bash"
