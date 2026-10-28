#!/usr/bin/env bash
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

if [[ "${X1_PRESERVE_UNDERLAY:-0}" != "1" ]]; then
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE="${1:-/opt/x1/moveit_source_ws}"

source /opt/ros/humble/setup.bash
mkdir -p "${WORKSPACE}/src"
vcs import "${WORKSPACE}/src" < "${SDK_ROOT}/environment/repos/moveit2_coffee.repos"

cd "${WORKSPACE}"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

echo "MoveIt2 源码已导入：${WORKSPACE}"
echo "如需构建，请执行："
echo "  source /opt/ros/humble/setup.bash"
echo "  cd ${WORKSPACE}"
echo "  colcon build --symlink-install --event-handlers console_direct+"
