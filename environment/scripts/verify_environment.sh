#!/usr/bin/env bash
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ -f "${SDK_ROOT}/install/setup.bash" ]]; then
  source "${SDK_ROOT}/install/setup.bash"
fi

required_commands=(ros2 colcon xacro)
for command_name in "${required_commands[@]}"; do
  command -v "${command_name}" >/dev/null || {
    echo "缺少命令：${command_name}" >&2
    exit 1
  }
done

required_paths=(
  "${SDK_ROOT}/src/arm_describle/package.xml"
  "${SDK_ROOT}/src/arm_describle/urdf/HRB3Q-LC.urdf"
  "${SDK_ROOT}/src/arm_describle/meshes0408/Origin_base.STL"
  "${SDK_ROOT}/environment/apt/ros2_humble_packages.txt"
  "${SDK_ROOT}/environment/repos/moveit2_coffee.repos"
)
for path in "${required_paths[@]}"; do
  [[ -e "${path}" ]] || {
    echo "缺少环境文件：${path}" >&2
    exit 1
  }
done

for package_name in controller_manager xacro moveit_core moveit_ros_move_group arm_describle x1_coffee_moveit_config; do
  ros2 pkg prefix "${package_name}" >/dev/null || {
    echo "ROS2 包未找到：${package_name}" >&2
    exit 1
  }
done

xacro "${SDK_ROOT}/src/x1_coffee_moveit_config/config/HRB3Q-LC.urdf.xacro" \
  gtn_library_path:="${X1_GTN_LIBRARY_PATH:-/opt/x1/gtn/lib/libgxn_x64.so}" \
  gtn_config_path:="${X1_GTN_CONFIG_PATH:-/etc/x1/gtn/gtn_core1.cfg}" \
  ecat_config_path:="${X1_ECAT_CONFIG_PATH:-/etc/x1/gtn/Gecat.eni}" \
  motion_enabled:=false \
  >/dev/null

echo "X1 Coffee SDK 环境检查通过。"
