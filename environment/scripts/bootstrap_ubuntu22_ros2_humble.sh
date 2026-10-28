#!/usr/bin/env bash
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

if [[ "${X1_PRESERVE_UNDERLAY:-0}" != "1" ]]; then
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE_FILE="${SDK_ROOT}/environment/apt/ros2_humble_packages.txt"

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

if [[ ! -f /etc/os-release ]]; then
  echo "无法识别操作系统" >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "该环境文件按 Ubuntu 22.04 编写，当前为 ${ID:-unknown} ${VERSION_ID:-unknown}" >&2
  exit 2
fi

mapfile -t PACKAGES < <(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "${PACKAGE_FILE}")

${SUDO} apt-get update
${SUDO} apt-get install -y "${PACKAGES[@]}"

source /opt/ros/humble/setup.bash

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  ${SUDO} rosdep init
fi
rosdep update

echo "ROS2 Humble 基础环境安装完成。"
echo "下一步：source /opt/ros/humble/setup.bash"
