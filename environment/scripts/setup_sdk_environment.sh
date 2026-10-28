#!/usr/bin/env bash

# 该脚本需要使用 source 调用，而不是直接执行：
# source environment/scripts/setup_sdk_environment.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "请使用 source 调用此脚本" >&2
  exit 2
fi

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash

MOVEIT_WS="${X1_MOVEIT_WS:-${SDK_ROOT}/environment/moveit_source_ws}"
if [[ -f "${MOVEIT_WS}/install/setup.bash" ]]; then
  source "${MOVEIT_WS}/install/setup.bash"
fi

if [[ -f "${SDK_ROOT}/install/setup.bash" ]]; then
  source "${SDK_ROOT}/install/setup.bash"
fi

export X1_COFFEE_SDK_ROOT="${SDK_ROOT}"
export X1_GTN_LIBRARY_PATH="${X1_GTN_LIBRARY_PATH:-/opt/x1/gtn/lib/libgxn_x64.so}"
export X1_GTN_CONFIG_PATH="${X1_GTN_CONFIG_PATH:-/etc/x1/gtn/gtn_core1.cfg}"
export X1_ECAT_CONFIG_PATH="${X1_ECAT_CONFIG_PATH:-/etc/x1/gtn/Gecat.eni}"

echo "已加载 X1 Coffee SDK 环境：${SDK_ROOT}"
echo "MoveIt 源码工作空间：${MOVEIT_WS}"
