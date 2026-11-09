#!/usr/bin/env bash
# 独立编译官方诊断 GUI，不构建 ROS 包，不连接 EtherCAT。
set -euo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GUI_SOURCE="${SDK_ROOT}/tools/eRob_SOEM_GUI"
GUI_BUILD="${SDK_ROOT}/build/eRob_SOEM_GUI"

if [[ $# -ne 0 ]]; then
  echo "用法：bash environment/scripts/build_erob_gui.sh" >&2
  exit 2
fi
if ! pkg-config --exists Qt5Widgets Qt5PrintSupport Qt5Concurrent Qt5Network; then
  echo "缺少 Qt5 开发依赖，请安装：sudo apt install build-essential cmake pkg-config qtbase5-dev qt5-qmake" >&2
  exit 1
fi

cmake -S "${GUI_SOURCE}" -B "${GUI_BUILD}" -DCMAKE_BUILD_TYPE=Release
# Qt 绘图库编译耗内存，下位机默认只开两个编译任务。
cmake --build "${GUI_BUILD}" --target ethercat_monitor --parallel "${X1_GUI_BUILD_JOBS:-2}"
echo "诊断 GUI 已生成：${GUI_BUILD}/bin/ethercat_monitor"
