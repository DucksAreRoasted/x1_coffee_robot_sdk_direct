#!/usr/bin/env bash
# 在下位机的图形桌面手动调用上游 GUI，不自动选择网口或使能电机。
set -euo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GUI_BUILD="${SDK_ROOT}/build/eRob_SOEM_GUI"

if [[ $# -ne 0 ]]; then
  echo "用法：bash environment/scripts/start_erob_gui.sh（在下位机图形桌面执行）" >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "未发现图形桌面，请在下位机桌面的终端执行。" >&2
  exit 1
fi
# 只拦截已知 SDK 主站进程，不自动停止任何用户进程。
if pgrep -f '(^|/)(ros2_control_node|ecat_one_axis_jog|ecat_one_axis_torque|ecat_safeop_diag|slaveinfo|ethercat_monitor)( |$)' >/dev/null; then
  echo "发现控制节点或 EtherCAT 诊断进程。请先退出已有主站，再启动 GUI。" >&2
  exit 1
fi
if [[ ! -x "${GUI_BUILD}/bin/ethercat_monitor" ]]; then
  bash "${SDK_ROOT}/environment/scripts/build_erob_gui.sh"
fi

echo "● 下位机诊断工具，连接关节的网口为 enp3s0，请在 GUI 中手动选择。"
echo "● 不可与 start_real_hold.sh 或其他 EtherCAT 主站同时运行。"
echo "● 请先选择 1..15 中的一个电机；连接后选轴锁定，换轴须退出并重开。"
echo "● 控制和反馈只针对所选轴，诊断工具仍不接入 SDK 的标定和限位。"
echo "● 此 GUI 可写参数、使能和运动；操作前请确认现场安全并备好硬件急停。"
# 从构建目录运行，上游运行时文件不写进 SDK 源码目录。
cd "${GUI_BUILD}"
exec sudo -E env LD_LIBRARY_PATH="${GUI_BUILD}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  "${GUI_BUILD}/bin/ethercat_monitor"
