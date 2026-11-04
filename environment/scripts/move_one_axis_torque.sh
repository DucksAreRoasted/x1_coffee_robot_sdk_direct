#!/usr/bin/env bash
# 只对一台电机出力。参数：网口 从站号 扭矩千分比 秒
# 例：move_one_axis_torque.sh eno1 1 50 1

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="${SDK_ROOT}/install/x1_coffee_hardware_core/lib/x1_coffee_hardware_core/ecat_one_axis_torque"
IFNAME="${1:-eno1}"
SLAVE="${2:-1}"
TORQUE="${3:-50}"
SECONDS_HOLD="${4:-1}"

if [[ ! -x "${BIN}" ]]; then
  echo "还没有 ${BIN}。请先编译 x1_coffee_hardware_core。" >&2
  exit 1
fi
if [[ ! -t 0 || ! -t 1 ]]; then
  echo "请在普通终端里运行，才能输入 sudo 密码。" >&2
  exit 1
fi

echo "从站 ${SLAVE} 将以额定电流的 ${TORQUE}/1000 出力 ${SECONDS_HOLD} 秒。请确认旁边没有人。"
echo "该轴不会停在某个角度。下面输入登录密码，输入时不显示字符。"
sudo -k
exec sudo env -u SHELLOPTS "${BIN}" "${IFNAME}" "${SLAVE}" "${TORQUE}" "${SECONDS_HOLD}"
