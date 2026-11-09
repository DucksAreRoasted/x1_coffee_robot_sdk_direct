#!/usr/bin/env bash
# 用零差官方 eRob_SOEM_linux 的 slaveinfo 看从站链路。不使能。
# 关节总线在下位机：bash environment/scripts/erob_slaveinfo.sh enp3s0
# eno1 是 192.168.2.230 到上位机的网口，上面没有关节。

set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="${SDK_ROOT}/src/x1_coffee_hardware_core/third_party/eRob_SOEM_linux"
BUILD="${SDK_ROOT}/build/eRob_SOEM_linux"
IFNAME="${1:-enp3s0}"

if [[ ! -d "${SRC}/soem" ]]; then
  echo "还没有官方源码。请先在仓库里执行：" >&2
  echo "git clone https://github.com/ZeroErrControl/eRob_SOEM_linux.git ${SRC}" >&2
  exit 1
fi
if [[ ! -x "${BUILD}/test/linux/slaveinfo/slaveinfo" ]]; then
  cmake -S "${SRC}" -B "${BUILD}"
  cmake --build "${BUILD}" --target slaveinfo -j"$(nproc)"
fi

echo "● 官方 slaveinfo，网口 ${IFNAME}。只读从站信息，不使能。"
exec sudo -E env LD_LIBRARY_PATH="${BUILD}" "${BUILD}/test/linux/slaveinfo/slaveinfo" "${IFNAME}"
