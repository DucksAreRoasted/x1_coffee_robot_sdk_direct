#!/usr/bin/env bash
# 实机初始化：打开 eno1，15 轴上电后停在当前角度。不回机械零位，不发新目标。

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IFNAME="${1:-eno1}"
BACKEND="${X1_ETHERCAT_BACKEND:-}"
if [[ -z "${BACKEND}" ]]; then
  if [[ -f "${SDK_ROOT}/build/x1_coffee_hardware_core/CMakeCache.txt" ]] &&
    grep -q '^X1_ETHERCAT_BACKEND:STRING=IGH$' \
      "${SDK_ROOT}/build/x1_coffee_hardware_core/CMakeCache.txt"; then
    BACKEND="IGH"
  else
    BACKEND="SOEM"
  fi
fi
BACKEND="${BACKEND^^}"

if [[ "${BACKEND}" != "SOEM" && "${BACKEND}" != "IGH" ]]; then
  echo "EtherCAT 后端无效: ${BACKEND}，只支持 SOEM 或 IGH" >&2
  exit 1
fi
if [[ ! "${IFNAME}" =~ ^[A-Za-z0-9._:-]+$ ]]; then
  echo "网口名无效: ${IFNAME}" >&2
  exit 1
fi
if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "还没有 install/setup.bash。请先在仓库根目录编译。" >&2
  exit 1
fi

if [[ ! -t 0 || ! -t 1 ]]; then
  echo "请在普通终端里运行这个脚本，才能输入 sudo 密码。" >&2
  exit 1
fi

if [[ "${BACKEND}" == "IGH" ]]; then
  IGH_PROBE="${SDK_ROOT}/install/x1_coffee_hardware_core/lib/x1_coffee_hardware_core/igh_init_probe"
  IGH_MAIN_DEVICES="/sys/module/ec_master/parameters/main_devices"
  if [[ ! -x "${IGH_PROBE}" ]]; then
    echo "已选择 IgH，但未找到可执行探测器: ${IGH_PROBE}" >&2
    echo "请先以 -DX1_ETHERCAT_BACKEND=IGH 编译 x1_coffee_hardware_core。" >&2
    exit 1
  fi
  if [[ ! -r "${IGH_MAIN_DEVICES}" ]] || [[ -z "$(tr -d '[:space:]' < "${IGH_MAIN_DEVICES}")" ]]; then
    echo "IgH 主站未加载或 main_devices 未配置，拒绝启动。" >&2
    exit 1
  fi
  if ! systemctl is-active --quiet ethercat; then
    echo "EtherLab ethercat 服务未运行，拒绝启动。" >&2
    exit 1
  fi
  IGH_DEVICE_MACS="$(tr -s ',' ' ' < "${IGH_MAIN_DEVICES}")"
  echo "● EtherCAT 后端：IgH，主站绑定设备 MAC：${IGH_DEVICE_MACS}。"
  echo "● IgH 使用 EtherLab main_devices 绑定，不使用网口参数 ${IFNAME}。"
  echo "● 此阶段只运行 15 轴上电使能与当前位置保持探测器，不启动 ROS/MoveIt。"
  echo "● 全部使能后保持运行；Ctrl-C 会发送 Shutdown 并退出。"
  echo "● 下面输入登录密码。输入时不显示，输完按回车。"
  sudo -k
  exec sudo -E chrt -f 80 taskset --cpu-list 3 "${IGH_PROBE}"
fi

# shellcheck disable=SC1091
source "${SDK_ROOT}/environment/scripts/ros_lan_env.sh"

echo "● 网口 ${IFNAME}：上电并保持当前位置，不回零。"
echo "● ROS 域 ${ROS_DOMAIN_ID}。15 轴进入运行后按官方节拍接通，约 1 秒，不要退出。"
echo "● 有轴没进运行时会断开总线再上电，直到 15 轴使能。Ctrl-C 停止。"
echo "✓ 看到「15 轴都已使能」后，再在上位机启动。"
echo "● 下面输入登录密码。输入时不显示，输完按回车。"
# 清掉几分钟内的 sudo 记录，否则不会再问密码。
sudo -k
exec sudo -E env -u SHELLOPTS bash --noprofile --norc -c "
  set +u
  set -eo pipefail
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTRTPS_DEFAULT_PROFILES_FILE='${FASTRTPS_DEFAULT_PROFILES_FILE}'
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing_*
  source /opt/ros/humble/setup.bash
  source '${SDK_ROOT}/install/setup.bash'
  # 主线程及普通子线程继承 CPU 2，EtherCAT 周期线程在 SDK 中单独绑定 CPU 3。
  exec taskset --cpu-list 2 ros2 launch x1_coffee_integration robot_startup.launch.py \
    use_fake_hardware:=false \
    motion_enabled:=false \
    drives_enabled:=true \
    ethercat_ifname:='${IFNAME}' \
    check_timeout_sec:=600
"
