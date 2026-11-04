#!/usr/bin/env bash
# 实机 MoveIt：用 root 打开 eno1，先把 15 轴对齐到 URDF 零位，再允许轨迹。
# URDF 零位是关节角全 0，不是出厂机械零位。人要在机器人旁边。
# 不要同时运行 start_real_hold.sh 或单电机程序。

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IFNAME="${1:-eno1}"
if [[ ! "${IFNAME}" =~ ^[A-Za-z0-9._:-]+$ ]]; then
  echo "网口名无效: ${IFNAME}" >&2
  exit 1
fi
PROFILE="${SDK_ROOT}/environment/scripts/fastdds_localhost.xml"

if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "还没有 install/setup.bash。请先在仓库根目录编译。" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}"

if [[ ! -t 0 || ! -t 1 ]]; then
  echo "请在普通终端里运行这个脚本，才能输入 sudo 密码。" >&2
  exit 1
fi

echo "网口 ${IFNAME}：上电后先对齐 URDF 零位（关节角全 0），再允许 MoveIt 轨迹。"
echo "下面会要求输入你的登录密码。输入时屏幕不显示字符，输完按回车。"
# 不用 sudo -E。当前终端如果 source 过 MoveIt，带进去的库路径会让控制进程在初始化时堆损坏。
sudo -k
if pgrep -x ros2_control_node >/dev/null 2>&1; then
  echo "结束残留的 ros2_control_node。"
  sudo pkill -x ros2_control_node || true
  sleep 1
fi
exec sudo env -u SHELLOPTS -u LD_LIBRARY_PATH -u LD_PRELOAD -u PYTHONPATH \
  -u CMAKE_PREFIX_PATH -u AMENT_PREFIX_PATH -u RMW_IMPLEMENTATION \
  bash --noprofile --norc -c "
  set +u
  set -eo pipefail
  unset LD_LIBRARY_PATH LD_PRELOAD PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY=1
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE='${PROFILE}'
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing_*
  source /opt/ros/humble/setup.bash
  source '${SDK_ROOT}/install/setup.bash'
  export FASTRTPS_DEFAULT_PROFILES_FILE='${PROFILE}'
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec ros2 launch x1_coffee_integration robot_startup.launch.py \
    use_fake_hardware:=false \
    motion_enabled:=true \
    drives_enabled:=true \
    arm_for_moveit:=true \
    ethercat_ifname:='${IFNAME}'
"
