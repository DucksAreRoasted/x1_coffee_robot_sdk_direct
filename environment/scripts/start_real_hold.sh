#!/usr/bin/env bash
# 实机初始化：打开 eno1，15 轴上电后停在当前角度。不回机械零位，不发新目标。

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IFNAME="${1:-eno1}"
if [[ ! "${IFNAME}" =~ ^[A-Za-z0-9._:-]+$ ]]; then
  echo "网口名无效: ${IFNAME}" >&2
  exit 1
fi
if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "还没有 install/setup.bash。请先在仓库根目录编译。" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${SDK_ROOT}/environment/scripts/ros_lan_env.sh"

if [[ ! -t 0 || ! -t 1 ]]; then
  echo "请在普通终端里运行这个脚本，才能输入 sudo 密码。" >&2
  exit 1
fi

echo "网口 ${IFNAME}：上电并保持当前位置。"
echo "15 轴各自接通。某轴若停在准备接通，会退回关电压并间隔 300 ms 再试，最多 5 次。"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，上位机用同一域打开 RViz。"
echo "请等待“15 轴已使能并保持，未下发运动”后再在上位机启动 MoveIt。"
echo "等待期间“尚未使能 N 轴”是进度信息；不要仅等待几秒就退出，默认检查超时为 60 秒。"
echo "下面会要求输入你的登录密码。输入时屏幕不显示字符，输完按回车。"
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
  exec ros2 launch x1_coffee_integration robot_startup.launch.py \
    use_fake_hardware:=false \
    motion_enabled:=false \
    drives_enabled:=true \
    ethercat_ifname:='${IFNAME}'
"
