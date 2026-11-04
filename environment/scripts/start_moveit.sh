#!/usr/bin/env bash
# 打开 MoveIt 规划和 RViz。不占用网口。
# 另一个终端要先把直连主站以允许运动的方式拉起，人站在机器人旁边。

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROFILE="${SDK_ROOT}/environment/scripts/fastdds_localhost.xml"

if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "还没有 install/setup.bash。请先在仓库根目录编译。" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}"

MOVE_GROUP="${SDK_ROOT}/environment/moveit_source_ws/install/moveit_ros_move_group/lib/moveit_ros_move_group/move_group"
if [[ ! -x "${MOVE_GROUP}" ]]; then
  echo "MoveIt 的 move_group 没有执行权限。下面输入登录密码，只补上安装目录里的执行位。"
  sudo find "${SDK_ROOT}/environment/moveit_source_ws/install" \
    -type f -path '*/lib/*/*' ! -path '*/python*' ! -perm -111 -exec chmod a+x {} +
fi

source /opt/ros/humble/setup.bash
source "${SDK_ROOT}/install/setup.bash"
# 锁定的 MoveIt 按 libgeometric_shapes.so.2.3.2 链接，本机 Humble 是 2.3.4。
export LD_LIBRARY_PATH="${SDK_ROOT}/environment/moveit_compat:${LD_LIBRARY_PATH:-}"

echo "控制栈请先在另一个终端运行 start_moveit_control.sh，并停掉单电机程序和 start_real_hold.sh。"
echo "看到「已对齐 URDF 零位，运动已允许」后再在 RViz 里执行。本窗口只启动规划。"

exec ros2 launch x1_coffee_moveit_config direct_moveit.launch.py
