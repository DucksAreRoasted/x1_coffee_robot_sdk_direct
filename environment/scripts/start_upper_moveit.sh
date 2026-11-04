#!/usr/bin/env bash
# 上位机：MoveIt、RViz 和键盘点动。不打开网口，不上电。
# 下位机先运行 start_real_hold.sh。

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "还没有 install/setup.bash。请先在仓库根目录编译。" >&2
  exit 1
fi
if [[ ! -t 0 || ! -t 1 ]]; then
  echo "请在普通终端里运行，键盘需要这个终端接收按键。" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${SDK_ROOT}/environment/scripts/ros_lan_env.sh"
export LD_LIBRARY_PATH="${SDK_ROOT}/environment/moveit_compat:${LD_LIBRARY_PATH:-}"

source /opt/ros/humble/setup.bash
source "${SDK_ROOT}/install/setup.bash"

MOVE_GROUP="${SDK_ROOT}/environment/moveit_source_ws/install/moveit_ros_move_group/lib/moveit_ros_move_group/move_group"
if [[ -e "${MOVE_GROUP}" && ! -x "${MOVE_GROUP}" ]]; then
  echo "MoveIt 的 move_group 没有执行权限。下面输入登录密码，只补上安装目录里的执行位。"
  sudo find "${SDK_ROOT}/environment/moveit_source_ws/install" \
    -type f -path '*/lib/*/*' ! -path '*/python*' ! -perm -111 -exec chmod a+x {} +
fi

MOVEIT_PID=""
stop_moveit() {
  if [[ -n "${MOVEIT_PID}" ]] && kill -0 "${MOVEIT_PID}" 2>/dev/null; then
    kill -TERM -- "-${MOVEIT_PID}" 2>/dev/null || kill -TERM "${MOVEIT_PID}" 2>/dev/null || true
    wait "${MOVEIT_PID}" 2>/dev/null || true
  fi
  MOVEIT_PID=""
}
trap stop_moveit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

MOVEIT_LOG="$(mktemp /tmp/x1-upper-moveit.XXXXXX.log)"
setsid ros2 launch x1_coffee_moveit_config keyboard_moveit.launch.py \
  </dev/null >"${MOVEIT_LOG}" 2>&1 &
MOVEIT_PID=$!

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，连接下位机 192.168.2.230。"
echo "已启动 MoveIt / RViz，日志: ${MOVEIT_LOG}"
echo "下位机需已经打印“15 轴已使能并保持”。"
echo "本终端是键盘：先等 missing=[]。1/2 选臂，w/s/a/d/q/e 平移 5 mm，Esc 退出。"
echo "退出键盘时会关闭本次 RViz。"

set +e
ros2 run x1_coffee_integration keyboard_tcp_teleop.py
status=$?
set -e
exit "${status}"
