#!/usr/bin/env bash
# 键盘末端点动，并打开 MoveIt。执行走 PVT，不走动态 PT。
# 先在另一个终端运行 start_real_hold.sh，看到上电保持后再开这个。

set +u
set -eo pipefail

SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROFILE="${SDK_ROOT}/environment/scripts/fastdds_localhost.xml"
RVIZ_CONFIG="${SDK_ROOT}/src/x1_coffee_integration/config/keyboard_teleop.rviz"
OPEN_RVIZ=true
if [[ "${1:-}" == "--no-rviz" && $# -eq 1 ]]; then
  OPEN_RVIZ=false
elif [[ $# -ne 0 ]]; then
  echo "用法: $0 [--no-rviz]" >&2
  exit 2
fi

if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "还没有 install/setup.bash。请先在仓库根目录编译。" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}"
export LD_LIBRARY_PATH="${SDK_ROOT}/environment/moveit_compat:${LD_LIBRARY_PATH:-}"

source /opt/ros/humble/setup.bash
source "${SDK_ROOT}/install/setup.bash"

MOVEIT_PID=""
EXEC_PID=""
stop_pid() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then
    return
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "${pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
}
cleanup_preview() {
  stop_pid "${MOVEIT_PID}"
  stop_pid "${EXEC_PID}"
}
trap cleanup_preview EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${OPEN_RVIZ}" == true ]]; then
  # 已经在跑的 root 上电进程仍可能占用 FastDDS 共享内存。放宽权限后，桌面用户才能读到模型。
  shopt -s nullglob
  fastdds_segments=(/dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*)
  shopt -u nullglob
  if [[ ${#fastdds_segments[@]} -gt 0 ]]; then
    sudo -E chmod a+rw "${fastdds_segments[@]}"
  fi
  MOVE_GROUP="${SDK_ROOT}/environment/moveit_source_ws/install/moveit_ros_move_group/lib/moveit_ros_move_group/move_group"
  if [[ -e "${MOVE_GROUP}" && ! -x "${MOVE_GROUP}" ]]; then
    echo "MoveIt 的 move_group 没有执行权限。下面输入登录密码，只补上安装目录里的执行位。"
    sudo find "${SDK_ROOT}/environment/moveit_source_ws/install" \
      -type f -path '*/lib/*/*' ! -path '*/python*' ! -perm -111 -exec chmod a+x {} +
  fi
  MOVEIT_LOG="$(mktemp /tmp/x1-keyboard-moveit.XXXXXX.log)"
  EXEC_LOG="$(mktemp /tmp/x1-keyboard-pvt.XXXXXX.log)"
  # 规划和 RViz 用当前桌面用户。执行器把轨迹收成 PVT，不占用键盘终端。
  setsid ros2 launch x1_coffee_moveit_config keyboard_moveit.launch.py \
    </dev/null >"${MOVEIT_LOG}" 2>&1 &
  MOVEIT_PID=$!
  setsid ros2 run x1_coffee_integration pvt_trajectory_executor.py \
    </dev/null >"${EXEC_LOG}" 2>&1 &
  EXEC_PID=$!
  echo "已启动 MoveIt 和 PVT 执行（规划 PID=${MOVEIT_PID}，执行 PID=${EXEC_PID}）。"
  echo "规划日志: ${MOVEIT_LOG}"
  echo "执行日志: ${EXEC_LOG}"
  echo "RViz 里规划后点 Execute。轨迹按 PVT 逐点下发，不使用动态 PT。"
  echo "按键仍在当前终端。退出键盘时会关闭本次 MoveIt。"
fi

echo "[x1-debug] keyboard ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "键盘和上电进程一样用 root，才能读到关节状态。可能再要一次 sudo 密码。"
echo "先等 [x1-debug] 里 missing=[]，再按键。1/2 选臂，w/s/a/d/q/e 平移 5 mm，Esc 退出。"
echo "若 RViz 能看见模型但 Execute 不动，请先停掉再重新运行 start_real_hold.sh，让新的通信配置生效。"
# 外层保留到键盘退出，才能执行 EXIT 清理；内层仍用 exec 保留原有终端和退出码语义。
sudo -E env -u SHELLOPTS bash --noprofile --norc -c "
  set +u
  set -eo pipefail
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY=1
  export FASTRTPS_DEFAULT_PROFILES_FILE='${PROFILE}'
  source /opt/ros/humble/setup.bash
  source '${SDK_ROOT}/install/setup.bash'
  exec ros2 run x1_coffee_integration keyboard_tcp_teleop.py
"
