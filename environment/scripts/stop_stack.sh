#!/usr/bin/env bash
# 温柔停止控制栈：SIGINT → SIGTERM → SIGKILL，最后确认主站已释放。
#
# 为什么不用 pkill -9：
#   1) 只杀 launch 会留下 ros2_control_node 孤儿进程，它还占着 /dev/EtherCAT0，
#      下一次启动就会报 Device or resource busy；
#   2) 直接 -9 会让内核里的 IgH 主站停在 Operation，systemctl restart ethercat
#      也救不回来，只能重启机器。
#
# 用法：bash environment/scripts/stop_stack.sh
set -u

say() { printf '%s\n' "$*"; }

sudo_cmd=(sudo)
if [ "${EUID}" -ne 0 ]; then
  say "需要 sudo 权限"
fi

launch_pids=$(pgrep -f "ros2 launch x1_coffee_integratio[n]" || true)
if [ -n "${launch_pids}" ]; then
  say "SIGINT → launch ${launch_pids}"
  # shellcheck disable=SC2086
  sudo kill -INT ${launch_pids} 2>/dev/null || true
else
  say "没有运行中的 launch"
fi

for _ in $(seq 1 20); do
  if ! pgrep -f "ros2_contro[l]_node" >/dev/null; then break; fi
  sleep 1
done

if pgrep -f "ros2_contro[l]_node" >/dev/null; then
  say "节点未在 20 秒内退出，改发 SIGTERM"
  sudo pkill -TERM -f "ros2_contro[l]_node" || true
  sleep 8
fi

if pgrep -f "ros2_contro[l]_node" >/dev/null; then
  say "SIGTERM 也无效，只能 SIGKILL；之后若主站不可用需要重启下位机"
  sudo pkill -KILL -f "ros2_contro[l]_node" || true
  sleep 3
fi

# launch 之外的残留（机器人上电脚本自己起的 python 节点）
for pattern in "robot_stat[e]_publisher" "safety_servic[e]" "gripper_action_serve[r]" \
               "spawn_controlle[r]s" "pvt_trajector[y]_executor" "x1_coffee_robot_sdk/instal[l]"; do
  if pgrep -f "${pattern}" >/dev/null; then
    say "残留进程 ${pattern}，发 SIGTERM"
    sudo pkill -TERM -f "${pattern}" || true
  fi
done
sleep 2

left=$(pgrep -af "ros2_contro[l]_node|ros2 launc[h] x1_coffee" || true)
if [ -n "${left}" ]; then
  say "✗ 仍有进程："
  printf '%s\n' "${left}"
else
  say "✓ 控制栈已退出"
fi

if sudo fuser /dev/EtherCAT0 >/dev/null 2>&1; then
  say "✗ /dev/EtherCAT0 仍被占用：$(sudo fuser /dev/EtherCAT0 2>&1)"
  say "  这种情况下下一次启动会报 Device or resource busy，需要重启下位机"
else
  say "✓ 主站已释放，可以重新启动"
fi
