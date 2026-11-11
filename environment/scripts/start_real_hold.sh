#!/usr/bin/env bash
# 实机初始化：15 轴上电后停在当前角度。不回机械零位，不发新目标。
#
# 两个后端，由编译缓存里的 X1_ETHERCAT_BACKEND 决定，也可以用同名环境变量覆盖：
#   IGH  —— EtherLab 主站。默认拉起完整 ROS2 control / 键盘 / MoveIt 控制器；
#           X1_IGH_PROBE_ONLY=1 时只跑上电保持探测器，不启动 ROS。
#   SOEM —— 旧路线，完整拉起 ROS2 control / MoveIt / 键盘。

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
  echo "✗ EtherCAT 后端无效：${BACKEND}" >&2
  echo "  只支持 SOEM 或 IGH。检查环境变量 X1_ETHERCAT_BACKEND。" >&2
  exit 1
fi
if [[ ! "${IFNAME}" =~ ^[A-Za-z0-9._:-]+$ ]]; then
  echo "✗ 网口名无效：${IFNAME}" >&2
  echo "  用法：bash environment/scripts/start_real_hold.sh <网口名>" >&2
  exit 1
fi
if [[ ! -f "${SDK_ROOT}/install/setup.bash" ]]; then
  echo "✗ 还没有 install/setup.bash，工作区没编译过。" >&2
  echo "  在仓库根目录编译：colcon build --base-paths src --symlink-install" >&2
  exit 1
fi
if [[ ! -t 0 || ! -t 1 ]]; then
  echo "✗ 需要在普通终端里运行，才能输入 sudo 密码。" >&2
  echo "  管道或后台方式运行会卡在这里。" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${SDK_ROOT}/environment/scripts/ros_lan_env.sh"

# 拉起完整控制栈。IgH 和 SOEM 用的是同一套 launch，差别只在传输层。
run_ros_stack() {
  local backend_label="$1"
  echo "════════════════════════════════════════════════════════════════"
  echo " X1 Coffee 实机上电保持 · EtherCAT 后端：${backend_label}"
  echo "════════════════════════════════════════════════════════════════"
  echo "● 网口参数 ${IFNAME}（IgH 后端由 main_devices 绑定网卡，忽略这个参数）"
  echo "● ROS 域 ${ROS_DOMAIN_ID}。15 轴进入运行后按官方节拍接通，约 1 秒，不要退出。"
  echo "● 这条路线会完整拉起 ROS2 control、MoveIt 控制器和键盘服务。"
  echo "● motion_enabled=false：上电只保持当前位置，不会自己动。"
  echo "✓ 看到「15 轴都已使能」后，再在上位机启动 MoveIt。"
  echo ""
  echo "⚠ Ctrl-C 会退使能，手臂失去保持力矩、会因重力下坠。请先确认有支撑。"
  echo ""
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
}

if [[ "${BACKEND}" == "SOEM" ]]; then
  run_ros_stack "SOEM（旧路线）"
fi

# ── 以下为 IgH 后端 ────────────────────────────────────────────────
IGH_PROBE="${SDK_ROOT}/install/x1_coffee_hardware_core/lib/x1_coffee_hardware_core/igh_init_probe"
IGH_MAIN_DEVICES="/sys/module/ec_master/parameters/main_devices"

if [[ "${X1_IGH_PROBE_ONLY:-0}" == "1" ]]; then
  echo "════════════════════════════════════════════════════════════════"
  echo " X1 Coffee 实机上电保持 · EtherCAT 后端：IgH（只跑探测器）"
  echo "════════════════════════════════════════════════════════════════"
  echo "这一步只做三件事："
  echo "  1. 15 个从站进 OP，过程数据凑满工作计数 WKC=45"
  echo "  2. 把当前实测位置锁存成保持目标（不回零、不发新目标）"
  echo "  3. 稳 5 秒后按 eRob 官方节拍使能，之后一直保持不动"
  echo ""
  echo "这一步不启动 ROS / MoveIt / 键盘，也不接收任何运动指令。"
  echo "只有看到「all 15 drives report Operation Enabled」才算成功。"
  echo "想看键盘和 MoveIt，去掉 X1_IGH_PROBE_ONLY=1 再跑一次。"
  echo ""
fi

if [[ ! -x "${IGH_PROBE}" ]]; then
  echo "✗ 找不到 IgH 探测器：${IGH_PROBE}" >&2
  echo "  这个可执行文件只在 -DX1_ETHERCAT_BACKEND=IGH 时编译生成。" >&2
  echo "  重新编译：colcon build --packages-select x1_coffee_hardware_core" >&2
  exit 1
fi
if [[ ! -r "${IGH_MAIN_DEVICES}" ]] || [[ -z "$(tr -d '[:space:]' < "${IGH_MAIN_DEVICES}")" ]]; then
  echo "✗ IgH 主站未加载，或 main_devices 没绑定网卡 MAC，拒绝启动。" >&2
  echo "  正常应该能读到形如 00:f1:f5:39:ac:5b 的 MAC。" >&2
  echo "  检查：lsmod | grep ec_master  和  /etc/ethercat.conf 里的 MASTER0_DEVICE" >&2
  exit 1
fi
if ! systemctl is-active --quiet ethercat; then
  echo "✗ EtherLab 的 ethercat 服务没在运行，拒绝启动。" >&2
  echo "  拉起：sudo systemctl start ethercat  然后 sudo systemctl status ethercat" >&2
  exit 1
fi

if [[ "${X1_IGH_PROBE_ONLY:-0}" != "1" ]]; then
  echo "● IgH 主站绑定网卡 MAC：$(tr -s ',' ' ' < "${IGH_MAIN_DEVICES}")"
  echo "● 这一轮 ROS2 control 走 IgH 传输层：标定、限位、使能节拍和 PVT 与 SOEM 相同。"
  echo "● 上电并保持当前位置，不回零，不发新目标。"
  echo ""
  echo "● 常见失败与含义："
  echo "    IgH: 请求主站失败             → 有别的进程占着主站，先退出旧进程"
  echo "    60 秒内没有 15 台同时进入运行 → 从站没全部进 OP，看 dmesg 里的 AL 状态码"
  echo "    AL 0x001B Sync manager watchdog → DC 同步模式没写进驱动器"
  echo "    AL 0x001A Synchronization error → DC 配了但时钟没收敛"
  echo ""
  run_ros_stack "IgH"
fi

IGH_DEVICE_MACS="$(tr -s ',' ' ' < "${IGH_MAIN_DEVICES}")"
echo "● 主站绑定网卡 MAC：${IGH_DEVICE_MACS}"
echo "● 探测器以 chrt -f 80 绑 CPU 3 运行，日志节拍 1 ms。"
echo "● 常见失败与含义："
echo "    no complete 15-axis feedback within 15 s → 从站没全部进 OP，多为同步模式没配好"
echo "    AL 0x001B Sync manager watchdog          → DC 同步模式没写进驱动器"
echo "    AL 0x001A Synchronization error          → DC 配了但时钟没收敛，试位移改 0"
echo ""
echo "⚠ Ctrl-C 会发 Shutdown 退使能，手臂失去保持力矩、会因重力下坠。"
echo "  退之前先把臂放到有支撑的姿态，或让人扶住。软件停止不等于硬件急停。"
echo ""
echo "● 下面输入登录密码。输入时不显示，输完按回车。"
sudo -k
exec sudo -E chrt -f 80 taskset --cpu-list 3 "${IGH_PROBE}"
