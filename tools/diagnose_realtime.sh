#!/usr/bin/env bash
set -Eeo pipefail

# Read-only, one-shot diagnostic for the X1 Coffee ros2_control stack.
# It never enables motion and never changes CPU/system settings.

DURATION_SEC="${DURATION_SEC:-30}"
START_STACK=0
if [[ "${1:-}" == "--launch" ]]; then
  START_STACK=1
  if [[ "${2:-}" =~ ^[0-9]+$ ]]; then DURATION_SEC="$2"; fi
elif [[ "${1:-}" =~ ^[0-9]+$ ]]; then
  # A numeric argument samples an already-running stack; it must never start
  # another controller_manager instance implicitly.
  DURATION_SEC="$1"
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-/tmp/x1_coffee_realtime_${STAMP}}"
mkdir -p "$OUT_DIR"
REPORT="$OUT_DIR/report.txt"
LAUNCH_PID=""

exec > >(tee "$OUT_DIR/console.log") 2>&1

say() { printf '\n=== %s ===\n' "$*" | tee -a "$REPORT"; }
run() {
  local name="$1"; shift
  printf '\n--- %s ---\n' "$name" | tee -a "$REPORT"
  ("$@" 2>&1 || true) | tee -a "$REPORT"
}

cleanup() {
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ROS Humble setup scripts dereference optional environment variables.  Keep
# nounset disabled while sourcing them; this script does not require nounset.
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then source "$ROOT_DIR/install/setup.bash"; fi

say "X1 Coffee realtime diagnostic"
printf 'root=%s\nduration_sec=%s\nout_dir=%s\n' "$ROOT_DIR" "$DURATION_SEC" "$OUT_DIR" | tee -a "$REPORT"

if [[ "$START_STACK" == 1 ]]; then
  say "Starting read-only real hardware stack"
  ROS_LOG_DIR="$OUT_DIR/ros_logs" ros2 launch x1_coffee_integration sdk_control.launch.py \
    hardware_profile:=coffee_15_no_slide \
    control_mode:=moveit use_fake_hardware:=false motion_enabled:=false \
    enable_ft_sensors:=false >"$OUT_DIR/launch.log" 2>&1 &
  LAUNCH_PID=$!
  sleep 10
fi

say "Process and ROS graph"
run "processes" pgrep -af 'ros2_control_node|robot_state_publisher|controller_manager/spawner|sdk_control.launch.py'
run "nodes" ros2 node list
run "controllers" ros2 control list_controllers
run "hardware" ros2 control list_hardware_components
run "controller_manager_params" ros2 param dump /controller_manager
run "joint_state_info" ros2 topic info /joint_states --verbose
run "left_state_info" ros2 topic info /left_group_controller/controller_state --verbose
run "right_state_info" ros2 topic info /right_group_controller/controller_state --verbose

say "System scheduling"
run "governors" bash -c 'for cpu in /sys/devices/system/cpu/cpu[0-9]*; do printf "%s: " "$cpu"; cat "$cpu/cpufreq/scaling_governor" 2>/dev/null || true; done'
run "frequencies" bash -c 'for cpu in /sys/devices/system/cpu/cpu[0-9]*; do printf "%s: " "$cpu"; cat "$cpu/cpufreq/scaling_cur_freq" 2>/dev/null || true; done'
run "load" bash -c 'uptime; nproc; cat /proc/loadavg'
run "virtualization" systemd-detect-virt
run "cgroup" systemd-cgtop --iterations=3

CONTROL_PID="$(pgrep -n -f 'ros2_control_node' || true)"
if [[ -n "$CONTROL_PID" ]]; then
  run "control_process" ps -p "$CONTROL_PID" -o pid,ppid,stat,ni,pri,rtprio,psr,pcpu,pmem,etime,cmd
  run "control_threads" ps -L -p "$CONTROL_PID" -o pid,tid,psr,stat,rtprio,pri,ni,pcpu,wchan:32,comm
  run "control_sched" chrt -p "$CONTROL_PID"
fi

say "Topic frequency sampling (${DURATION_SEC}s each)"
run "joint_states_hz" timeout "$DURATION_SEC" ros2 topic hz /joint_states
run "left_controller_state_hz" timeout "$DURATION_SEC" ros2 topic hz /left_group_controller/controller_state
run "right_controller_state_hz" timeout "$DURATION_SEC" ros2 topic hz /right_group_controller/controller_state

say "Hardware callback counters"
if [[ -f /tmp/x1_coffee_cycle_rate_after_log_fix.log ]]; then
  run "existing_cycle_log" tail -200 /tmp/x1_coffee_cycle_rate_after_log_fix.log
fi
find /tmp -maxdepth 3 -type f -name '*.log' -print0 2>/dev/null |
  xargs -0 grep -H -E 'Hardware cycle rate|GTN read timing|watchdog timeout|FIFO' 2>/dev/null |
  tail -200 | tee "$OUT_DIR/cycle_log_matches.txt" | tee -a "$REPORT" || true

say "Automatic interpretation"
python3 - "$OUT_DIR" <<'PY' | tee -a "$REPORT"
import pathlib, re, sys
root = pathlib.Path(sys.argv[1])
text = ''
for p in root.rglob('*'):
    if p.is_file() and p.name.endswith(('.log','.txt')):
        try: text += p.read_text(errors='ignore') + '\n'
        except Exception: pass
matches = re.findall(r'Hardware cycle rate: read_hz=([0-9.]+) write_hz=([0-9.]+)', text)
if matches:
    r = [float(a) for a,b in matches[-10:]]; w = [float(b) for a,b in matches[-10:]]
    print(f'cycle_samples={len(matches)} read_hz_last={r[-1]:.2f} write_hz_last={w[-1]:.2f}')
    if min(r) >= 95 and min(w) >= 95: print('verdict=cycle_rate_ok_but_check_topic_max_gap')
    elif max(r) < 90: print('verdict=controller_loop_below_90hz_check_scheduler_executor')
    else: print('verdict=controller_loop_degraded_check_max_gap_and_system_load')
else:
    print('verdict=no_cycle_rate_log_found; ensure rebuilt install is sourced')
if 'watchdog timeout during enabled motion' in text or '动态 PT FIFO' in text:
    print('safety=motion_fault_seen_do_not_enable_motion')
PY

say "Finished"
printf 'Report: %s\nRaw console: %s\n' "$REPORT" "$OUT_DIR/console.log"
printf 'Motion was never enabled by this script.\n'
