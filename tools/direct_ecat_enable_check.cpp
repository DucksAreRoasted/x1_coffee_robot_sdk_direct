#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"

namespace core = x1_coffee_hardware_core;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void signalHandler(int) {interrupted = 1;}

// 本程序只使用 SDK 的保持当前位置流程，不调用 startMotion/write/PVT/零力矩接口。
int main(int argc, char ** argv)
{
  bool enable = false;
  std::string ifname, parameter_file;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--enable") {enable = true;}
    else if (arg == "--ifname" && i + 1 < argc) {ifname = argv[++i];}
    else if (arg == "--parameters" && i + 1 < argc) {parameter_file = argv[++i];}
    else {std::fprintf(stderr, "Unknown/incomplete argument: %s\n", arg.c_str()); return 64;}
  }
  // 固定使用已经确认的专用网口，避免误碰 SSH 网口。
  if (ifname != "enp3s0" || parameter_file.empty()) {
    std::fprintf(stderr, "Required: --ifname enp3s0 --parameters FILE [--enable]\n");
    return 64;
  }
  core::HardwareParameters parameters;
  std::ifstream input(parameter_file);
  std::string key, value;
  while (input >> key >> value) {parameters[key] = value;}
  if (!input.eof() || parameters.empty()) {std::fprintf(stderr, "Invalid parameter file\n"); return 64;}
  parameters["hardware_profile"] = "coffee_15_no_slide";
  parameters["axis_count"] = "15";
  parameters["ethercat_ifname"] = ifname;
  parameters["drives_enabled"] = enable ? "true" : "false";
  parameters["zero_torque_axes"] = "";
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  core::SoemEthercatTransport transport;
  auto result = transport.configure(parameters);
  if (!result) {std::fprintf(stderr, "CONFIGURE_FAILED %s\n", result.detail.c_str()); return 2;}
  std::printf("BEGIN ifname=%s drives_enabled=%d motion_commands=none\n", ifname.c_str(), enable);
  std::fflush(stdout);
  result = transport.activate();
  if (!result) {
    std::fprintf(stderr, "ACTIVATE_FAILED code=%u detail=%s\n", static_cast<unsigned>(result.error), result.detail.c_str());
    transport.deactivate();
    return 2;
  }
  std::printf("BUS_OP_CONFIRMED\n");
  bool passed = false;
  bool failed = false;
  bool baseline_set = false;
  core::HardwareSnapshot baseline{}, latest{};
  std::array<std::uint32_t, core::kCoffeeAxisCount> previous_status{};
  double maximum_drift = 0.0;
  const auto started = Clock::now();
  auto all_enabled_since = Clock::time_point{};
  auto next_summary = started;
  while (!interrupted && Clock::now() - started < std::chrono::seconds(enable ? 25 : 3)) {
    result = transport.read(&latest);
    if (!result) {
      std::fprintf(stderr, "READ_FAILED code=%u detail=%s\n", static_cast<unsigned>(result.error), result.detail.c_str());
      failed = true;
      break;
    }
    if (!baseline_set) {baseline = latest; baseline_set = true;}
    int enabled_count = 0, alarms = 0;
    for (std::size_t i = 0; i < latest.axes.size(); ++i) {
      const auto & axis = latest.axes[i];
      const std::string joint(core::coffeeAxisDescriptors()[i].joint_name);
      const double wrap = std::stod(parameters.at("wrap_period_pulse." + joint));
      double delta = static_cast<double>(axis.encoder_pulse_raw - baseline.axes[i].encoder_pulse_raw);
      if (wrap > 0.0) {delta = std::remainder(delta, wrap);}
      const double drift = std::abs(delta / std::stod(parameters.at("pulses_per_unit." + joint)));
      maximum_drift = std::max(maximum_drift, drift);
      enabled_count += axis.enabled;
      alarms += axis.alarm;
      if (previous_status[i] != axis.status_bits || next_summary == started) {
        std::printf("AXIS slave=%zu joint=%s status=0x%04x enabled=%d alarm=%d pulse=%lld drift_rad=%.9f\n",
          i + 1, joint.c_str(), axis.status_bits, axis.enabled, axis.alarm,
          static_cast<long long>(axis.encoder_pulse_raw), drift);
        previous_status[i] = axis.status_bits;
      }
      if (!axis.position_valid || !axis.encoder_valid || axis.alarm || drift > 0.01) {failed = true;}
    }
    const auto now = Clock::now();
    if (now >= next_summary) {
      std::printf("SAMPLE elapsed_s=%.3f enabled=%d/15 alarms=%d max_drift_rad=%.9f\n",
        std::chrono::duration<double>(now - started).count(), enabled_count, alarms, maximum_drift);
      std::fflush(stdout);
      next_summary = now + std::chrono::seconds(1);
    }
    // 检出报警或异常位移立即终止，不在此工具里清报警重试。
    if (failed) {std::fprintf(stderr, "FEEDBACK_CHECK_FAILED\n"); break;}
    if (enable && enabled_count == 15) {
      if (all_enabled_since == Clock::time_point{}) {all_enabled_since = now;}
      if (now - all_enabled_since >= std::chrono::seconds(2)) {passed = true; break;}
    } else {all_enabled_since = {};}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!enable && !failed && !interrupted && baseline_set) {passed = true;}
  // 先在通信周期仍运行时下使能并确认，再关闭主站，避免直接退出丢失下使能帧。
  const auto stopped = transport.stop(false);
  bool disabled_confirmed = false;
  const auto stop_deadline = Clock::now() + std::chrono::seconds(2);
  while (Clock::now() < stop_deadline) {
    core::HardwareSnapshot snapshot;
    const auto read = transport.read(&snapshot);
    if (read && std::none_of(snapshot.axes.begin(), snapshot.axes.end(), [](const auto & a) {return a.enabled;})) {
      disabled_confirmed = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  // 手册规定刹车动作约 150 ms，留出余量后退出周期通信。
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  const auto closed = transport.deactivate();
  std::printf("CLEANUP stop_ok=%d all_disabled_confirmed=%d deactivate_ok=%d\n",
    static_cast<bool>(stopped), disabled_confirmed, static_cast<bool>(closed));
  std::printf("RESULT mode=%s passed=%d all_enabled_held_2s=%d max_drift_rad=%.9f interrupted=%d\n",
    enable ? "enable_hold" : "read_only", passed, enable && passed, maximum_drift, interrupted != 0);
  std::fflush(stdout);
  if (!stopped || !closed || !disabled_confirmed) {return 3;}
  return passed ? 0 : 2;
}
