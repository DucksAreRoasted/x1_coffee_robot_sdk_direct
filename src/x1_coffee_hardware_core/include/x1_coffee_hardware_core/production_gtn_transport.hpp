#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "x1_coffee_hardware_core/gtn_native_api.hpp"
#include "x1_coffee_hardware_core/gtn_transport.hpp"

namespace x1_coffee_hardware_core
{

class ProductionGtnTransport final : public GtnTransport
{
public:
  ProductionGtnTransport();
  explicit ProductionGtnTransport(std::unique_ptr<GtnNativeApi> native_api);
  ~ProductionGtnTransport() override;

  Result configure(const HardwareParameters & parameters) override;
  Result activate() override;
  Result deactivate() override;
  Result read(HardwareSnapshot * snapshot) override;
  Result write(const PositionCommandFrame & command) override;
  Result stop(bool emergency) override;
  Result startMotion() override;
  Result enterZeroTorque() override;
  Result requestZeroTorque() override;
  Result engageBrakes() override;
  Result requestEngageBrakes() override;
  using GtnTransport::requestMovePvt;
  Result requestMovePvt(
    const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
    double duration_sec) override;
  bool controllerClosed() const override;
  bool consumeBlockingCardWork() override;

private:
  enum class State {kUnconfigured, kConfigured, kActive};

  struct AxisCalibration
  {
    double pulses_per_unit{0.0};
    double zero_offset_pulse{0.0};
    double model_zero{0.0};
    std::int32_t wrap_period_pulse{0};
  };

  Result parseParameters(const HardwareParameters & parameters);
  Result initializeController();
  std::filesystem::path gecatLogPath() const;
  std::uintmax_t currentGecatLogSize() const;
  Result waitForAllSlavesOp();
  Result verifyConfiguredAxisBus(const char * stage);
  Result enableMotionAndStartPt(const HardwareSnapshot & initial_snapshot);
  Result enableDrives();
  void serviceCardRequest();
  Result dispatchCardRequest(int request);
  Result applyZeroTorque();
  Result applyBrakeHold();
  Result applyPvtMove();
  Result nativeFailure(CoreError error, const char * api, short code, int axis = 0) const;
  Result failActivation(const Result & cause);
  double pulseToPosition(std::size_t index, std::int32_t pulse) const;
  Result positionToPulse(std::size_t index, double position, double * pulse) const;

  std::unique_ptr<GtnNativeApi> native_api_;
  State state_{State::kUnconfigured};
  short core_{1};
  int ready_retries_{50};
  int ready_interval_ms_{100};
  // 手册例程 5-1 没有步间延时。这两个参数保留是为了不改已编译插件的成员布局，
  // 实机配置保持 0。ecat_init_attempts 只在 GTN_InitEcatCommEx 返回 -10（未找到从站）时使用。
  int ecat_step_delay_ms_{0};
  int ecat_start_delay_ms_{0};
  int ecat_init_attempts_{1};
  // 0 关闭等待，单元测试没有 Gecat.log。实机配置在 xacro 里打开。
  int ecat_op_wait_ms_{0};
  std::uintmax_t op_log_offset_{0};
  int pt_cycle_ms_{10};
  int pt_prebuffer_points_{8};
  // Diagnostics are intentionally much slower than the safety-critical
  // status/encoder path.  A value of 20 means approximately 5 Hz when the
  // controller manager is running at 100 Hz.
  int diagnostic_poll_div_{20};
  bool read_current_torque_{true};
  int timing_log_div_{50};
  bool motion_enabled_{false};
  // startMotion 之前为 false。activate 返回时也必须是 false，即使 motion_enabled 为真。
  std::atomic<bool> motion_armed_{false};
  bool zero_torque_{false};
  std::atomic<bool> blocking_card_work_{false};
  int pt_fifo_stall_{0};
  std::mutex request_mutex_;
  std::condition_variable request_cv_;
  int card_request_{0};
  bool request_done_{false};
  Result request_result_{};
  std::vector<std::array<double, kCoffeeAxisCount>> pvt_waypoints_{};
  double pvt_duration_sec_{0.0};
  // 上电使能，不等于允许运动。默认关闭，避免普通启动去 GTN_AxisOn。
  bool drives_enabled_{false};
  // Fail-closed: only a successful new activation clears a stop latch.
  bool stop_latched_{false};
  bool emergency_latched_{false};
  bool controller_open_{false};
  // 本次开卡已经 Close。之后 stop 不再调用 GTN_Stop / AxisOff / Close。
  bool shutdown_complete_{false};
  // 本次开卡已经调用过 GTN_Stop。deactivate 不再补一次，避免 -8 之后段错误。
  bool native_stop_sent_{false};
  bool ecat_initialized_{false};
  bool ecat_started_{false};
  bool axes_enabled_{false};
  bool pt_started_{false};
  HardwareSnapshot startup_snapshot_{};
  bool has_startup_snapshot_{false};
  const HardwareLayout * layout_{&defaultHardwareLayout()};
  std::int32_t all_axes_mask_{0x7FFF};
  std::int32_t pt_time_ms_{0};
  std::string library_path_;
  std::string gtn_config_path_;
  std::string ecat_config_path_;
  std::array<AxisCalibration, kCoffeeAxisCount> calibration_{};
  std::array<double, kCoffeeAxisCount> previous_positions_{};
  std::array<std::int32_t, kCoffeeAxisCount> latest_encoder_pulses_{};
  std::chrono::steady_clock::time_point previous_sample_time_{};
  bool has_previous_sample_{false};
  std::array<std::int16_t, kCoffeeAxisCount> latest_currents_{};
  std::array<std::int16_t, kCoffeeAxisCount> latest_torques_{};
  std::array<bool, kCoffeeAxisCount> latest_current_valid_{};
  std::array<bool, kCoffeeAxisCount> latest_torque_valid_{};
  bool has_diagnostic_sample_{false};
  std::uint64_t diagnostic_poll_tick_{0};
  std::uint64_t read_timing_tick_{0};
};

}  // namespace x1_coffee_hardware_core
