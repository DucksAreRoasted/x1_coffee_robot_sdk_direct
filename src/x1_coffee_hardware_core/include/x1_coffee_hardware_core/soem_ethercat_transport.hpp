#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "x1_coffee_hardware_core/gtn_transport.hpp"

namespace x1_coffee_hardware_core
{

struct SlaveFeedback
{
  bool valid{false};
  std::uint16_t status_word{0};
  std::int32_t actual_position{0};
  std::int16_t actual_torque{0};
  std::int16_t actual_current{0};
};

struct SlaveCommand
{
  std::uint16_t control_word{0};
  std::int32_t target_position{0};
  std::int16_t target_torque{0};
};

// 生产主站和单测替身都走这个口。替身不打开网口。
class EthercatLink
{
public:
  using Cycle = std::function<void (
      const std::array<SlaveFeedback, kCoffeeAxisCount> &,
      std::array<SlaveCommand, kCoffeeAxisCount> *)>;

  virtual ~EthercatLink() = default;
  virtual Result openAndReachOp(
    const std::string & ifname,
    const std::array<bool, kCoffeeAxisCount> & torque_axis,
    const Cycle & on_cycle) = 0;
  virtual Result shutdownToInit() = 0;
};

// 笔记本网口上的 SOEM 主站。不加载固高库。
class SoemEthercatTransport final : public GtnTransport
{
public:
  SoemEthercatTransport();
  explicit SoemEthercatTransport(std::unique_ptr<EthercatLink> link);
  ~SoemEthercatTransport() override;

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
  Result requestMovePvt(
    const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
    double duration_sec) override;
  // 每个路点带自己的段时间。15 轴用同一时刻插补，中间点不停。
  Result requestMovePvtSegments(
    const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
    const std::vector<double> & segment_sec);
  bool controllerClosed() const override;

private:
  struct AxisCalibration
  {
    double pulses_per_unit{0.0};
    double zero_offset_pulse{0.0};
    double model_zero{0.0};
    std::int32_t wrap_period_pulse{0};
  };

  void onCycle(
    const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
    std::array<SlaveCommand, kCoffeeAxisCount> * command);
  double pulseToPosition(std::size_t index, std::int32_t pulse) const;
  Result positionToPulse(
    std::size_t index, double position, std::int32_t current_pulse, double * pulse) const;

  std::unique_ptr<EthercatLink> link_;
  const HardwareLayout * layout_{nullptr};
  std::array<AxisCalibration, kCoffeeAxisCount> calibration_{};
  std::array<bool, kCoffeeAxisCount> torque_axis_{};
  std::array<bool, kCoffeeAxisCount> held_{};
  std::array<std::int32_t, kCoffeeAxisCount> held_pulse_{};
  std::array<SlaveFeedback, kCoffeeAxisCount> latest_{};
  std::array<double, kCoffeeAxisCount> previous_position_{};
  bool has_previous_position_{false};
  std::string ifname_;
  bool configured_{false};
  bool active_{false};
  bool closed_{false};
  bool drives_enabled_{false};
  bool motion_armed_{false};
  bool shutdown_outputs_{false};
  // 周期同步位置的绝对目标。过程数据里的 0x607A 每个周期只向它走近一个速度上限。
  std::array<std::int32_t, kCoffeeAxisCount> goal_pulse_{};
  std::array<double, kCoffeeAxisCount> max_velocity_rad_s_{};
  std::array<int, kCoffeeAxisCount> velocity_step_{};
  std::array<int, kCoffeeAxisCount> active_step_{};
  // 协同插补表。time_sec 从当前锁存位置起算，15 轴共用。
  struct CoordinatedKnot
  {
    double time_sec{0.0};
    std::array<double, kCoffeeAxisCount> pulse{};
  };
  std::vector<CoordinatedKnot> path_{};
  int path_cycle_{0};
  bool path_active_{false};
  Result installPath(
    const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
    const std::vector<double> & segment_sec);
  void samplePath();
  // 每台按自己的状态字走接通。停在「准备接通」超过 1 秒就退回关电压，满 300 ms 再试。
  std::array<int, kCoffeeAxisCount> enable_ready_cycles_{};
  std::array<int, kCoffeeAxisCount> enable_backoff_cycles_{};
  std::array<int, kCoffeeAxisCount> enable_retries_{};
  // 0 按状态字走，1 关电压，2 关机并空出 300 ms。已使能的轴保持 0。
  std::array<int, kCoffeeAxisCount> enable_phase_{};
  mutable std::mutex sample_mu_;
};

}  // namespace x1_coffee_hardware_core
