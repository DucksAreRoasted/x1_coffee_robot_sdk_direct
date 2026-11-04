#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "x1_coffee_hardware_core/gtn_transport.hpp"
#include "x1_coffee_hardware_core/urdf_gravity.hpp"

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
  // 运行中改指定轴的 0x6060。axis_mask 的 bit i 对应轴 i。替身什么也不做。
  virtual void requestDriveMode(std::uint8_t mode, std::uint16_t axis_mask)
  {
    (void)mode;
    (void)axis_mask;
  }
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
  // 停在当前角度，主站按位置误差写 0x6071。退出后回到周期同步位置。
  Result enterMitHold();
  Result exitMitHold();
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
  std::int16_t mitTorque(std::size_t index, std::int32_t actual_pulse);

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
  // eRob 手册 5.1.1：上电到伺服准备好。实机先等 5 秒再接通，单测为 0。
  int servo_ready_delay_ms_{0};
  std::chrono::steady_clock::time_point servo_wait_begin_{};
  int servo_wait_logged_sec_{-1};
  bool servo_wait_done_logged_{false};
  // 主站 MIT。扭矩单位是额定电流的千分比，1000 即额定的 100%。
  bool mit_active_{false};
  // 单人调试只松开右臂 r1–r7，bit 8..14。左臂和腰保持周期同步位置。
  std::uint16_t mit_mask_{0x7F00};
  double mit_kp_{480.0};
  double mit_kd_{24.0};
  double mit_limit_{1000.0};
  // URDF 静重力，单位 N·m。换成 0x6071 千分比时除以额定力矩。额定力矩尚未从关节读出，先用 50 N·m。
  UrdfGravity gravity_;
  bool gravity_ready_{false};
  double rated_torque_nm_{50.0};
  double gravity_scale_{1.0};
  std::array<double, kCoffeeAxisCount> mit_gravity_permille_{};
  std::array<std::int32_t, kCoffeeAxisCount> mit_hold_pulse_{};
  std::array<double, kCoffeeAxisCount> mit_prev_rad_{};
  std::array<double, kCoffeeAxisCount> mit_velocity_{};
  std::array<bool, kCoffeeAxisCount> mit_have_sample_{};
  mutable std::mutex sample_mu_;
};

}  // namespace x1_coffee_hardware_core
