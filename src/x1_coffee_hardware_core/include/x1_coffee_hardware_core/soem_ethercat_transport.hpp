#pragma once

#include <array>
#include <limits>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "x1_coffee_hardware_core/gtn_transport.hpp"
#include "x1_coffee_hardware_core/urdf_gravity.hpp"
#include "x1_coffee_hardware_core/current_gravity_file.hpp"
#include "x1_coffee_hardware_core/mit_handfeel.hpp"

namespace x1_coffee_hardware_core
{

struct SlaveFeedback
{
  bool valid{false};
  std::uint16_t status_word{0};
  std::int32_t actual_position{0};
  std::int16_t actual_torque{0};
  std::int16_t actual_current{0};
  std::uint8_t mode_of_operation{0};
};

struct SlaveCommand
{
  std::uint16_t control_word{0};
  std::int32_t target_position{0};
  std::int16_t target_torque{0};
  std::uint8_t mode_of_operation{0x08};
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
  // 关电压没有改变状态字时记录缓存总线状态，不访问邮箱或自动复位。
  virtual void requestStuckEnableProbe(std::uint16_t axis_mask)
  {
    (void)axis_mask;
  }
  // 实机主站用来通知「15 台是否都已进入运行」。替身不绑定，单测仍按已运行处理。
  virtual void bindBusOperationalFlag(std::atomic<bool> * flag)
  {
    (void)flag;
  }
  // 已经广播请求运行、但还没全部进入时置位。过程数据从这时起按官方节拍发控制字。
  virtual void bindCyclicEnableFlag(std::atomic<bool> * flag)
  {
    (void)flag;
  }
  // 全部进入运行、稳态周期开始之前调用一次。用来起 5 秒伺服准备计时。
  virtual void bindOperationalHook(std::function<void()> hook)
  {
    (void)hook;
  }
  virtual void copyRatedTorqueNm(std::array<double, kCoffeeAxisCount> * out) const
  {
    if (out != nullptr) {
      out->fill(0.0);
    }
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
  Result cancelPvtHold();  // 取消位置轨迹并锁存当前有效反馈，不切换扭矩模式。
  Result startMotion() override;
  Result enterZeroTorque() override;
  Result requestZeroTorque() override;
  Result engageBrakes() override;
  Result requestEngageBrakes() override;
  // 停在当前角度，主站按位置误差写 0x6071。退出后回到周期同步位置。
  Result loadCurrentGravity(const std::string & path);
  Result enterCurrentGravity();
  Result currentGravityStatus();
  Result currentGravityHoldStatus();
  Result enterMitHold();
  Result enterMitHoldMasked(std::uint16_t mask, double ff_scale) override;
  Result enterMitHoldEx(
    std::uint16_t mask, const std::array<double, 7> & kp, const std::array<double, 7> & kd,
    double ff_scale) override;
  Result enterMitHandfeel();
  Result enterMitHandfeelEx(
    const std::array<double, 7> & kp, const std::array<double, 7> & kd, double ff_scale) override;
  Result enterMitHandfeelMasked(
    std::uint16_t mask, const std::array<double, 7> & kp, const std::array<double, 7> & kd,
    double ff_scale) override;
  Result enterMitHandfeelCfg(std::uint16_t mask, const MitHandfeelConfig & config) override;
  // 只切 CST 并把 0x6071 写常数，不接 kp/kd；传 0 表示退出并回到 CSP。
  Result enterCstZeroTorque(std::uint16_t axis_mask, std::int32_t torque_permille = 0) override;
  Result cstZeroStatus(std::uint16_t axis_mask) override;
  // 自动辨识用：单轴力矩斜坡。从 from 以 rate 走�to，关节动起来就记下起动值。
  Result startCstZeroRamp(
    std::uint16_t axis_mask, std::int32_t from_permille, std::int32_t to_permille,
    double rate_permille_per_s) override;
  Result cstZeroRampStatus() override;
  Result updateMitHandfeel(int joint, double q_des, double dq_des, double kp, double kd, double tau_ff);
  Result exitMitHold();
  // 右臂只有重力前馈，kp 和 kd 都是 0。用来手感检验补偿方向和大小。
  Result enterZeroGravity();
  // 右臂自己走到两侧停稳点，记录力矩并拟合。左臂和腰不动。结果先留在内存。
  Result startRightArmGravityCalibration();
  Result resetRightArmGravityToUrdf();
  Result saveRightArmGravity();
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
  std::chrono::steady_clock::time_point previous_read_time_{};
  std::string ifname_;
  bool configured_{false};
  bool active_{false};
  bool closed_{false};
  bool drives_enabled_{false};
  // 真机在逐台确认运行之前为 false，这段只允许关机字。默认 true，单测替身不改它。
  std::atomic<bool> bus_operational_{true};
  // 运行请求已经发出、尚未全部进入。真机从这时起在过程数据里发官方控制字。
  std::atomic<bool> cyclic_enable_{false};
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
  // 位置轴接通拍数。eRob_CSP.cpp：1 到 1000 用固定控制字，之后才按状态字保持。
  int official_enable_step_{0};
  // 力矩轴仍按状态字走接通。位置轴的关电压重试不再使用这些计数。
  std::array<int, kCoffeeAxisCount> enable_ready_cycles_{};
  std::array<int, kCoffeeAxisCount> enable_backoff_cycles_{};
  std::array<int, kCoffeeAxisCount> enable_retries_{};
  // 0 按状态字走，1 关电压直到接通禁止，2 关机并空出 300 ms。已使能的轴保持 0。
  std::array<int, kCoffeeAxisCount> enable_phase_{};
  // 上电使能不保证一次成功。每 2 秒打印仍未使能的轴，不改变控制字。
  void logEnableProgress(
    const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
    const std::array<SlaveCommand, kCoffeeAxisCount> & command);
  int enable_debug_cycles_{0};
  std::uint16_t enabled_mask_seen_{0};
  bool all_enabled_logged_{false};
  std::array<bool, kCoffeeAxisCount> enable_probe_sent_{};
  // eRob 手册 5.1.1：上电到伺服准备好。实机先等 5 秒再接通，单测为 0。
  int servo_ready_delay_ms_{0};
  std::chrono::steady_clock::time_point servo_wait_begin_{};
  int servo_wait_logged_sec_{-1};
  bool servo_wait_done_logged_{false};
  // 主站 MIT。扭矩单位是额定电流的千分比，1000 即额定的 100%。
  CurrentGravityBundle current_bundle_;
  std::array<CurrentGravity, 2> current_models_;
  std::array<CurrentGravity::Arm, 2> current_previous_{};
  CurrentGravity::Pose current_previous_q_{};
  CurrentGravity::Pose current_velocity_{};
  bool current_loaded_{false};
  bool current_active_{false};
  int current_mode_wait_{0};
  bool current_modes_confirmed_{false};
  int current_fault_{0};  // 1反馈、2MIT控制器、3模式、4电流、5软限位、6速度、7模型输出。
  int current_fault_joint_{-1};
  std::array<int, kCoffeeAxisCount> current_over_limit_cycles_{};
  std::chrono::steady_clock::time_point last_cycle_at_{};
  void updateCurrentGravity(const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback);
  bool mit_active_{false};
  bool mit_zero_g_{false};
  // 单人调试只松开右臂 r1–r7，bit 8..14。左臂和腰保持周期同步位置。
  std::uint16_t mit_mask_{0x7F00};
  // 零力矩 CST 验证的轴掩码，非 0 时这些轴按 CST 发，力矩为下面的常值偏置。
  std::uint16_t cst_zero_mask_{0};
  // 力矩斜坡状态：ramp_now_ 每个周期推进，ramp_onset_ 记录关节开始运动的扭矩。
  bool ramp_active_{false};
  bool ramp_done_{false};
  std::size_t ramp_axis_{0};
  double ramp_rate_{0.0};
  double ramp_now_{0.0};
  std::int32_t ramp_from_{0};
  std::int32_t ramp_to_{0};
  double ramp_q0_{0.0};
  std::int32_t ramp_onset_{std::numeric_limits<std::int32_t>::min()};
  int ramp_cycles_{0};
  // 起动判据：相对斜坡起点的位移超过这个值就算动起来了（约 0.6°）。
  static constexpr double kRampMotionRad = 0.01;
  // 第 4 步的小力矩，单位额定千分比，限 ±5%（±50‰），只作用于掩码内轴。
  std::int32_t cst_zero_torque_permille_{0};
  // 额定力矩的 5%。超过这个数就不是「小力矩验证」了。
  static constexpr std::int32_t kCstZeroMaxPermille = 50;
  // 逐轴增益，顺序 r1..r7。不同关节的惯量差两个数量级，全局单值没法同时合适。
  std::array<double, 7> mit_kp_{{480.0, 480.0, 480.0, 480.0, 480.0, 480.0, 480.0}};
  std::array<double, 7> mit_kd_{{24.0, 24.0, 24.0, 24.0, 24.0, 24.0, 24.0}};
  // 重力前馈倍率：0 = 纯 PD 手感试验，1 = 正常前馈。
  double mit_ff_scale_{1.0};
  double mit_limit_{1000.0};
  // URDF 静重力，单位 N·m。换成 0x6071 时只除以该轴读到的 0x6076。
  MitHandfeel handfeel_;
  bool current_handfeel_{false};
  // 正式零重力和 MIT 手感均允许离开进入时姿态；安全边界由轴级保护负责。
  bool current_zero_gravity_{false};
  UrdfGravity gravity_;
  bool gravity_ready_{false};
  double rated_torque_nm_{50.0};
  double gravity_scale_{1.0};
  std::array<double, kCoffeeAxisCount> rated_axis_nm_{};
  std::uint16_t gravity_unscaled_mask_{0};
  bool rightArmRatedTorqueReady(std::string * missing) const;
  std::string right_arm_gravity_yaml_;
  std::array<double, 7> right_mass_scale_{};
  std::array<std::array<double, 3>, 7> right_com_delta_{};
  // 连杆模型之后的小修正。周期同步扭矩里加进 0x6071，不再单独映射 0x60B2。
  std::array<double, 7> right_joint_scale_{};
  std::array<double, 7> right_joint_bias_{};
  // 零差形式的单关节残余幅值，单位 N·m。右臂来自重力文件的 sin_amplitude，默认 0。
  std::array<double, kCoffeeAxisCount> gravity_amplitude_nm_{};
  bool right_correction_known_{false};
  void rememberRightArmCorrection(
    const std::array<double, 7> & mass_scale,
    const std::array<std::array<double, 3>, 7> & com_delta);
  // 参数化入口：mask/kp/kd/ff_scale 由调用方给定，zero_gravity 时忽略 kp（只用前馈+阻尼）。
  Result beginRightArmTorque(
    std::uint16_t mask, const std::array<double, 7> & kp_permille,
    const std::array<double, 7> & kd_permille, double ff_scale, bool zero_gravity);
  std::atomic<bool> gravity_calib_stop_{false};
  std::atomic<bool> gravity_calib_running_{false};
  std::thread gravity_calib_thread_;
  void runRightArmGravityCalibration();
  bool writeRightArmGravityFile(
    const std::array<double, 7> & mass_scale,
    const std::array<std::array<double, 3>, 7> & com_delta) const;
  std::array<double, kCoffeeAxisCount> mit_gravity_nm_{};
  std::array<double, kCoffeeAxisCount> mit_gravity_permille_{};
  std::array<double, kCoffeeAxisCount> mit_qdes_rad_{};
  std::array<double, 7> mit_balance_nm_{};
  std::array<bool, 7> mit_balance_ok_{};
  // 标定软持时由主站给出目标角。平时 MIT 仍锁存按下瞬间的编码器。
  bool mit_track_qdes_{false};
  bool mit_soft_hold_{false};
  std::array<std::int32_t, kCoffeeAxisCount> mit_hold_pulse_{};
  std::array<double, kCoffeeAxisCount> mit_prev_rad_{};
  std::array<double, kCoffeeAxisCount> mit_velocity_{};
  std::array<bool, kCoffeeAxisCount> mit_have_sample_{};
  mutable std::mutex sample_mu_;
};

}  // namespace x1_coffee_hardware_core
