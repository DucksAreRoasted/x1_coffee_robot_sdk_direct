#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <string>

namespace x1_coffee_hardware_core {
// 全部采用关节侧 SI 单位；驱动器电流单位只在传输边界转换。
struct MITCommand {
  double q_des{0}, dq_des{0}, kp{0}, kd{0}, tau_ff{0};
};
struct MITJointState {
  double q{0}, dq{0}, tau{0};
  bool valid{false};
};
struct MITLimits {
  double q_min{0}, q_max{0}, velocity{0}, torque{0}, torque_rate{0};
  double kp_max{0}, kd_max{0};
  double q_rate{0}, dq_rate{0}, kp_rate{0}, kd_rate{0}, ff_rate{0};
};
struct MITConversion {
  // 带方向的输出端 Nm/千分比；不得自动从 0x6076 推断。
  double nm_per_permille{0};
  double max_permille{0};
  bool verified{false};
  bool valid() const noexcept;
  double toNm(double raw) const noexcept;
  bool toPdo(double nm, std::int16_t * raw) const noexcept;
};
enum class MITFault : std::uint8_t {
  none, configuration, input, position, velocity, measured_torque,
  communication, drive, mode, timing, lease, contention, external_stop
};
const char * mitFaultName(MITFault fault) noexcept;
struct MITResult {
  MITFault fault{MITFault::none};
  double torque{0};
  bool saturated{false};
};
class MITController {
public:
  bool configure(const MITLimits & limits) noexcept;
  bool accepts(const MITCommand & command) const noexcept;
  bool seed(const MITJointState & state) noexcept;
  MITResult compute(const MITCommand & command, const MITJointState & state, double dt) noexcept;
  const MITCommand & applied() const noexcept {return applied_;}
  static bool initialGains(double inertia, double damping, double omega, double zeta,
    double * kp, double * kd) noexcept;
private:
  MITLimits limits_{};
  MITCommand applied_{};
  double previous_{0};
  bool ready_{false}, seeded_{false};
};

constexpr std::size_t kMITJoints = 14;
using MITCommands = std::array<MITCommand, kMITJoints>;
using MITStates = std::array<MITJointState, kMITJoints>;
struct MITConfig {
  std::array<MITLimits, kMITJoints> limits{};
  std::array<MITConversion, kMITJoints> conversion{};
  MITCommands initial{};
  // 示例为未确认状态；验证记录由非实时加载器保存，不在周期中处理字符串。
  bool transition_verified{false};
  double max_dt{0}, lease_sec{0}, mode_timeout{0}, quiet_velocity{0}, quiet_sec{0};
};
bool loadMITConfig(const std::string & path, MITConfig * out, std::string * error);

// 一次更新携带整组参数和递增序号。生产者由上层串行化；消费者只有 EtherCAT 周期线程。
struct MITFrame {MITCommands commands{}; std::uint64_t sequence{0}; double stamp{0};};
class MITMailbox {
public:
  bool push(const MITFrame & frame) noexcept;
  bool pop(MITFrame * frame) noexcept;
private:
  static_assert(std::atomic<unsigned>::is_always_lock_free, "需要无锁索引");
  std::array<MITFrame, 4> frames_{};
  std::atomic<unsigned> read_{0}, write_{0};
};

// 动力学/摩擦/辨识接口：外部计算的 Nm 前馈写入 tau_ff，经同一平滑及预算约束。
// Pinocchio 等提供者应在非实时侧运行；不得把未知分配行为的回调放进周期线程。
enum class MITPhase : std::uint8_t {idle, entering, active, holding, held, fault};
struct MITFeedback {
  MITStates joints{};
  std::array<std::uint8_t, kMITJoints> mode{};
  bool bus_op{false}, all_enabled{false}, all_valid{false}, waist_csp{false};
};
struct MITOutput {
  std::array<std::int16_t, kMITJoints> torque{};
  std::uint16_t cst_mask{0}; // bit 0..13 = l1..l7,r1..r7；腰不在此掩码中。
  bool shutdown{false};
};
struct MITStatus {
  MITPhase phase{MITPhase::idle};
  MITFault fault{MITFault::none};
  int joint{-1};
  std::uint16_t saturated{0};
  std::uint64_t sequence{0};
};
class MITSession {
public:
  bool configure(const MITConfig & config) noexcept;
  bool start(std::uint16_t mask, const MITFeedback & feedback, double now) noexcept;
  bool submit(const MITFrame & frame, double now) noexcept;
  void requestHold(MITFault reason = MITFault::none, int joint = -1) noexcept;
  void trip(MITFault fault, int joint = -1) noexcept;
  MITOutput step(const MITFeedback & feedback, double now) noexcept;
  MITStatus status() const noexcept {return status_;}
  // 故障只能在上层确认重新初始化后重建会话，不能通过参数更新恢复。
private:
  bool healthy(const MITFeedback & feedback) const noexcept;
  MITConfig config_{};
  std::array<MITController, kMITJoints> controllers_{};
  MITCommands target_{};
  std::array<double, kMITJoints> seed_torque_{};
  MITStatus status_{};
  std::uint16_t mask_{0};
  double previous_time_{0}, phase_time_{0}, lease_time_{0}, quiet_time_{0};
  bool configured_{false};
};
}
