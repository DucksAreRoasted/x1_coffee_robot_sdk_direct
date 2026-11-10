#include <algorithm>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"
#include "../src/enable_trace.hpp"
#include "../src/ethercat_startup.hpp"

namespace
{

using x1_coffee_hardware_core::CoreError;
using x1_coffee_hardware_core::EthercatLink;
using x1_coffee_hardware_core::HardwareParameters;
using x1_coffee_hardware_core::HardwareSnapshot;
using x1_coffee_hardware_core::PositionCommandFrame;
using x1_coffee_hardware_core::SlaveCommand;
using x1_coffee_hardware_core::SlaveFeedback;
using x1_coffee_hardware_core::SoemEthercatTransport;
using x1_coffee_hardware_core::coffeeAxisDescriptors;
using x1_coffee_hardware_core::kCoffeeAxisCount;

class FakeLink final : public EthercatLink
{
public:
  std::uint16_t status{0x1250};
  std::array<SlaveCommand, kCoffeeAxisCount> last{};
  int shutdowns{0};
  bool feedback_valid{true};
  std::atomic<bool> * bus_operational{nullptr};

  void bindBusOperationalFlag(std::atomic<bool> * flag) override
  {
    bus_operational = flag;
  }
  Cycle cycle;

  x1_coffee_hardware_core::Result openAndReachOp(
    const std::string &,
    const std::array<bool, kCoffeeAxisCount> &,
    const Cycle & on_cycle) override
  {
    cycle = on_cycle;
    replay();
    return x1_coffee_hardware_core::Result::ok();
  }

  void replay()
  {
    std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
    for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
      feedback[index].valid = feedback_valid;
      feedback[index].status_word = status;
      feedback[index].actual_position = 1000 * static_cast<std::int32_t>(index + 1);
    }
    cycle(feedback, &last);
  }

  x1_coffee_hardware_core::Result shutdownToInit() override
  {
    ++shutdowns;
    return x1_coffee_hardware_core::Result::ok();
  }
};

HardwareParameters parameters(bool drives_enabled, const std::string & zero_torque_axes = "")
{
  HardwareParameters values{
    {"hardware_profile", "coffee_15_no_slide"},
    {"axis_count", "15"},
    {"ethercat_ifname", "eno1"},
    {"drives_enabled", drives_enabled ? "true" : "false"},
  };
  if (!zero_torque_axes.empty()) {
    values["zero_torque_axes"] = zero_torque_axes;
  }
  for (const auto & axis : coffeeAxisDescriptors()) {
    const std::string joint(axis.joint_name);
    values["pulses_per_unit." + joint] = "1000";
    values["zero_offset_pulse." + joint] = "0";
    values["model_zero." + joint] = "0";
    values["wrap_period_pulse." + joint] = "0";
  }
  return values;
}

// 回归覆盖寄存器保留字、错误状态和不完整工作计数，全部使用内存数据。
TEST(EthercatStartup, reads_al_code_after_reserved_register_and_decodes_little_endian)
{
  const auto al = x1_coffee_hardware_core::detail::decodeAlStatus(
    {0x14, 0x00, 0xAB, 0xCD, 0x30, 0x12});
  EXPECT_EQ(al.state, 0x0014);
  EXPECT_EQ(al.code, 0x1230);
}

TEST(EthercatStartup, error_and_unknown_states_cannot_confirm_operational)
{
  using x1_coffee_hardware_core::detail::alStateReady;
  EXPECT_TRUE(alStateReady(0x04, 0x04));
  EXPECT_TRUE(alStateReady(0x08, 0x04));
  EXPECT_TRUE(alStateReady(0x08, 0x08));
  for (const std::uint16_t state : {0x00, 0x02, 0x14, 0x18, 0x0F}) {
    EXPECT_FALSE(alStateReady(state, 0x04));
    EXPECT_FALSE(alStateReady(state, 0x08));
  }
  EXPECT_FALSE(alStateReady(0x04, 0x08));
}

TEST(EthercatStartup, send_phase_stays_ahead_of_sync0)
{
  using x1_coffee_hardware_core::detail::outputReadyBeforeSync;
  using x1_coffee_hardware_core::detail::syncLeadErrorNs;
  constexpr std::int64_t cycle = 1000000;
  constexpr std::int64_t lead = 500000;
  EXPECT_EQ(syncLeadErrorNs(500000, cycle, lead), 0);
  EXPECT_EQ(syncLeadErrorNs(1500000, cycle, lead), 0);
  EXPECT_EQ(syncLeadErrorNs(1000000, cycle, lead), 500000);
  EXPECT_TRUE(outputReadyBeforeSync(500000, cycle, lead));
  EXPECT_TRUE(outputReadyBeforeSync(900000, cycle, lead));
  EXPECT_FALSE(outputReadyBeforeSync(100000, cycle, lead));
  EXPECT_FALSE(outputReadyBeforeSync(0, cycle, lead));
}

TEST(EthercatStartup, system_time_difference_uses_sign_magnitude)
{
  using x1_coffee_hardware_core::detail::systemTimeDifferenceNs;
  EXPECT_EQ(systemTimeDifferenceNs(0), 0);
  EXPECT_EQ(systemTimeDifferenceNs(8), 8);
  EXPECT_EQ(systemTimeDifferenceNs(0x8000000au), -10);
  EXPECT_EQ(systemTimeDifferenceNs(0x80000099u), -153);
}

TEST(EthercatStartup, partial_or_missing_process_data_is_invalid)
{
  using x1_coffee_hardware_core::detail::completeProcessData;
  EXPECT_TRUE(completeProcessData(45, 45));
  for (const int wkc : {-1, 0, 1, 15, 43, 44, 46}) {
    EXPECT_FALSE(completeProcessData(wkc, 45));
  }
  EXPECT_FALSE(completeProcessData(0, 0));
}

TEST(SoemEthercatTransport, invalid_feedback_does_not_enable_or_advance_retry)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1231;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  raw->feedback_valid = false;
  for (int cycle = 0; cycle < 1600; ++cycle) {
    raw->replay();
    EXPECT_EQ(raw->last[0].control_word, 0x0000);
    EXPECT_EQ(raw->last[0].target_torque, 0);
  }
  HardwareSnapshot snapshot;
  EXPECT_FALSE(transport.read(&snapshot));
  raw->feedback_valid = true;
  raw->replay();
  // 无效帧不推进官方节拍。下一拍仍在 400 ms 复位段。
  EXPECT_EQ(raw->last[0].control_word, 0x0080);
  EXPECT_EQ(raw->last[0].target_position, 1000);
}

TEST(SoemEthercatTransport, first_invalid_feedback_does_not_latch_position)
{
  auto link = std::make_unique<FakeLink>();
  link->feedback_valid = false;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->last[0].target_position, 0);
  EXPECT_EQ(raw->last[0].control_word, 0);
  HardwareSnapshot snapshot;
  EXPECT_FALSE(transport.read(&snapshot));
  raw->feedback_valid = true;
  raw->replay();
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_EQ(raw->last[0].target_position, 1000);
}

TEST(SoemEthercatTransport, faults_never_emit_automatic_reset_in_csp_or_cst)
{
  for (const std::uint16_t fault : {0x1208, 0x1288, 0x120F}) {
    auto link = std::make_unique<FakeLink>();
    link->status = 0x1237;
    FakeLink * raw = link.get();
    SoemEthercatTransport transport(std::move(link));
    ASSERT_TRUE(transport.configure(parameters(true, "1")));
    ASSERT_TRUE(transport.activate());
    raw->status = fault;
    for (int cycle = 0; cycle < 10; ++cycle) {
      raw->replay();
      // 第 1 轴是周期同步扭矩，故障时保持 0。位置轴在官方头 400 ms 发 0x0080。
      EXPECT_EQ(raw->last[0].control_word, 0x0000);
      EXPECT_EQ(raw->last[0].target_torque, 0);
      EXPECT_EQ(raw->last[1].control_word, 0x0080);
    }
    for (int cycle = 0; cycle < 1000; ++cycle) {
      raw->replay();
    }
    raw->replay();
    for (const auto & command : raw->last) {
      EXPECT_EQ(command.control_word, 0x0000);
    }
    HardwareSnapshot snapshot;
    ASSERT_TRUE(transport.read(&snapshot));
    EXPECT_TRUE(snapshot.axes[0].alarm);
    EXPECT_FALSE(snapshot.axes[0].enabled);
  }
}

TEST(SoemEthercatTransport, latched_bus_drop_blocks_enable_and_reports_read_failure)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_NE(raw->bus_operational, nullptr);
  raw->bus_operational->store(false);
  raw->replay();
  for (const auto & command : raw->last) {
    EXPECT_EQ(command.control_word, 0x0006);
    EXPECT_EQ(command.target_torque, 0);
  }
  HardwareSnapshot snapshot;
  EXPECT_FALSE(transport.read(&snapshot));
}

TEST(SoemEthercatTransport, rejects_missing_interface_name)
{
  SoemEthercatTransport transport(std::make_unique<FakeLink>());
  HardwareParameters values = parameters(false);
  values.erase("ethercat_ifname");
  const auto result = transport.configure(values);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, CoreError::kInvalidConfiguration);
}

TEST(SoemEthercatTransport, enable_holds_actual_position_and_not_pulse_zero)
{
  auto link = std::make_unique<FakeLink>();
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->last[0].target_position, 1000);
  EXPECT_EQ(raw->last[0].control_word, 0x0080);
  EXPECT_NE(raw->last[0].target_position, 0);
  HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_DOUBLE_EQ(snapshot.axes[0].position, 1.0);
  EXPECT_EQ(snapshot.axes[0].encoder_pulse_raw, 1000);
}

TEST(SoemEthercatTransport, read_only_keeps_control_word_clear)
{
  auto link = std::make_unique<FakeLink>();
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(false)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->last[0].control_word, 0x0000);
  EXPECT_EQ(raw->last[0].target_position, 1000);
}

TEST(SoemEthercatTransport, zero_torque_axis_uses_cst_word_and_zero_torque)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true, "1")));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->last[0].control_word, 0x000F);
  EXPECT_EQ(raw->last[0].target_torque, 0);
  EXPECT_EQ(raw->last[1].control_word, 0x0080);
  const auto zero = transport.requestZeroTorque();
  EXPECT_TRUE(zero);
  const auto motion = transport.startMotion();
  EXPECT_FALSE(motion);
  EXPECT_EQ(motion.error, CoreError::kWriteFailed);
}

TEST(SoemEthercatTransport, target_tracks_actual_position_until_enabled)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1231;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->last[0].control_word, 0x0080);
  EXPECT_EQ(raw->last[6].target_position, 7000);
  EXPECT_EQ(raw->last[6].control_word, 0x0080);

  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    feedback[index].valid = true;
    feedback[index].status_word = 0x1231;
    feedback[index].actual_position = 5000 + static_cast<std::int32_t>(index);
  }
  raw->cycle(feedback, &raw->last);
  EXPECT_EQ(raw->last[6].target_position, 5006);
  EXPECT_EQ(raw->last[12].target_position, 5012);
  EXPECT_EQ(raw->last[0].control_word, 0x0080);
  EXPECT_EQ(raw->last[6].control_word, 0x0080);
  EXPECT_EQ(raw->last[14].control_word, 0x0080);
}

void finishOfficialEnable(FakeLink * raw)
{
  for (int cycle = 0; cycle < 1000; ++cycle) {
    raw->replay();
  }
}

TEST(SoemEthercatTransport, official_csp_enable_uses_fixed_control_words)
{
  auto link = std::make_unique<FakeLink>();
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->last[0].control_word, 0x0080);
  EXPECT_EQ(raw->last[0].target_position, 1000);

  int reset = 1;
  int shutdown = 0;
  int switch_on = 0;
  int enable = 0;
  for (int cycle = 0; cycle < 999; ++cycle) {
    raw->replay();
    const std::uint16_t word = raw->last[0].control_word;
    EXPECT_EQ(raw->last[0].target_position, 1000);
    if (word == 0x0080) {
      ++reset;
    } else if (word == 0x0006) {
      ++shutdown;
    } else if (word == 0x0007) {
      ++switch_on;
    } else if (word == 0x000F) {
      ++enable;
    }
  }
  EXPECT_EQ(reset, 400);
  EXPECT_EQ(shutdown, 200);
  EXPECT_EQ(switch_on, 200);
  EXPECT_EQ(enable, 200);
}

TEST(SoemEthercatTransport, csp_target_slews_without_a_single_cycle_jump)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  finishOfficialEnable(raw);
  EXPECT_EQ(raw->last[0].target_position, 1000);
  EXPECT_EQ(raw->last[0].control_word, 0x000F);

  std::vector<std::array<double, kCoffeeAxisCount>> waypoints(1);
  waypoints[0].fill(1.0);
  waypoints[0][0] = 2.0;
  const auto moved = transport.requestMovePvt(waypoints, 1.0);
  ASSERT_TRUE(moved);
  raw->replay();
  EXPECT_GT(raw->last[0].target_position, 1000);
  EXPECT_LT(raw->last[0].target_position, 1100);
  EXPECT_EQ(raw->last[0].control_word, 0x000F);
}

TEST(SoemEthercatTransport, coordinated_path_passes_the_middle_knot_before_the_end)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  finishOfficialEnable(raw);
  std::vector<std::array<double, kCoffeeAxisCount>> waypoints(2);
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const double held = static_cast<double>(index + 1);
    waypoints[0][index] = held;
    waypoints[1][index] = held;
  }
  waypoints[0][0] = 1.5;
  waypoints[1][0] = 2.0;
  ASSERT_TRUE(transport.requestMovePvtSegments(waypoints, {0.5, 0.5}));
  // 1 ms 一拍，500 拍是第一段 0.5 秒，再 500 拍到总时间 1 秒。
  for (int cycle = 0; cycle < 500; ++cycle) {
    raw->replay();
  }
  EXPECT_NEAR(raw->last[0].target_position, 1500, 5);
  EXPECT_NEAR(raw->last[1].target_position, 2000, 1);
  for (int cycle = 0; cycle < 500; ++cycle) {
    raw->replay();
  }
  EXPECT_NEAR(raw->last[0].target_position, 2000, 5);
}

TEST(SoemEthercatTransport, move_is_rejected_until_drives_are_enabled)
{
  SoemEthercatTransport transport(std::make_unique<FakeLink>());
  ASSERT_TRUE(transport.configure(parameters(false)));
  ASSERT_TRUE(transport.activate());
  std::vector<std::array<double, kCoffeeAxisCount>> waypoints(1);
  waypoints[0].fill(1.0);
  const auto moved = transport.requestMovePvt(waypoints, 1.0);
  EXPECT_FALSE(moved);
  EXPECT_EQ(moved.error, CoreError::kWriteFailed);
}

TEST(SoemEthercatTransport, write_holds_until_motion_is_armed_then_slews)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  finishOfficialEnable(raw);
  PositionCommandFrame command;
  command.positions.fill(1.0);
  command.positions[0] = 2.0;
  const auto ignored = transport.write(command);
  EXPECT_TRUE(ignored);
  raw->replay();
  EXPECT_EQ(raw->last[0].target_position, 1000);

  const auto armed = transport.startMotion();
  ASSERT_TRUE(armed);
  ASSERT_TRUE(transport.write(command));
  raw->replay();
  EXPECT_GT(raw->last[0].target_position, 1000);
  EXPECT_LT(raw->last[0].target_position, 1100);
  EXPECT_EQ(raw->last[0].control_word, 0x000F);
}

TEST(SoemEthercatTransport, start_motion_requires_enabled_drives)
{
  SoemEthercatTransport transport(std::make_unique<FakeLink>());
  ASSERT_TRUE(transport.configure(parameters(false)));
  ASSERT_TRUE(transport.activate());
  const auto motion = transport.startMotion();
  EXPECT_FALSE(motion);
  EXPECT_EQ(motion.error, CoreError::kWriteFailed);
}

class RetryLink final : public EthercatLink
{
public:
  int opens{0};
  int failures_left{0};
  std::string failure{"从站未全部进入运行"};

  x1_coffee_hardware_core::Result openAndReachOp(
    const std::string &,
    const std::array<bool, kCoffeeAxisCount> &,
    const Cycle &) override
  {
    ++opens;
    if (failures_left > 0) {
      --failures_left;
      return x1_coffee_hardware_core::Result::fail(CoreError::kEthercatNotReady, failure);
    }
    return x1_coffee_hardware_core::Result::ok();
  }

  x1_coffee_hardware_core::Result shutdownToInit() override
  {
    return x1_coffee_hardware_core::Result::ok();
  }
};

TEST(SoemEthercatTransport, retries_safeop_bringup_until_operational)
{
  auto link = std::make_unique<RetryLink>();
  link->failures_left = 2;
  RetryLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(false)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->opens, 3);
}

TEST(SoemEthercatTransport, retries_when_the_failure_only_lists_slave_states)
{
  auto link = std::make_unique<RetryLink>();
  link->failures_left = 1;
  link->failure = "bus state=0x8 late_us=0 phase_us=508 sync0=1 slave13=0x4/0x0";
  RetryLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(false)));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(raw->opens, 2);
}

TEST(SoemEthercatTransport, does_not_retry_a_process_data_size_mismatch)
{
  auto link = std::make_unique<RetryLink>();
  link->failures_left = 3;
  link->failure = "从站 2 的过程数据长度不符";
  RetryLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(false)));
  const auto opened = transport.activate();
  EXPECT_FALSE(opened);
  EXPECT_EQ(raw->opens, 1);
}

TEST(SoemEthercatTransport, deactivate_returns_the_link_to_init)
{
  auto link = std::make_unique<FakeLink>();
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(false)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.deactivate());
  EXPECT_EQ(raw->shutdowns, 1);
  EXPECT_TRUE(transport.controllerClosed());
}

TEST(SoemEthercatTransport, a_stalled_axis_does_not_block_axes_that_are_already_enabled)
{
  for (const std::size_t stalled : {0u, 2u, 8u, 14u}) {
    SCOPED_TRACE(stalled);
    auto link = std::make_unique<FakeLink>();
    FakeLink * raw = link.get();
    SoemEthercatTransport transport(std::move(link));
    ASSERT_TRUE(transport.configure(parameters(true)));
    ASSERT_TRUE(transport.activate());
    finishOfficialEnable(raw);
    std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
    for (auto & item : feedback) {
      item.valid = true;
      item.status_word = 0x1637;
      item.actual_position = 1000;
    }
    for (int tick = 0; tick < 100; ++tick) {
      feedback[stalled].status_word = (tick % 2) == 0 ? 0x1231 : 0x12b1;
      feedback[stalled].actual_position = 1000 + tick % 3;
      raw->cycle(feedback, &raw->last);
    }
    EXPECT_EQ(raw->last[stalled].control_word, 0x0007);
    EXPECT_EQ(raw->last[stalled].target_position, feedback[stalled].actual_position);
    for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
      if (i == stalled) {
        continue;
      }
      EXPECT_EQ(raw->last[i].control_word, 0x000F);
    }
  }
}

TEST(SoemEthercatTransport, mit_hold_pulls_back_toward_the_latched_pose)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
  finishOfficialEnable(raw);
  const auto entered = transport.enterMitHold();
  ASSERT_TRUE(entered) << entered.detail;
  raw->replay();
  EXPECT_EQ(raw->last[8].target_torque, 0);
  EXPECT_EQ(raw->last[0].target_torque, 0);

  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    feedback[index].valid = true;
    feedback[index].status_word = 0x1237;
    feedback[index].actual_position = 1000 * static_cast<std::int32_t>(index + 1);
  }
  feedback[8].actual_position = 9100;
  raw->cycle(feedback, &raw->last);
  EXPECT_LT(raw->last[8].target_torque, 0);
  EXPECT_GE(raw->last[8].target_torque, -1000);
  EXPECT_EQ(raw->last[8].target_position, 9100);
  EXPECT_EQ(raw->last[0].target_torque, 0);
  EXPECT_EQ(raw->last[1].target_torque, 0);

  const auto left = transport.exitMitHold();
  ASSERT_TRUE(left) << left.detail;
  raw->cycle(feedback, &raw->last);
  EXPECT_EQ(raw->last[8].target_torque, 0);
}

TEST(EnableTrace, preserves_previous_submitted_target_and_counts_transient_bus_errors)
{
  x1_coffee_hardware_core::detail::EnableTrace trace;
  x1_coffee_hardware_core::detail::EnableTraceFrame frame;
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  std::array<SlaveCommand, kCoffeeAxisCount> command{};
  feedback[2] = {true, 0x1231, 42};
  command[2].control_word = 7;
  command[2].target_position = 42;
  trace.capture(feedback, command, 100, true, 45, 45, 1, 0);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_EQ(frame.sent[2].control_word, 0);

  command[2].target_position = 43;
  trace.capture(feedback, command, 600, true, 45, 45, 1, 0);
  EXPECT_FALSE(trace.pop(&frame));  // 不因目标每拍小幅变化产生逐拍打印。
  feedback[2].status_word = 0x0231;
  trace.capture(feedback, command, 1100, true, 30, 45, 0, 250000);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_EQ(frame.sent[2].target_position, 43);  // 必须是上一拍，而不是上一条日志。
  EXPECT_EQ(frame.low_wkc_frames, 1u);
  EXPECT_EQ(frame.nonpositive_send_frames, 1u);
  EXPECT_EQ(frame.minimum_wkc, 30);
  EXPECT_EQ(frame.max_cycle_gap_us, 500u);
  EXPECT_EQ(frame.state_age_us[2], 1000u);  // 厂商高位变化不会重置状态停留时间。
  trace.capture(feedback, command, 1600, true, 45, 45, 1, 0);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_EQ(frame.low_wkc_frames, 1u);
  EXPECT_EQ(frame.minimum_wkc, 30);
  EXPECT_FALSE(trace.pop(&frame));
  trace.capture(feedback, command, 2000100, true, 45, 45, 1, 0);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_TRUE(frame.summary);
  EXPECT_EQ(frame.state_age_us[2], 2000000u);
  EXPECT_EQ(frame.raw_status_changes[2], 1u);
}

TEST(EnableTrace, bounded_overflow_does_not_block_and_reports_dropped_frames)
{
  using x1_coffee_hardware_core::detail::EnableTrace;
  EnableTrace trace;
  x1_coffee_hardware_core::detail::EnableTraceFrame frame;
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  std::array<SlaveCommand, kCoffeeAxisCount> command{};
  for (std::size_t tick = 0; tick < EnableTrace::kCapacity + 5; ++tick) {
    command[0].control_word = tick % 2 == 0 ? 6 : 7;
    trace.capture(feedback, command, tick * 500, true, 45, 45, 1, 0);
  }
  EXPECT_EQ(trace.dropped(), 6u);  // 环形队列留一个空槽用于区分空与满。
  std::size_t count = 0;
  while (trace.pop(&frame)) {
    EXPECT_EQ(frame.tick, count++);
  }
  EXPECT_EQ(count, EnableTrace::kCapacity - 1);
  trace.capture(feedback, command, 3000000, true, 45, 45, 1, 0);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_TRUE(frame.summary);
  EXPECT_EQ(frame.dropped, 6u);
}

TEST(EnableTrace, producer_and_consumer_preserve_complete_frames)
{
  x1_coffee_hardware_core::detail::EnableTrace trace;
  std::atomic<bool> done{false};
  constexpr std::uint32_t count = 20000;
  std::thread producer([&]() {
      std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
      std::array<SlaveCommand, kCoffeeAxisCount> command{};
      for (std::uint32_t tick = 0; tick < count; ++tick) {
        command[0].control_word = tick % 2 == 0 ? 6 : 7;
        command[0].target_position = static_cast<std::int32_t>(tick);
        trace.capture(feedback, command, tick * 500, true, 45, 45, 1, 0);
      }
      done.store(true, std::memory_order_release);
    });
  std::uint32_t consumed = 0;
  std::uint64_t last_tick = 0;
  bool complete = false;
  do {
    // 先读取完成标志再排空；完成标志为 true 时不会再有生产者追加帧。
    complete = done.load(std::memory_order_acquire);
    x1_coffee_hardware_core::detail::EnableTraceFrame frame;
    while (trace.pop(&frame)) {
      if (consumed > 0) {
        EXPECT_GT(frame.tick, last_tick);
      }
      EXPECT_EQ(frame.next[0].target_position, static_cast<std::int32_t>(frame.tick));
      if (frame.tick > 0) {
        EXPECT_EQ(frame.sent[0].target_position, static_cast<std::int32_t>(frame.tick - 1));
      }
      last_tick = frame.tick;
      ++consumed;
    }
    std::this_thread::yield();
  } while (!complete);
  producer.join();
  EXPECT_EQ(consumed + trace.dropped(), count);
}

}  // namespace

// 模拟 120 ms 发帧中断和看门狗状态变化，不连接硬件。
TEST(EnableTrace, retains_timing_peaks_and_pre_ack_al_changes)
{
  using namespace x1_coffee_hardware_core::detail;
  EnableTrace trace;
  EnableTraceFrame frame;
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  std::array<SlaveCommand, kCoffeeAxisCount> command{};
  EnableCycleDiagnostics diagnostics;
  diagnostics.send_started_us = 1000;
  trace.capture(feedback, command, 1010, false, 45, 45, 1, 0, diagnostics);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_EQ(frame.diagnostics.al_age_us, -1);
  diagnostics.send_started_us = 121000;
  diagnostics.wake_late_us = 119000;
  diagnostics.al_age_us = 0;
  diagnostics.al_state[5] = 0x14;
  diagnostics.al_code[5] = 0x1b;
  trace.capture(feedback, command, 121010, false, 42, 45, 1, 0, diagnostics);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_NE(frame.al_changed & (1u << 5), 0u);
  EXPECT_EQ(frame.max_send_gap_us, 120000u);
  EXPECT_EQ(frame.max_wake_late_us, 119000u);
  EXPECT_EQ(frame.low_wkc_frames, 1u);
  diagnostics.send_started_us = 122000;
  diagnostics.wake_late_us = 5;
  diagnostics.al_state[5] = 0x04;
  diagnostics.al_code[5] = 0;
  trace.capture(feedback, command, 122010, false, 42, 45, 1, 0, diagnostics);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_NE(frame.al_changed & (1u << 5), 0u);
  EXPECT_EQ(frame.diagnostics.al_state[5], 0x04);
  EXPECT_EQ(frame.max_wake_late_us, 119000u);
  diagnostics.send_started_us = 123000;
  trace.capture(feedback, command, 123010, true, 45, 45, 1, 0, diagnostics);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_EQ(frame.max_send_gap_us, 120000u);
  EXPECT_EQ(frame.max_wake_late_us, 119000u);
}

// 逐台注入 AL 故障和恢复，验证 15 台均会产生独立变化位，缓存变旧不产生假事件。
TEST(EnableTrace, reports_each_slave_al_transition_without_repeating_cached_samples)
{
  using namespace x1_coffee_hardware_core::detail;
  EnableTrace trace;
  EnableTraceFrame frame;
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  std::array<SlaveCommand, kCoffeeAxisCount> command{};
  EnableCycleDiagnostics diagnostics;
  diagnostics.al_age_us = 0;
  diagnostics.al_state.fill(8);
  std::uint64_t now = 1000;
  trace.capture(feedback, command, now, false, 45, 45, 1, 0, diagnostics);
  ASSERT_TRUE(trace.pop(&frame));
  EXPECT_EQ(frame.al_changed, 0x7fffu);
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    diagnostics.al_state[i] = 0x14;
    diagnostics.al_code[i] = 0x1b;
    trace.capture(feedback, command, ++now, false, 45, 45, 1, 0, diagnostics);
    ASSERT_TRUE(trace.pop(&frame));
    EXPECT_EQ(frame.al_changed, 1u << i);
    EXPECT_EQ(frame.diagnostics.al_code[i], 0x1b);
    diagnostics.al_age_us = 1000;
    trace.capture(feedback, command, ++now, false, 45, 45, 1, 0, diagnostics);
    EXPECT_FALSE(trace.pop(&frame));
    diagnostics.al_age_us = 0;
    diagnostics.al_state[i] = 8;
    diagnostics.al_code[i] = 0;
    trace.capture(feedback, command, ++now, false, 45, 45, 1, 0, diagnostics);
    ASSERT_TRUE(trace.pop(&frame));
    EXPECT_EQ(frame.al_changed, 1u << i);
  }
}
