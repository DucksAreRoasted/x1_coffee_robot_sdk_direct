#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"
#include "../src/enable_trace.hpp"

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
      feedback[index].valid = true;
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
  EXPECT_EQ(raw->last[0].control_word, 0x0006);
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
  EXPECT_EQ(raw->last[1].control_word, 0x000F);
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
  EXPECT_EQ(raw->last[0].control_word, 0x0007);
  EXPECT_EQ(raw->last[6].target_position, 7000);
  EXPECT_EQ(raw->last[6].control_word, 0x0007);

  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    feedback[index].valid = true;
    feedback[index].status_word = 0x1231;
    feedback[index].actual_position = 5000 + static_cast<std::int32_t>(index);
  }
  raw->cycle(feedback, &raw->last);
  EXPECT_EQ(raw->last[6].target_position, 5006);
  EXPECT_EQ(raw->last[12].target_position, 5012);
  EXPECT_EQ(raw->last[0].control_word, 0x0007);
  EXPECT_EQ(raw->last[6].control_word, 0x0007);
  EXPECT_EQ(raw->last[14].control_word, 0x0007);
}

TEST(SoemEthercatTransport, each_axis_enables_on_its_own_and_a_stuck_axis_retries)
{
  auto link = std::make_unique<FakeLink>();
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());

  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  for (auto & item : feedback) {
    item.valid = true;
    item.status_word = 0x1637;
    item.actual_position = 1000;
  }
  feedback[2].status_word = 0x12b1;
  std::vector<std::uint16_t> words;
  for (int cycle = 0; cycle < 8000; ++cycle) {
    feedback[2].actual_position = 1000 + cycle % 3;
    raw->cycle(feedback, &raw->last);
    EXPECT_EQ(raw->last[0].control_word, 0x000F);
    EXPECT_EQ(raw->last[2].target_position, feedback[2].actual_position);
    words.push_back(raw->last[2].control_word);
  }
  // 停住时：接通字、使能字、关电压、关机满 300 ms，然后再次接通。
  std::vector<int> gap_lengths;
  int gap_cycles = 0;
  int enable_words = 0;
  for (const std::uint16_t word : words) {
    EXPECT_TRUE(word == 0x0007 || word == 0x000F || word == 0x0000 || word == 0x0006);
    if (word == 0x000F) {
      ++enable_words;
    }
    if (word == 0x0006) {
      ++gap_cycles;
    } else if (gap_cycles > 0) {
      gap_lengths.push_back(gap_cycles);
      gap_cycles = 0;
    }
  }
  if (gap_cycles > 0) {
    gap_lengths.push_back(gap_cycles);
  }
  EXPECT_GT(enable_words, 0);
  ASSERT_GE(gap_lengths.size(), 2u);
  EXPECT_EQ(gap_lengths[0], 600);
  EXPECT_EQ(gap_lengths[1], 600);
}

TEST(SoemEthercatTransport, csp_target_slews_without_a_single_cycle_jump)
{
  auto link = std::make_unique<FakeLink>();
  link->status = 0x1237;
  FakeLink * raw = link.get();
  SoemEthercatTransport transport(std::move(link));
  ASSERT_TRUE(transport.configure(parameters(true)));
  ASSERT_TRUE(transport.activate());
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
  std::vector<std::array<double, kCoffeeAxisCount>> waypoints(2);
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const double held = static_cast<double>(index + 1);
    waypoints[0][index] = held;
    waypoints[1][index] = held;
  }
  waypoints[0][0] = 1.5;
  waypoints[1][0] = 2.0;
  ASSERT_TRUE(transport.requestMovePvtSegments(waypoints, {0.5, 0.5}));
  for (int cycle = 0; cycle < 1000; ++cycle) {
    raw->replay();
  }
  EXPECT_NEAR(raw->last[0].target_position, 1500, 5);
  EXPECT_NEAR(raw->last[1].target_position, 2000, 1);
  for (int cycle = 0; cycle < 1000; ++cycle) {
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
  EXPECT_GE(raw->last[8].target_torque, -500);
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
