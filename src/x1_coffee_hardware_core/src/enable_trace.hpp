#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"

namespace x1_coffee_hardware_core::detail
{

struct EnableTraceFrame
{
  std::uint64_t tick{0}, elapsed_us{0}, dropped{0}, low_wkc_frames{0}, nonpositive_send_frames{0};
  std::uint64_t max_cycle_gap_us{0};
  std::int64_t dc_phase_ns{0};
  int wkc{0}, expected_wkc{0}, minimum_wkc{0}, send_rc{0};
  bool operational{false}, summary{false};
  std::uint32_t changed_axes{0};
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  // sent 是上一拍提交给发送函数的过程映像，不代表驱动器已经确认收到。
  std::array<SlaveCommand, kCoffeeAxisCount> sent{}, next{};
  std::array<std::uint64_t, kCoffeeAxisCount> state_age_us{};
  std::array<std::uint64_t, kCoffeeAxisCount> raw_status_changes{};
};

// 单生产者/单消费者队列。周期线程仅复制固定大小数据，不分配内存、不等锁、不输出日志。
// 队列满时丢诊断帧并计数，绝不为了保住日志阻塞总线。
class EnableTrace
{
public:
  static constexpr std::size_t kCapacity = 32;
  static constexpr std::uint64_t kSummaryIntervalUs = 2000000;
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

  void capture(
    const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
    const std::array<SlaveCommand, kCoffeeAxisCount> & command,
    std::uint64_t now_us, bool operational, int wkc, int expected_wkc,
    int send_rc, std::int64_t dc_phase_ns)
  {
    const bool phase_changed = !initialized_ || operational != operational_;
    if (!initialized_) {
      started_us_ = now_us;
    }
    if (phase_changed) {
      minimum_wkc_ = wkc;
      low_wkc_frames_ = 0;
      nonpositive_send_frames_ = 0;
      max_cycle_gap_us_ = 0;
    } else if (now_us - previous_us_ > max_cycle_gap_us_) {
      max_cycle_gap_us_ = now_us - previous_us_;
    }
    if (wkc < minimum_wkc_) {
      minimum_wkc_ = wkc;
    }
    const bool bad_wkc = wkc != expected_wkc;
    if (bad_wkc) {
      ++low_wkc_frames_;
    }
    if (send_rc <= 0) {
      ++nonpositive_send_frames_;
    }
    std::uint32_t changed = 0;
    for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
      const bool state_changed = !initialized_ ||
        (feedback[i].status_word & 0x006f) != (previous_feedback_[i].status_word & 0x006f) ||
        feedback[i].valid != previous_feedback_[i].valid;
      if (state_changed) {
        state_since_us_[i] = now_us;
      }
      if (initialized_ && feedback[i].status_word != previous_feedback_[i].status_word) {
        ++raw_status_changes_[i];
      }
      // 电压、警告、远程位也立即记录；目标到达/忽略等高位抖动只累计次数，避免刷屏。
      const bool other_changed =
        ((feedback[i].status_word ^ previous_feedback_[i].status_word) & 0x0290) != 0;
      if (state_changed || other_changed ||
        command[i].control_word != previous_command_[i].control_word)
      {
        changed |= std::uint32_t{1} << i;
      }
    }
    const bool summary = phase_changed || now_us - last_summary_us_ >= kSummaryIntervalUs;
    if (summary || changed != 0 || bad_wkc != previous_bad_wkc_ || send_rc <= 0) {
      EnableTraceFrame frame;
      frame.tick = tick_;
      frame.elapsed_us = now_us - started_us_;
      frame.operational = operational;
      frame.summary = summary;
      frame.changed_axes = changed;
      frame.wkc = wkc;
      frame.expected_wkc = expected_wkc;
      frame.minimum_wkc = minimum_wkc_;
      frame.low_wkc_frames = low_wkc_frames_;
      frame.send_rc = send_rc;
      frame.nonpositive_send_frames = nonpositive_send_frames_;
      frame.max_cycle_gap_us = max_cycle_gap_us_;
      frame.dropped = dropped_.load(std::memory_order_relaxed);
      frame.dc_phase_ns = dc_phase_ns;
      frame.feedback = feedback;
      frame.sent = previous_command_;
      frame.next = command;
      frame.raw_status_changes = raw_status_changes_;
      for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
        frame.state_age_us[i] = now_us - state_since_us_[i];
      }
      push(frame);
      if (summary) {
        last_summary_us_ = now_us;
      }
    }
    // 即使日志丢帧，上一拍指令也必须每拍更新，避免把旧日志误称为上一拍输出。
    previous_feedback_ = feedback;
    previous_command_ = command;
    previous_bad_wkc_ = bad_wkc;
    previous_us_ = now_us;
    operational_ = operational;
    initialized_ = true;
    ++tick_;
  }

  bool pop(EnableTraceFrame * frame)
  {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {
      return false;
    }
    *frame = frames_[read];
    read_.store((read + 1) % kCapacity, std::memory_order_release);
    return true;
  }

  std::uint32_t dropped() const {return dropped_.load(std::memory_order_relaxed);}

private:
  void push(const EnableTraceFrame & frame)
  {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % kCapacity;
    if (next == read_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    frames_[write] = frame;
    write_.store(next, std::memory_order_release);
  }

  std::array<EnableTraceFrame, kCapacity> frames_{};
  std::atomic<std::uint32_t> read_{0}, write_{0}, dropped_{0};
  bool initialized_{false}, operational_{false}, previous_bad_wkc_{false};
  std::uint64_t tick_{0}, started_us_{0}, last_summary_us_{0};
  std::uint64_t low_wkc_frames_{0}, nonpositive_send_frames_{0};
  std::uint64_t previous_us_{0}, max_cycle_gap_us_{0};
  int minimum_wkc_{0};
  std::array<std::uint64_t, kCoffeeAxisCount> state_since_us_{};
  std::array<std::uint64_t, kCoffeeAxisCount> raw_status_changes_{};
  std::array<SlaveFeedback, kCoffeeAxisCount> previous_feedback_{};
  std::array<SlaveCommand, kCoffeeAxisCount> previous_command_{};
};

}  // namespace x1_coffee_hardware_core::detail
