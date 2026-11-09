#pragma once

#include <array>
#include <cstdint>

namespace x1_coffee_hardware_core::detail
{

struct AlStatus
{
  std::uint16_t state;
  std::uint16_t code;
};

// ESC 0x0130 是状态，0x0132 保留，0x0134 才是 AL 错误码；寄存器按小端传输。
inline AlStatus decodeAlStatus(const std::array<std::uint8_t, 6> & bytes)
{
  return {
    static_cast<std::uint16_t>(bytes[0] | (bytes[1] << 8)),
    static_cast<std::uint16_t>(bytes[4] | (bytes[5] << 8))};
}

// 带 ERROR 位的状态和未知状态都不能作为进入运行的依据。
inline bool alStateReady(std::uint16_t actual, std::uint16_t requested)
{
  return actual == requested || (requested == 0x04 && actual == 0x08);
}

inline bool completeProcessData(int wkc, int expected)
{
  return expected > 0 && wkc == expected;
}

// 分布时钟在一个周期内的位置。还没有时钟时返回 -1。
inline std::int64_t dcPhaseNs(std::int64_t dc_time, std::int64_t cycle_ns)
{
  if (cycle_ns <= 0 || dc_time <= 0) {
    return -1;
  }
  std::int64_t phase = dc_time % cycle_ns;
  if (phase < 0) {
    phase += cycle_ns;
  }
  return phase;
}

// 相对“脉冲前 lead”的误差，单位 ns。正数表示这一拍偏晚，靠近或越过了 SYNC0。
inline std::int64_t syncLeadErrorNs(
  std::int64_t dc_time, std::int64_t cycle_ns, std::int64_t lead_ns)
{
  const std::int64_t phase = dcPhaseNs(dc_time, cycle_ns);
  if (phase < 0 || lead_ns <= 0 || lead_ns >= cycle_ns) {
    return 0;
  }
  std::int64_t delta = phase - (cycle_ns - lead_ns);
  if (delta > cycle_ns / 2) {
    delta -= cycle_ns;
  } else if (delta <= -cycle_ns / 2) {
    delta += cycle_ns;
  }
  return delta;
}

// 收帧落在脉冲前的窗口里，随后发出的输出才会赶在 SYNC0 之前写入从站。
inline bool outputReadyBeforeSync(
  std::int64_t dc_time, std::int64_t cycle_ns, std::int64_t lead_ns)
{
  const std::int64_t phase = dcPhaseNs(dc_time, cycle_ns);
  if (phase < 0 || lead_ns <= 0) {
    return false;
  }
  const std::int64_t earliest = cycle_ns - lead_ns - 150000;
  const std::int64_t latest = cycle_ns - 80000;
  return phase >= earliest && phase <= latest;
}

}  // namespace x1_coffee_hardware_core::detail
