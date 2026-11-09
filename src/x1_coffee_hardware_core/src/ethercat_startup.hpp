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

// SOEM issue 725 的 ec_sync：把收帧拉到周期起点。正数表示偏晚，靠近周期中点的 SYNC0。
inline std::int64_t syncToCycleStartErrorNs(std::int64_t dc_time, std::int64_t cycle_ns)
{
  const std::int64_t phase = dcPhaseNs(dc_time, cycle_ns);
  if (phase < 0) {
    return 0;
  }
  if (phase > cycle_ns / 2) {
    return phase - cycle_ns;
  }
  return phase;
}

// SYNC0 在周期中点。收帧落在前 400 µs，发帧才赶在脉冲之前。
inline bool frameBeforeHalfCycleSync(std::int64_t dc_time, std::int64_t cycle_ns)
{
  const std::int64_t phase = dcPhaseNs(dc_time, cycle_ns);
  return phase >= 0 && phase <= 400000 && cycle_ns > 400000;
}

}  // namespace x1_coffee_hardware_core::detail
