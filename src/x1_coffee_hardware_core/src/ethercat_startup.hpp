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

}  // namespace x1_coffee_hardware_core::detail
