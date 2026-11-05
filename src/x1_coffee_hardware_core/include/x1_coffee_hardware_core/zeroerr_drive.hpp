#pragma once

#include <cmath>
#include <cstdint>

namespace x1_coffee_hardware_core
{

// 与 zero-voyager/ZeroErr my_Demo 的过程数据布局一致，按 10 字节压紧。
struct ZeroErrOutput
{
  std::int32_t target_position;
  std::uint32_t digital_output;
  std::uint16_t control_word;
  // 周期同步扭矩目标。位置模式下驱动器忽略它，MIT 时才使用。
  std::int16_t target_torque;
} __attribute__((packed));

struct ZeroErrInput
{
  std::int32_t actual_position;
  std::uint32_t digital_inputs;
  std::uint16_t status_word;
  // 0x6077，额定电流的千分比。位置模式和 MIT 都读它。
  std::int16_t actual_torque;
} __attribute__((packed));

// 接收 PDO 0x1600 的三条映射，顺序与该示例相同。
constexpr std::uint32_t kZeroErrRxTargetPosition = 0x607A0020;
constexpr std::uint32_t kZeroErrRxDigitalOutput = 0x60FE0020;
constexpr std::uint32_t kZeroErrRxControlWord = 0x60400010;
// 发送 PDO 0x1A00。
constexpr std::uint32_t kZeroErrTxActualPosition = 0x60640020;
constexpr std::uint32_t kZeroErrTxDigitalInput = 0x60FD0020;
constexpr std::uint32_t kZeroErrTxStatusWord = 0x60410010;

// 该示例写入的模式和轮廓参数。0x01 在 eRob 表 5-1 是轮廓位置。
constexpr std::uint8_t kZeroErrProfilePositionMode = 0x01;
constexpr std::uint32_t kZeroErrProfileVelocity = 30000;
constexpr std::uint32_t kZeroErrProfileAcceleration = 15566;
constexpr std::uint32_t kZeroErrProfileDeceleration = 15566;

// 按 Ethercat_control.cpp 的状态字分支选择控制字。其余位保持该示例写出的值。
std::uint16_t zeroErrControlWord(std::uint16_t status_word);

// eRob 通信手册表 5-1：周期同步位置。每个通信周期写 0x607A，使能控制字停在 0x000F。
constexpr std::uint8_t kZeroErrCyclicPositionMode = 0x08;

// eRob 手册 5.9：周期同步扭矩。使能后控制字停在 0x000F，不发轮廓位置的 0x001F。
constexpr std::uint8_t kZeroErrCyclicTorqueMode = 0x0A;
constexpr std::uint32_t kZeroErrRxTargetTorque = 0x60710010;
constexpr std::uint32_t kZeroErrTxActualTorque = 0x60770010;
constexpr std::uint32_t kZeroErrTxActualCurrent = 0x60780010;

std::uint16_t zeroErrTorqueControlWord(std::uint16_t status_word);

// CSP 使能步骤，通信手册 PDF页 61。bit4–bit0 依次为 00110、00111、01111。
std::uint16_t zeroErrCspControlWord(std::uint16_t status_word);

// 手册 8.2.96：0x6076 单位是 mN·m，相对扭矩都参照它。
// 手册 8.2.91、8.2.97：0x6071 和 0x6077 是额定电流的千分比。额定电流对应额定扭矩，
// 所以千分比 = 扭矩(N·m) / (0x6076 的 mN·m / 1000) × 1000。不用 URDF 的 effort 代替。
inline double torquePermilleFromNm(double newton_meters, double rated_nm)
{
  if (!(rated_nm > 0.0) || !std::isfinite(newton_meters)) {
    return 0.0;
  }
  return newton_meters / rated_nm * 1000.0;
}

inline double torqueNmFromPermille(std::int16_t permille, double rated_nm)
{
  if (!(rated_nm > 0.0)) {
    return 0.0;
  }
  return static_cast<double>(permille) * rated_nm / 1000.0;
}

}  // namespace x1_coffee_hardware_core
