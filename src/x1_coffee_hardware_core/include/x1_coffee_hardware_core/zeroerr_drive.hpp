#pragma once

#include <cmath>
#include <cstdint>

namespace x1_coffee_hardware_core
{

// 在官方CSP布局基础上扩展位置/扭矩共用PDO：Rx 10字节、Tx 16字节。
// 通信手册4.4.2：模式字节后的8位填充必须处于最后一个映射项。
struct ZeroErrOutput
{
  std::uint16_t control_word;
  std::int32_t target_position;
  std::int16_t target_torque;
  std::uint8_t mode_of_operation;
  std::uint8_t reserved;
} __attribute__((packed));

struct ZeroErrInput
{
  std::uint16_t status_word;
  std::int32_t actual_position;
  std::int32_t actual_velocity;
  // 0x6077，额定电流的千分比。
  std::int16_t actual_torque;
  // 0x6078，CSP 采样也要提供独立的电流反馈。
  std::int16_t actual_current;
  std::uint8_t mode_of_operation;
  std::uint8_t reserved;
} __attribute__((packed));

// 接收 PDO 0x1600，顺序与 eRob_CSP.cpp 相同。
constexpr std::uint32_t kZeroErrRxControlWord = 0x60400010;
constexpr std::uint32_t kZeroErrRxTargetPosition = 0x607A0020;
constexpr std::uint32_t kZeroErrRxModeOfOperation = 0x60600008;
constexpr std::uint32_t kZeroErrRxPadding = 0x00000008;
// 发送 PDO 0x1A00。
constexpr std::uint32_t kZeroErrTxStatusWord = 0x60410010;
constexpr std::uint32_t kZeroErrTxActualPosition = 0x60640020;
constexpr std::uint32_t kZeroErrTxActualVelocity = 0x606C0020;
constexpr std::uint32_t kZeroErrTxModeDisplay = 0x60610008;

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
