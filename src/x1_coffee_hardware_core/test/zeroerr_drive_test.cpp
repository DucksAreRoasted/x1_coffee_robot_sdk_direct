#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/zeroerr_drive.hpp"

namespace
{

using x1_coffee_hardware_core::ZeroErrInput;
using x1_coffee_hardware_core::ZeroErrOutput;
using x1_coffee_hardware_core::zeroErrControlWord;

TEST(ZeroErrDrive, control_word_follows_the_demo_status_cases)
{
  EXPECT_EQ(zeroErrControlWord(0x1208), 0x0080);
  EXPECT_EQ(zeroErrControlWord(0x1250), 0x0006);
  EXPECT_EQ(zeroErrControlWord(0x1231), 0x0007);
  EXPECT_EQ(zeroErrControlWord(0x1233), 0x000F);
  EXPECT_EQ(zeroErrControlWord(0x1237), 0x001F);
  EXPECT_EQ(zeroErrControlWord(0x1637), 0x001F);
  EXPECT_EQ(zeroErrControlWord(0x0000), 0x0006);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrTorqueControlWord(0x1237), 0x000F);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrTorqueControlWord(0x1637), 0x000F);
  EXPECT_EQ(x1_coffee_hardware_core::kZeroErrCyclicPositionMode, 0x08);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrCspControlWord(0x1237), 0x000F);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrCspControlWord(0x1637), 0x000F);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrCspControlWord(0x1233), 0x000F);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrCspControlWord(0x1250), 0x0006);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrCspControlWord(0x1231), 0x0007);
  EXPECT_EQ(x1_coffee_hardware_core::zeroErrCspControlWord(0x0231), 0x0007);
}

TEST(ZeroErrDrive, rated_torque_converts_newton_meters_without_a_stand_in)
{
  EXPECT_NEAR(x1_coffee_hardware_core::torquePermilleFromNm(16.633, 50.0), 332.66, 1e-9);
  EXPECT_NEAR(x1_coffee_hardware_core::torqueNmFromPermille(333, 50.0), 16.65, 1e-12);
  EXPECT_NEAR(
    x1_coffee_hardware_core::torqueNmFromPermille(
      static_cast<std::int16_t>(std::lround(
        x1_coffee_hardware_core::torquePermilleFromNm(-16.521, 20.0))),
      20.0),
    -16.52, 0.01);
  EXPECT_EQ(x1_coffee_hardware_core::torquePermilleFromNm(16.0, 0.0), 0.0);
}

TEST(ZeroErrDrive, pdo_image_supports_position_torque_and_mode_feedback)
{
  EXPECT_EQ(sizeof(ZeroErrOutput), 10u);
  EXPECT_EQ(sizeof(ZeroErrInput), 16u);
  ZeroErrInput input{};
  input.actual_torque = 123;
  input.actual_current = -321;
  const auto * feedback = reinterpret_cast<const std::uint8_t *>(&input);
  EXPECT_EQ(feedback[10], 123);
  EXPECT_EQ(feedback[12], 0xBF);
  EXPECT_EQ(feedback[13], 0xFE);
  ZeroErrOutput output{};
  output.control_word = 0x000F;
  output.target_position = 5566;
  output.mode_of_operation = 0x08;
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&output);
  EXPECT_EQ(bytes[0], 0x0F);
  EXPECT_EQ(bytes[1], 0x00);
  EXPECT_EQ(bytes[2], 0xBE);
  EXPECT_EQ(bytes[3], 0x15);
  EXPECT_EQ(bytes[8], 0x08);
  output.target_torque = -321;
  EXPECT_EQ(bytes[6], 0xBF);
  EXPECT_EQ(bytes[7], 0xFE);
  input.mode_of_operation = 0x0A;
  EXPECT_EQ(feedback[14], 0x0A);
}

}  // namespace
