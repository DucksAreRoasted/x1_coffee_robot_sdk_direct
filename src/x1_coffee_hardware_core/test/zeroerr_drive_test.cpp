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

TEST(ZeroErrDrive, pdo_image_is_ten_bytes_in_demo_order)
{
  EXPECT_EQ(sizeof(ZeroErrOutput), 12u);
  EXPECT_EQ(sizeof(ZeroErrInput), 12u);
  ZeroErrOutput output{};
  output.target_position = 5566;
  output.digital_output = 0;
  output.control_word = 0x000F;
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&output);
  EXPECT_EQ(bytes[0], 0xBE);
  EXPECT_EQ(bytes[1], 0x15);
  EXPECT_EQ(bytes[4], 0x00);
  EXPECT_EQ(bytes[8], 0x0F);
  EXPECT_EQ(bytes[9], 0x00);
}

}  // namespace
