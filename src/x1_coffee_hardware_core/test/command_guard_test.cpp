#include <gtest/gtest.h>

#include <limits>

#include "x1_coffee_hardware_core/command_guard.hpp"

namespace core = x1_coffee_hardware_core;

TEST(CommandGuard, RejectsNan)
{
  core::CoffeeAxisLimits limits{};
  core::CommandGuard guard(limits);
  core::PositionCommandFrame frame{};
  frame.positions[2] = std::numeric_limits<double>::quiet_NaN();
  std::array<double, core::kCoffeeAxisCount> previous{};
  const auto result = guard.validate(frame, previous, 0.01);
  EXPECT_EQ(result.error, core::CoreError::kCommandNotFinite);
}

TEST(CommandGuard, RejectsPositionLimit)
{
  core::CoffeeAxisLimits limits{};
  limits[0].minimum = 0.0;
  limits[0].maximum = 1.0;
  core::CommandGuard guard(limits);
  core::PositionCommandFrame frame{};
  frame.positions[0] = 1.1;
  std::array<double, core::kCoffeeAxisCount> previous{};
  const auto result = guard.validate(frame, previous, 0.01);
  EXPECT_EQ(result.error, core::CoreError::kPositionLimitViolation);
}

TEST(CommandGuard, RejectsExcessiveVelocity)
{
  core::CoffeeAxisLimits limits{};
  limits[2].maximum_velocity = 1.0;
  core::CommandGuard guard(limits);
  core::PositionCommandFrame frame{};
  frame.positions[2] = 0.1;
  std::array<double, core::kCoffeeAxisCount> previous{};
  const auto result = guard.validate(frame, previous, 0.01);
  EXPECT_EQ(result.error, core::CoreError::kVelocityLimitViolation);
}

TEST(CommandGuard, ClampsVelocityToTheCycleLimit)
{
  core::CoffeeAxisLimits limits{};
  limits[2].maximum_velocity = 1.0;
  core::CommandGuard guard(limits);
  core::PositionCommandFrame frame{};
  frame.positions[2] = 0.1;
  std::array<double, core::kCoffeeAxisCount> previous{};
  guard.clampVelocity(frame, previous, 0.01);
  EXPECT_NEAR(frame.positions[2], 0.01, 1e-12);
  const auto result = guard.validate(frame, previous, 0.01);
  EXPECT_TRUE(static_cast<bool>(result));
}

TEST(CommandGuard, ClampedNonzeroPositionsPassValidation)
{
  core::CoffeeAxisLimits limits{};
  limits[2].maximum_velocity = 1.0;
  core::CommandGuard guard(limits);
  for (double start : {-1.607001, -0.989921, 0.970147, 1.640077}) {
    for (double direction : {-1.0, 1.0}) {
      core::PositionCommandFrame frame{};
      std::array<double, core::kCoffeeAxisCount> previous{};
      previous[2] = start;
      frame.positions[2] = start + direction * 0.3;
      guard.clampVelocity(frame, previous, 0.01);
      EXPECT_TRUE(static_cast<bool>(guard.validate(frame, previous, 0.01)));
      // A material excess must still be rejected.
      frame.positions[2] = start + direction * 0.010001;
      EXPECT_EQ(guard.validate(frame, previous, 0.01).error,
        core::CoreError::kVelocityLimitViolation);
    }
  }
}
