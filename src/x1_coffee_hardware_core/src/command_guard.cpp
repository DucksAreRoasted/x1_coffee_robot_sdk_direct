#include "x1_coffee_hardware_core/command_guard.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>
#include <sstream>

namespace x1_coffee_hardware_core
{

CommandGuard::CommandGuard(CoffeeAxisLimits limits)
: limits_(std::move(limits))
{
}

Result CommandGuard::validate(
  const PositionCommandFrame & command,
  const std::array<double, kCoffeeAxisCount> & previous_positions,
  double period_seconds) const
{
  const auto & descriptors = coffeeAxisDescriptors();
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const double target = command.positions[index];
    if (!std::isfinite(target)) {
      return Result::fail(
        CoreError::kCommandNotFinite,
        "joint=" + std::string(descriptors[index].joint_name) + " target_not_finite");
    }

    const auto & limit = limits_[index];
    if (target < limit.minimum || target > limit.maximum) {
      std::ostringstream stream;
      stream << "joint=" << descriptors[index].joint_name << " target=" << target <<
        " limits=[" << limit.minimum << ',' << limit.maximum << ']';
      return Result::fail(CoreError::kPositionLimitViolation, stream.str());
    }

    if (period_seconds > 0.0 && std::isfinite(limit.maximum_velocity)) {
      const double velocity = std::abs(target - previous_positions[index]) / period_seconds;
      // Adding a clamped step to a nonzero position can round outward by an
      // ulp. Allow only that coordinate-rounding error, not a velocity margin.
      const double max_step = limit.maximum_velocity * period_seconds;
      const double roundoff = 8.0 * std::numeric_limits<double>::epsilon() *
        std::max({std::abs(target), std::abs(previous_positions[index]), max_step});
      if (velocity > limit.maximum_velocity &&
        std::abs(target - previous_positions[index]) - max_step > roundoff)
      {
        std::ostringstream stream;
        stream << "joint=" << descriptors[index].joint_name << " requested_velocity=" <<
          velocity << " maximum_velocity=" << limit.maximum_velocity;
        return Result::fail(CoreError::kVelocityLimitViolation, stream.str());
      }
    }
  }
  return Result::ok();
}

void CommandGuard::clampVelocity(
  PositionCommandFrame & command,
  const std::array<double, kCoffeeAxisCount> & previous_positions,
  double period_seconds) const
{
  if (!(period_seconds > 0.0)) {
    return;
  }
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const double maximum_velocity = limits_[index].maximum_velocity;
    if (!std::isfinite(maximum_velocity)) {
      continue;
    }
    const double max_step = maximum_velocity * period_seconds;
    const double delta = command.positions[index] - previous_positions[index];
    if (std::abs(delta) > max_step) {
      command.positions[index] = previous_positions[index] + std::copysign(max_step, delta);
    }
  }
}

}  // namespace x1_coffee_hardware_core
