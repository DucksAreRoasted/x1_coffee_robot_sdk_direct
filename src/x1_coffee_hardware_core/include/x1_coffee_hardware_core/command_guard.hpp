#pragma once

#include <array>

#include "x1_coffee_hardware_core/hardware_types.hpp"

namespace x1_coffee_hardware_core
{

class CommandGuard
{
public:
  explicit CommandGuard(CoffeeAxisLimits limits);

  Result validate(
    const PositionCommandFrame & command,
    const std::array<double, kCoffeeAxisCount> & previous_positions,
    double period_seconds) const;

  // Pull each joint back to the farthest point this cycle is allowed to reach.
  // Dropping the whole frame leaves the controller commanding a point the bus
  // will never accept, so later cycles stay blocked.
  void clampVelocity(
    PositionCommandFrame & command,
    const std::array<double, kCoffeeAxisCount> & previous_positions,
    double period_seconds) const;

  const CoffeeAxisLimits & limits() const { return limits_; }

private:
  CoffeeAxisLimits limits_;
};

}  // namespace x1_coffee_hardware_core
