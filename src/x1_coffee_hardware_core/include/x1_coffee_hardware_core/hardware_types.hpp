#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

namespace x1_coffee_hardware_core
{

enum class CoreError : std::uint32_t
{
  kOk = 0,
  kInvalidConfiguration = 1,
  kTransportUnavailable = 2,
  kEthercatNotReady = 3,
  kReadFailed = 4,
  kWriteFailed = 5,
  kCommandNotFinite = 6,
  kPositionLimitViolation = 7,
  kVelocityLimitViolation = 8,
  kStateTimeout = 9,
  kAxisAlarm = 10,
  kEmergencyStop = 11,
};

struct AxisState
{
  std::int64_t encoder_pulse_raw{0};
  double position{0.0};
  double velocity{0.0};
  std::uint32_t status_bits{0};
  std::int16_t current_permille{0};
  std::int16_t torque_permille{0};
  bool enabled{false};
  bool moving{false};
  bool alarm{false};
  bool follow_error{false};
  bool emergency_input_active{false};
  bool encoder_valid{false};
  bool position_valid{false};
  bool velocity_valid{false};
  bool current_valid{false};
  bool torque_valid{false};
};

struct HardwareSnapshot
{
  std::array<AxisState, kCoffeeAxisCount> axes{};
  std::chrono::steady_clock::time_point sampled_at{};
};

struct PositionCommandFrame
{
  std::array<double, kCoffeeAxisCount> positions{};
  std::chrono::steady_clock::time_point generated_at{};
};

struct AxisLimit
{
  double minimum{-std::numeric_limits<double>::infinity()};
  double maximum{std::numeric_limits<double>::infinity()};
  double maximum_velocity{std::numeric_limits<double>::infinity()};
};

using CoffeeAxisLimits = std::array<AxisLimit, kCoffeeAxisCount>;

struct Result
{
  CoreError error{CoreError::kOk};
  std::string detail;

  explicit operator bool() const { return error == CoreError::kOk; }

  static Result ok() { return {}; }
  static Result fail(CoreError code, std::string message)
  {
    return Result{code, std::move(message)};
  }
};

}  // namespace x1_coffee_hardware_core
