#pragma once

#include <string>
#include <utility>

namespace x1_coffee_device_sdk
{

struct DeviceResult
{
  bool success{true};
  std::string detail;

  static DeviceResult ok() { return {}; }
  static DeviceResult fail(std::string message) { return {false, std::move(message)}; }
  explicit operator bool() const { return success; }
};

}  // namespace x1_coffee_device_sdk
