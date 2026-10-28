#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "x1_coffee_device_sdk/device_result.hpp"

namespace x1_coffee_device_sdk
{

class WaistClient
{
public:
  explicit WaistClient(
    rclcpp::Node::SharedPtr node,
    std::string command_topic = "/waist_forward_position_controller/commands");
  DeviceResult commandRadians(double position_radians);

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

}  // namespace x1_coffee_device_sdk
