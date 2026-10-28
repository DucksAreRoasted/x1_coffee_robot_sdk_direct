#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "x1_coffee_device_sdk/device_result.hpp"

namespace x1_coffee_device_sdk
{

class GripperClient
{
public:
  GripperClient(
    rclcpp::Node::SharedPtr node, std::string command_topic,
    double open_position, double closed_position);

  DeviceResult open();
  DeviceResult close();
  DeviceResult moveTo(double position);

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  double open_position_;
  double closed_position_;
  double minimum_;
  double maximum_;
};

}  // namespace x1_coffee_device_sdk
