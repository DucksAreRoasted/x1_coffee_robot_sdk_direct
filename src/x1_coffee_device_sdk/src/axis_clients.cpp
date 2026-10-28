#include "x1_coffee_device_sdk/axis_clients.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace x1_coffee_device_sdk
{
namespace
{
DeviceResult publishPosition(
  const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & publisher,
  double value)
{
  if (!std::isfinite(value)) {
    return DeviceResult::fail("position must be finite");
  }
  std_msgs::msg::Float64MultiArray command;
  command.data = {value};
  publisher->publish(command);
  return DeviceResult::ok();
}
}  // namespace

WaistClient::WaistClient(rclcpp::Node::SharedPtr node, std::string command_topic)
{
  if (!node || command_topic.empty()) {
    throw std::invalid_argument("WaistClient requires a node and non-empty topic");
  }
  publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::move(command_topic), rclcpp::SystemDefaultsQoS());
}

DeviceResult WaistClient::commandRadians(double position_radians)
{
  return publishPosition(publisher_, position_radians);
}

}  // namespace x1_coffee_device_sdk
