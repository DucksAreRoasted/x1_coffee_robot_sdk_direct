#include "x1_coffee_device_sdk/gripper_client.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace x1_coffee_device_sdk
{

GripperClient::GripperClient(
  rclcpp::Node::SharedPtr node, std::string command_topic,
  double open_position, double closed_position)
: open_position_(open_position), closed_position_(closed_position),
  minimum_(std::min(open_position, closed_position)),
  maximum_(std::max(open_position, closed_position))
{
  if (!node || command_topic.empty() || !std::isfinite(open_position) ||
    !std::isfinite(closed_position) || open_position == closed_position)
  {
    throw std::invalid_argument("GripperClient configuration is invalid");
  }
  publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::move(command_topic), rclcpp::SystemDefaultsQoS());
}

DeviceResult GripperClient::open()
{
  return moveTo(open_position_);
}

DeviceResult GripperClient::close()
{
  return moveTo(closed_position_);
}

DeviceResult GripperClient::moveTo(double position)
{
  if (!std::isfinite(position)) {
    return DeviceResult::fail("gripper position must be finite");
  }
  if (position < minimum_ || position > maximum_) {
    return DeviceResult::fail("gripper position is outside configured range");
  }
  std_msgs::msg::Float64MultiArray command;
  command.data = {position};
  publisher_->publish(command);
  return DeviceResult::ok();
}

}  // namespace x1_coffee_device_sdk
