#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "x1_coffee_interfaces/msg/motor_state_array.hpp"

namespace x1_coffee_device_sdk
{

class MotorStateReader
{
public:
  using Message = x1_coffee_interfaces::msg::MotorStateArray;
  using Callback = std::function<void(const Message &)>;

  explicit MotorStateReader(
    rclcpp::Node::SharedPtr node,
    std::string topic = "/x1/motor_states");

  std::optional<Message> latest() const;
  void setCallback(Callback callback);

private:
  void receive(Message::ConstSharedPtr message);

  mutable std::mutex mutex_;
  std::optional<Message> latest_;
  Callback callback_;
  rclcpp::Subscription<Message>::SharedPtr subscription_;
};

}  // namespace x1_coffee_device_sdk
