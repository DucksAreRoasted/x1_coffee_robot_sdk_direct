#include "x1_coffee_device_sdk/motor_state_reader.hpp"

#include <stdexcept>
#include <utility>

namespace x1_coffee_device_sdk
{

MotorStateReader::MotorStateReader(rclcpp::Node::SharedPtr node, std::string topic)
{
  if (!node || topic.empty()) {
    throw std::invalid_argument("MotorStateReader requires a node and non-empty topic");
  }
  subscription_ = node->create_subscription<Message>(
    std::move(topic), rclcpp::SensorDataQoS(),
    std::bind(&MotorStateReader::receive, this, std::placeholders::_1));
}

std::optional<MotorStateReader::Message> MotorStateReader::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

void MotorStateReader::setCallback(Callback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = std::move(callback);
}

void MotorStateReader::receive(Message::ConstSharedPtr message)
{
  Callback callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = *message;
    callback = callback_;
  }
  if (callback) {
    callback(*message);
  }
}

}  // namespace x1_coffee_device_sdk
