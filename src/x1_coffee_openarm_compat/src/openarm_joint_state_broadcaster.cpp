#include "x1_coffee_openarm_compat/openarm_joint_state_broadcaster.hpp"

#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace x1_coffee_openarm_compat
{
namespace
{
using controller_interface::CallbackReturn;
}

CallbackReturn OpenArmJointStateBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("source_joints", {});
    auto_declare<std::vector<std::string>>("alias_joints", {});
    auto_declare<std::string>("topic_name", "/x1/openarm_compat/joint_states");
    auto_declare<double>("publish_rate", 100.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmJointStateBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  source_joints_ = node->get_parameter("source_joints").as_string_array();
  alias_joints_ = node->get_parameter("alias_joints").as_string_array();
  topic_name_ = node->get_parameter("topic_name").as_string();
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  if (source_joints_.empty() || source_joints_.size() != alias_joints_.size() ||
    topic_name_.empty() || publish_rate_ <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid joint alias broadcaster parameters");
    return CallbackReturn::ERROR;
  }
  message_.name = alias_joints_;
  message_.position.resize(alias_joints_.size());
  message_.velocity.resize(alias_joints_.size());
  publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
    topic_name_, rclcpp::SystemDefaultsQoS());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OpenArmJointStateBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
OpenArmJointStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : source_joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

CallbackReturn OpenArmJointStateBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != source_joints_.size() * 2) {
    RCLCPP_ERROR(get_node()->get_logger(), "Claimed state interface count is incorrect");
    return CallbackReturn::ERROR;
  }
  last_publish_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmJointStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OpenArmJointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (last_publish_time_.nanoseconds() != 0 &&
    (time - last_publish_time_).seconds() < 1.0 / publish_rate_)
  {
    return controller_interface::return_type::OK;
  }
  message_.header.stamp = time;
  for (std::size_t i = 0; i < source_joints_.size(); ++i) {
    message_.position[i] = state_interfaces_[2 * i].get_value();
    message_.velocity[i] = state_interfaces_[2 * i + 1].get_value();
    if (!std::isfinite(message_.position[i]) || !std::isfinite(message_.velocity[i])) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Non-finite state for source joint %s", source_joints_[i].c_str());
      return controller_interface::return_type::ERROR;
    }
  }
  publisher_->publish(message_);
  last_publish_time_ = time;
  return controller_interface::return_type::OK;
}

}  // namespace x1_coffee_openarm_compat

PLUGINLIB_EXPORT_CLASS(
  x1_coffee_openarm_compat::OpenArmJointStateBroadcaster,
  controller_interface::ControllerInterface)
