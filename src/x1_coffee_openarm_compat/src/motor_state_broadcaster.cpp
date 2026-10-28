#include "x1_coffee_openarm_compat/motor_state_broadcaster.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

namespace x1_coffee_openarm_compat
{
namespace
{
using controller_interface::CallbackReturn;
constexpr std::array<const char *, 14> kDiagnosticInterfaces = {
  "encoder_pulse_raw", "status_bits", "current_permille", "torque_permille",
  "enabled", "moving", "alarm", "follow_error", "emergency_input_active",
  "encoder_valid", "position_valid", "velocity_valid", "current_valid", "torque_valid"};
constexpr std::size_t kInterfacesPerJoint = 2 + kDiagnosticInterfaces.size();
}  // namespace

CallbackReturn MotorStateBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("hardware_profile", "coffee_15_no_slide");
    auto_declare<std::string>("topic_name", "/x1/motor_states");
    auto_declare<double>("publish_rate", 50.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorStateBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  topic_name_ = node->get_parameter("topic_name").as_string();
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  const auto profile = x1_coffee_hardware_core::parseHardwareProfile(
    node->get_parameter("hardware_profile").as_string());
  if (!profile || topic_name_.empty() || publish_rate_ <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid motor broadcaster profile/topic/publish_rate");
    return CallbackReturn::ERROR;
  }
  const auto & layout = x1_coffee_hardware_core::hardwareLayout(*profile);
  const auto & descriptors = x1_coffee_hardware_core::coffeeAxisDescriptors();
  std::vector<std::string> expected_joints;
  expected_joints.reserve(layout.physical_axis_count);
  for (std::size_t physical_index = 0; physical_index < layout.physical_axis_count;
    ++physical_index)
  {
    expected_joints.emplace_back(
      descriptors[layout.physical_to_model_index[physical_index]].joint_name);
  }
  if (joints_ != expected_joints) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Motor broadcaster joints must match the physical-axis order for profile '%s'",
      std::string(layout.name).c_str());
    return CallbackReturn::ERROR;
  }
  message_.states.resize(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto canonical = *x1_coffee_hardware_core::canonicalIndexForJoint(joints_[i]);
    message_.states[i].joint_name = joints_[i];
    message_.states[i].hardware_axis = static_cast<std::uint16_t>(i + 1);
    message_.states[i].position_unit = std::string(descriptors[canonical].position_unit);
  }
  publisher_ = node->create_publisher<x1_coffee_interfaces::msg::MotorStateArray>(
    topic_name_, rclcpp::SensorDataQoS());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotorStateBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration MotorStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    for (const auto * interface : kDiagnosticInterfaces) {
      configuration.names.push_back(joint + "/" + interface);
    }
  }
  return configuration;
}

CallbackReturn MotorStateBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != joints_.size() * kInterfacesPerJoint) {
    RCLCPP_ERROR(get_node()->get_logger(), "Claimed motor state interface count is incorrect");
    return CallbackReturn::ERROR;
  }
  last_publish_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (last_publish_time_.nanoseconds() != 0 &&
    (time - last_publish_time_).seconds() < 1.0 / publish_rate_)
  {
    return controller_interface::return_type::OK;
  }
  message_.header.stamp = time;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const std::size_t base = i * kInterfacesPerJoint;
    auto & state = message_.states[i];
    for (std::size_t field = 0; field < kInterfacesPerJoint; ++field) {
      if (!std::isfinite(state_interfaces_[base + field].get_value())) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Non-finite motor state for %s", joints_[i].c_str());
        return controller_interface::return_type::ERROR;
      }
    }
    state.position = state_interfaces_[base].get_value();
    state.velocity = state_interfaces_[base + 1].get_value();
    state.encoder_pulse_raw = static_cast<std::int64_t>(std::llround(state_interfaces_[base + 2].get_value()));
    state.status_bits = static_cast<std::uint32_t>(std::llround(state_interfaces_[base + 3].get_value()));
    state.current_permille = static_cast<std::int16_t>(std::lround(state_interfaces_[base + 4].get_value()));
    state.torque_permille = static_cast<std::int16_t>(std::lround(state_interfaces_[base + 5].get_value()));
    state.enabled = state_interfaces_[base + 6].get_value() > 0.5;
    state.moving = state_interfaces_[base + 7].get_value() > 0.5;
    state.alarm = state_interfaces_[base + 8].get_value() > 0.5;
    state.follow_error = state_interfaces_[base + 9].get_value() > 0.5;
    state.emergency_input_active = state_interfaces_[base + 10].get_value() > 0.5;
    state.encoder_valid = state_interfaces_[base + 11].get_value() > 0.5;
    state.position_valid = state_interfaces_[base + 12].get_value() > 0.5;
    state.velocity_valid = state_interfaces_[base + 13].get_value() > 0.5;
    state.current_valid = state_interfaces_[base + 14].get_value() > 0.5;
    state.torque_valid = state_interfaces_[base + 15].get_value() > 0.5;
  }
  publisher_->publish(message_);
  last_publish_time_ = time;
  return controller_interface::return_type::OK;
}

}  // namespace x1_coffee_openarm_compat

PLUGINLIB_EXPORT_CLASS(
  x1_coffee_openarm_compat::MotorStateBroadcaster,
  controller_interface::ControllerInterface)
