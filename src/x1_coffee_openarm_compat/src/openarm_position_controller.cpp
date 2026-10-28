#include "x1_coffee_openarm_compat/openarm_position_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace x1_coffee_openarm_compat
{
namespace
{
using controller_interface::CallbackReturn;

bool normalizeLimits(std::vector<double> * values, std::size_t size, double default_value)
{
  if (values->empty()) {
    values->assign(size, default_value);
  }
  return values->size() == size;
}
}  // namespace

CallbackReturn OpenArmPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<int>("expected_command_size", 0);
    auto_declare<int>("command_timeout_ms", 100);
    auto_declare<bool>("initial_alignment_check", true);
    auto_declare<double>("max_initial_difference", 0.35);
    auto_declare<double>("alignment_duration_sec", 3.0);
    auto_declare<std::vector<double>>("lower_limits", {});
    auto_declare<std::vector<double>>("upper_limits", {});
    auto_declare<std::vector<double>>("max_velocities", {});
    auto_declare<std::vector<std::int64_t>>("input_indices", {});
    auto_declare<std::vector<double>>("command_scales", {});
    auto_declare<std::vector<double>>("command_offsets", {});
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmPositionController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  const auto configured_size = node->get_parameter("expected_command_size").as_int();
  const auto timeout_ms = node->get_parameter("command_timeout_ms").as_int();
  initial_alignment_check_ = node->get_parameter("initial_alignment_check").as_bool();
  max_initial_difference_ = node->get_parameter("max_initial_difference").as_double();
  alignment_duration_seconds_ = node->get_parameter("alignment_duration_sec").as_double();
  lower_limits_ = node->get_parameter("lower_limits").as_double_array();
  upper_limits_ = node->get_parameter("upper_limits").as_double_array();
  max_velocities_ = node->get_parameter("max_velocities").as_double_array();
  input_indices_ = node->get_parameter("input_indices").as_integer_array();
  command_scales_ = node->get_parameter("command_scales").as_double_array();
  command_offsets_ = node->get_parameter("command_offsets").as_double_array();

  if (joints_.empty() || configured_size < 0 || timeout_ms <= 0 ||
    max_initial_difference_ < 0.0 || alignment_duration_seconds_ < 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid OpenArm controller parameters");
    return CallbackReturn::ERROR;
  }
  expected_command_size_ = configured_size == 0 ? joints_.size() :
    static_cast<std::size_t>(configured_size);
  if (input_indices_.empty()) {
    input_indices_.resize(joints_.size());
    for (std::size_t i = 0; i < input_indices_.size(); ++i) {
      input_indices_[i] = static_cast<std::int64_t>(i);
    }
  }
  if (input_indices_.size() != joints_.size() ||
    !normalizeLimits(&lower_limits_, joints_.size(), -std::numeric_limits<double>::infinity()) ||
    !normalizeLimits(&upper_limits_, joints_.size(), std::numeric_limits<double>::infinity()) ||
    !normalizeLimits(&max_velocities_, joints_.size(), std::numeric_limits<double>::infinity()) ||
    !normalizeLimits(&command_scales_, joints_.size(), 1.0) ||
    !normalizeLimits(&command_offsets_, joints_.size(), 0.0))
  {
    RCLCPP_ERROR(node->get_logger(), "Joint, command, and limit vector sizes must match");
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (lower_limits_[i] > upper_limits_[i] || max_velocities_[i] <= 0.0 ||
      input_indices_[i] < 0 ||
      static_cast<std::size_t>(input_indices_[i]) >= expected_command_size_ ||
      !std::isfinite(command_scales_[i]) || !std::isfinite(command_offsets_[i]))
    {
      RCLCPP_ERROR(node->get_logger(), "Invalid limits for joint %s", joints_[i].c_str());
      return CallbackReturn::ERROR;
    }
  }
  command_timeout_seconds_ = static_cast<double>(timeout_ms) / 1000.0;
  alignment_start_positions_.assign(joints_.size(), 0.0);
  last_output_.assign(joints_.size(), 0.0);
  command_buffer_.writeFromNonRT(CommandFrame{});

  command_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    std::bind(&OpenArmPositionController::commandCallback, this, std::placeholders::_1));
  status_publisher_ = node->create_publisher<x1_coffee_interfaces::msg::TeleopStatus>(
    "~/status", rclcpp::SystemDefaultsQoS());
  status_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&OpenArmPositionController::publishStatus, this));
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OpenArmPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
OpenArmPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

CallbackReturn OpenArmPositionController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joints_.size() || state_interfaces_.size() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Claimed interface count does not match configured joints");
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double measured = state_interfaces_[i].get_value();
    if (!std::isfinite(measured)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Initial state is not finite for %s", joints_[i].c_str());
      return CallbackReturn::ERROR;
    }
    command_interfaces_[i].set_value(measured);
    alignment_start_positions_[i] = measured;
    last_output_[i] = measured;
  }
  command_buffer_.writeFromNonRT(CommandFrame{});
  aligned_ = false;
  alignment_started_ = false;
  command_fresh_.store(false);
  status_aligned_.store(false);
  last_error_code_.store(kOk);
  active_.store(true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmPositionController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_.store(false);
  command_fresh_.store(false);
  aligned_ = false;
  alignment_started_ = false;
  status_aligned_.store(false);
  return CallbackReturn::SUCCESS;
}

void OpenArmPositionController::commandCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  CommandFrame frame;
  frame.received_at = get_node()->now();
  if (message->data.size() != expected_command_size_) {
    reject(kWrongLength);
    command_buffer_.writeFromNonRT(frame);
    return;
  }
  frame.positions.resize(joints_.size());
  for (std::size_t i = 0; i < frame.positions.size(); ++i) {
    const double input = message->data[static_cast<std::size_t>(input_indices_[i])];
    if (!std::isfinite(input)) {
      reject(kNotFinite);
      command_buffer_.writeFromNonRT(frame);
      return;
    }
    frame.positions[i] = command_scales_[i] * input + command_offsets_[i];
    if (!std::isfinite(frame.positions[i])) {
      reject(kNotFinite);
      command_buffer_.writeFromNonRT(frame);
      return;
    }
    if (frame.positions[i] < lower_limits_[i] || frame.positions[i] > upper_limits_[i]) {
      reject(kPositionLimit);
      command_buffer_.writeFromNonRT(frame);
      return;
    }
  }
  frame.valid = true;
  accepted_frames_.fetch_add(1);
  last_error_code_.store(kOk);
  command_buffer_.writeFromNonRT(frame);
}

controller_interface::return_type OpenArmPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto * frame = command_buffer_.readFromRT();
  if (frame == nullptr || !frame->valid) {
    command_fresh_.store(false);
    for (std::size_t i = 0; i < last_output_.size(); ++i) {
      command_interfaces_[i].set_value(last_output_[i]);
    }
    return controller_interface::return_type::OK;
  }
  const double age = (time - frame->received_at).seconds();
  if (age < 0.0 || age > command_timeout_seconds_) {
    command_fresh_.store(false);
    last_error_code_.store(kTimeout);
    for (std::size_t i = 0; i < last_output_.size(); ++i) {
      command_interfaces_[i].set_value(last_output_[i]);
    }
    return controller_interface::return_type::OK;
  }
  command_fresh_.store(true);

  if (!alignment_started_) {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      alignment_start_positions_[i] = state_interfaces_[i].get_value();
      if (initial_alignment_check_ &&
        std::abs(frame->positions[i] - alignment_start_positions_[i]) > max_initial_difference_)
      {
        if (last_error_code_.exchange(kInitialDifference) != kInitialDifference) {
          rejected_frames_.fetch_add(1);
        }
        command_fresh_.store(false);
        return controller_interface::return_type::OK;
      }
    }
    alignment_started_at_ = time;
    alignment_started_ = true;
  }

  double alpha = 1.0;
  if (!aligned_ && alignment_duration_seconds_ > 0.0) {
    alpha = std::clamp((time - alignment_started_at_).seconds() / alignment_duration_seconds_, 0.0, 1.0);
  }
  if (alpha >= 1.0) {
    aligned_ = true;
    status_aligned_.store(true);
  }

  const double dt = std::max(period.seconds(), 0.0);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    double target = alignment_start_positions_[i] +
      alpha * (frame->positions[i] - alignment_start_positions_[i]);
    if (std::isfinite(max_velocities_[i]) && dt > 0.0) {
      const double delta = max_velocities_[i] * dt;
      target = std::clamp(target, last_output_[i] - delta, last_output_[i] + delta);
    }
    last_output_[i] = target;
    command_interfaces_[i].set_value(target);
  }
  return controller_interface::return_type::OK;
}

void OpenArmPositionController::reject(ErrorCode code)
{
  rejected_frames_.fetch_add(1);
  last_error_code_.store(code);
}

const char * OpenArmPositionController::errorDetail(std::uint32_t code)
{
  switch (code) {
    case kOk: return "ok";
    case kWrongLength: return "command length does not match configured joints";
    case kNotFinite: return "command contains NaN or Inf";
    case kPositionLimit: return "command exceeds configured position limit";
    case kInitialDifference: return "first command is too far from measured pose";
    case kTimeout: return "command watchdog timeout; holding last output";
    default: return "unknown error";
  }
}

void OpenArmPositionController::publishStatus()
{
  if (!status_publisher_) {
    return;
  }
  x1_coffee_interfaces::msg::TeleopStatus message;
  message.header.stamp = get_node()->now();
  message.controller_name = get_node()->get_name();
  message.active = active_.load();
  message.aligned = status_aligned_.load();
  message.command_fresh = command_fresh_.load();
  message.accepted_frames = accepted_frames_.load();
  message.rejected_frames = rejected_frames_.load();
  message.last_error_code = last_error_code_.load();
  message.last_error_detail = errorDetail(message.last_error_code);
  status_publisher_->publish(message);
}

}  // namespace x1_coffee_openarm_compat

PLUGINLIB_EXPORT_CLASS(
  x1_coffee_openarm_compat::OpenArmPositionController,
  controller_interface::ControllerInterface)
