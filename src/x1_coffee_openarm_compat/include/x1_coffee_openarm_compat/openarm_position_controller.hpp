#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "x1_coffee_interfaces/msg/teleop_status.hpp"

namespace x1_coffee_openarm_compat
{

class OpenArmPositionController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum ErrorCode : std::uint32_t
  {
    kOk = 0,
    kWrongLength = 1,
    kNotFinite = 2,
    kPositionLimit = 3,
    kInitialDifference = 4,
    kTimeout = 5,
  };

  struct CommandFrame
  {
    std::vector<double> positions;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void publishStatus();
  void reject(ErrorCode code);
  static const char * errorDetail(std::uint32_t code);

  std::vector<std::string> joints_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> max_velocities_;
  std::vector<std::int64_t> input_indices_;
  std::vector<double> command_scales_;
  std::vector<double> command_offsets_;
  std::vector<double> alignment_start_positions_;
  std::vector<double> last_output_;
  std::size_t expected_command_size_{0};
  double command_timeout_seconds_{0.1};
  bool initial_alignment_check_{true};
  double max_initial_difference_{0.35};
  double alignment_duration_seconds_{3.0};
  bool aligned_{false};
  bool alignment_started_{false};
  rclcpp::Time alignment_started_at_{0, 0, RCL_ROS_TIME};

  realtime_tools::RealtimeBuffer<CommandFrame> command_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;
  rclcpp::Publisher<x1_coffee_interfaces::msg::TeleopStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::atomic<bool> active_{false};
  std::atomic<bool> command_fresh_{false};
  std::atomic<bool> status_aligned_{false};
  std::atomic<std::uint32_t> accepted_frames_{0};
  std::atomic<std::uint32_t> rejected_frames_{0};
  std::atomic<std::uint32_t> last_error_code_{kOk};
};

}  // namespace x1_coffee_openarm_compat
