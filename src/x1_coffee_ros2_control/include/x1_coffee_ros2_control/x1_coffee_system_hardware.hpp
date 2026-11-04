#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "x1_coffee_hardware_core/command_guard.hpp"
#include "x1_coffee_hardware_core/coffee_axis_model.hpp"
#include "x1_coffee_hardware_core/gtn_transport.hpp"

namespace x1_coffee_ros2_control
{

class X1CoffeeSystemHardware final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(X1CoffeeSystemHardware)

  ~X1CoffeeSystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool parseLimits();
  bool createTransport();
  bool startMotionService();
  void scheduleMotionService();
  void allowMotionFromMeasurement(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void logTransportError(const char * operation, const x1_coffee_hardware_core::Result & result);
  bool parseRuntimeParameters();

  rclcpp::Node::SharedPtr motion_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_motion_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_tracked_service_;
  std::atomic<bool> snap_commands_to_measurement_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_torque_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr brake_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mit_hold_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mit_exit_service_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pvt_move_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pvt_result_pub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr motion_executor_;
  std::thread motion_spin_thread_;
  std::thread motion_service_thread_;

  std::unique_ptr<x1_coffee_hardware_core::GtnTransport> transport_;
  std::unique_ptr<x1_coffee_hardware_core::CommandGuard> guard_;
  const x1_coffee_hardware_core::HardwareLayout * layout_{
    &x1_coffee_hardware_core::defaultHardwareLayout()};
  x1_coffee_hardware_core::CoffeeAxisLimits limits_{};
  std::array<double, x1_coffee_hardware_core::kCoffeeAxisCount> positions_{};
  std::array<double, x1_coffee_hardware_core::kCoffeeAxisCount> velocities_{};
  std::array<double, x1_coffee_hardware_core::kCoffeeAxisCount> commands_{};
  std::array<double, x1_coffee_hardware_core::kCoffeeAxisCount> last_sent_commands_{};
  std::array<std::array<double, 14>, x1_coffee_hardware_core::kCoffeeAxisCount>
    diagnostic_states_{};
  std::array<std::size_t, x1_coffee_hardware_core::kCoffeeAxisCount> declared_to_canonical_{};
  int state_timeout_ms_{100};
  int command_timeout_ms_{100};
  int motion_startup_grace_ms_{3000};
  bool motion_enabled_{false};
  bool stop_on_axis_alarm_{true};
  bool stop_on_emergency_input_{true};
  std::chrono::steady_clock::time_point last_read_at_{};
  std::chrono::steady_clock::time_point last_write_at_{};
  std::chrono::steady_clock::time_point activation_completed_at_{};
  bool has_read_sample_{false};
  bool has_write_sample_{false};
  bool active_{false};
  bool motion_started_{false};
  std::uint64_t read_call_count_{0};
  std::uint64_t write_call_count_{0};
  std::chrono::steady_clock::time_point last_read_call_at_{};
  std::chrono::steady_clock::time_point last_write_call_at_{};
  std::chrono::steady_clock::time_point cycle_report_started_at_{};
  std::uint64_t cycle_report_read_start_{0};
  std::uint64_t cycle_report_write_start_{0};
  std::chrono::steady_clock::time_point last_read_warning_at_{};
  std::chrono::steady_clock::time_point last_write_warning_at_{};
  std::chrono::steady_clock::time_point last_guard_hold_log_at_{};
};

}  // namespace x1_coffee_ros2_control
