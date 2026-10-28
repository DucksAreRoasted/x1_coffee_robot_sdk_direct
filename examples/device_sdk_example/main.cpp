#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "x1_coffee_device_sdk/axis_clients.hpp"
#include "x1_coffee_device_sdk/motor_state_reader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("x1_coffee_device_sdk_example");

  x1_coffee_device_sdk::WaistClient waist(node);
  x1_coffee_device_sdk::MotorStateReader motors(node);
  motors.setCallback(
    [node](const auto & message) {
      RCLCPP_INFO(node->get_logger(), "Received %zu real motor states", message.states.size());
    });

  // Absolute SI-unit commands. The matching forward controllers must own these axes.
  waist.commandRadians(0.0);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
