#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "x1_coffee_interfaces/srv/safety_command.hpp"

namespace x1_coffee_device_sdk
{

class SafetyServiceServer final : public rclcpp::Node
{
public:
  using SafetyCommand = x1_coffee_interfaces::srv::SafetyCommand;
  using Trigger = std_srvs::srv::Trigger;

  SafetyServiceServer()
  : Node("x1_coffee_safety_service_server")
  {
    simulate_ = declare_parameter<bool>("simulate", false);
    command_publisher_ = create_publisher<std_msgs::msg::String>(
      "/x1/safety/backend_command", rclcpp::SystemDefaultsQoS());
    command_service_ = create_service<SafetyCommand>(
      "/x1/safety/command",
      [this](const SafetyCommand::Request::SharedPtr request,
        SafetyCommand::Response::SharedPtr response)
      {
        dispatch(request->command, request->reason, *response);
      });
    pause_service_ = createTrigger("/x1/safety/pause", SafetyCommand::Request::PAUSE);
    resume_service_ = createTrigger("/x1/safety/resume", SafetyCommand::Request::RESUME);
    reset_service_ = createTrigger("/x1/safety/fault_reset", SafetyCommand::Request::FAULT_RESET);
    emergency_service_ = createTrigger(
      "/x1/safety/emergency_stop", SafetyCommand::Request::EMERGENCY_STOP);
  }

private:
  rclcpp::Service<Trigger>::SharedPtr createTrigger(const std::string & name, uint8_t command)
  {
    return create_service<Trigger>(
      name, [this, command, name](const Trigger::Request::SharedPtr,
        Trigger::Response::SharedPtr response)
      {
        SafetyCommand::Response custom_response;
        dispatch(command, name, custom_response);
        response->success = custom_response.accepted;
        response->message = custom_response.message;
      });
  }

  void dispatch(uint8_t command, const std::string & reason, SafetyCommand::Response & response)
  {
    if (command < SafetyCommand::Request::PAUSE ||
      command > SafetyCommand::Request::EMERGENCY_STOP)
    {
      response.accepted = false;
      response.message = "unknown safety command";
      return;
    }
    if (!simulate_) {
      response.accepted = false;
      response.message =
        "real safety backend is not implemented; connect the GTN/safety adapter first";
      return;
    }
    std_msgs::msg::String message;
    message.data = std::to_string(command) + ":" + reason;
    command_publisher_->publish(message);
    response.accepted = true;
    response.message = "accepted by explicit fake safety backend";
  }

  bool simulate_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  rclcpp::Service<SafetyCommand>::SharedPtr command_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::Service<Trigger>::SharedPtr emergency_service_;
};

}  // namespace x1_coffee_device_sdk

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x1_coffee_device_sdk::SafetyServiceServer>());
  rclcpp::shutdown();
  return 0;
}
