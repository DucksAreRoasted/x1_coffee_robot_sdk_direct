#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace x1_coffee_device_sdk
{

class GripperActionServer final : public rclcpp::Node
{
public:
  using Action = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  GripperActionServer()
  : Node("x1_coffee_gripper_action_server")
  {
    simulate_ = declare_parameter<bool>("simulate", false);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.001);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 5.0);
    configureSide(left_, "left");
    configureSide(right_, "right");
    RCLCPP_INFO(
      get_logger(), "Gripper Action adapters ready (simulate=%s)", simulate_ ? "true" : "false");
  }

private:
  struct Side
  {
    std::string name;
    double minimum{0.0};
    double maximum{0.044};
    std::mutex mutex;
    std::optional<double> position;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_subscription;
    rclcpp_action::Server<Action>::SharedPtr action_server;
  };

  void configureSide(Side & side, const std::string & name)
  {
    side.name = name;
    const auto prefix = name + ".";
    const auto action_name = declare_parameter<std::string>(
      prefix + "action_name", "/" + name + "_gripper_controller/gripper_cmd");
    const auto command_topic = declare_parameter<std::string>(
      prefix + "command_topic", "/" + name + "_gripper/commands");
    const auto state_topic = declare_parameter<std::string>(
      prefix + "state_topic", "/" + name + "_gripper/state");
    side.minimum = declare_parameter<double>(prefix + "min_position", 0.0);
    side.maximum = declare_parameter<double>(prefix + "max_position", 0.044);
    if (!(std::isfinite(side.minimum) && std::isfinite(side.maximum)) ||
      side.minimum >= side.maximum)
    {
      throw std::invalid_argument(name + " gripper limits are invalid");
    }
    if (simulate_) {
      side.position = side.minimum;
    }
    side.command_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic, rclcpp::SystemDefaultsQoS());
    side.state_subscription = create_subscription<std_msgs::msg::Float64>(
      state_topic, rclcpp::SensorDataQoS(),
      [&side](const std_msgs::msg::Float64::SharedPtr message) {
        if (std::isfinite(message->data)) {
          std::scoped_lock lock(side.mutex);
          side.position = message->data;
        }
      });
    side.action_server = rclcpp_action::create_server<Action>(
      this, action_name,
      [this, &side](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        const double target = goal->command.position;
        if (!std::isfinite(target) || target < side.minimum || target > side.maximum ||
          !std::isfinite(goal->command.max_effort))
        {
          RCLCPP_WARN(get_logger(), "Rejected invalid %s gripper goal", side.name.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this, &side](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, &side, goal_handle]() {execute(side, goal_handle);}).detach();
      });
  }

  void execute(Side & side, const std::shared_ptr<GoalHandle> goal_handle)
  {
    const double target = goal_handle->get_goal()->command.position;
    std_msgs::msg::Float64MultiArray command;
    command.data = {target};
    side.command_publisher->publish(command);

    const auto started = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(timeout_sec_);
    const auto result = std::make_shared<Action::Result>();
    const auto feedback = std::make_shared<Action::Feedback>();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->position = currentPosition(side).value_or(target);
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = false;
        goal_handle->canceled(result);
        return;
      }

      if (simulate_ &&
        std::chrono::steady_clock::now() - started > std::chrono::milliseconds(100))
      {
        std::scoped_lock lock(side.mutex);
        side.position = target;
      }

      const auto position = currentPosition(side);
      feedback->position = position.value_or(0.0);
      feedback->effort = 0.0;
      feedback->stalled = false;
      feedback->reached_goal = position && std::abs(*position - target) <= goal_tolerance_;
      goal_handle->publish_feedback(feedback);
      if (feedback->reached_goal) {
        result->position = *position;
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = true;
        goal_handle->succeed(result);
        return;
      }
      if (std::chrono::steady_clock::now() - started > timeout) {
        result->position = position.value_or(0.0);
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = false;
        RCLCPP_ERROR(
          get_logger(), "%s gripper goal timed out; no verified hardware state reached the target",
          side.name.c_str());
        goal_handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  static std::optional<double> currentPosition(Side & side)
  {
    std::scoped_lock lock(side.mutex);
    return side.position;
  }

  bool simulate_{false};
  double goal_tolerance_{0.001};
  double timeout_sec_{5.0};
  Side left_;
  Side right_;
};

}  // namespace x1_coffee_device_sdk

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<x1_coffee_device_sdk::GripperActionServer>());
  rclcpp::shutdown();
  return 0;
}
