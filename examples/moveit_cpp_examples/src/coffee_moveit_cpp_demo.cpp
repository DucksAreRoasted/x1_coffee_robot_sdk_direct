#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::vector<double> safeTarget(const std::vector<double> & current, const std::string & mode)
{
  auto target = current;
  if (!target.empty()) {
    target.front() += 0.05;
  }
  if (mode == "both" && target.size() >= 14) {
    target[7] -= 0.05;
  }
  return target;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "left";
  const bool execute = argc > 2 && std::string(argv[2]) == "--execute";
  const std::string group_name = mode == "right" ? "right_group" :
    (mode == "both" ? "both_arms" : "left_group");

  auto node = std::make_shared<rclcpp::Node>(
    "x1_coffee_moveit_cpp_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setStartStateToCurrentState();

  const auto current = move_group.getCurrentJointValues();
  if (current.empty()) {
    RCLCPP_ERROR(node->get_logger(), "No current joint state for group %s", group_name.c_str());
    executor.cancel();
    rclcpp::shutdown();
    spinner.join();
    return 2;
  }
  if (!move_group.setJointValueTarget(safeTarget(current, mode))) {
    RCLCPP_ERROR(node->get_logger(), "Unable to set a safe joint target for %s", group_name.c_str());
    executor.cancel();
    rclcpp::shutdown();
    spinner.join();
    return 3;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = move_group.plan(plan);
  if (planned != moveit::core::MoveItErrorCode::SUCCESS ||
    plan.trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed for %s", group_name.c_str());
    executor.cancel();
    rclcpp::shutdown();
    spinner.join();
    return 4;
  }
  RCLCPP_INFO(
    node->get_logger(), "Planning succeeded for %s with %zu joints and %zu points",
    group_name.c_str(), plan.trajectory.joint_trajectory.joint_names.size(),
    plan.trajectory.joint_trajectory.points.size());

  if (execute) {
    const auto executed = move_group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Execution failed for %s", group_name.c_str());
      executor.cancel();
      rclcpp::shutdown();
      spinner.join();
      return 5;
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Plan-only mode; pass --execute only after real-hardware acceptance");
  }

  executor.cancel();
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
