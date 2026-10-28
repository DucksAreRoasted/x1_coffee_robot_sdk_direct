#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/conversions.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kLeftGroup[] = "left_arm";
constexpr char kRightGroup[] = "right_arm";
constexpr char kBothGroup[] = "both_arms";
constexpr char kLeftTip[] = "left_gripper_tcp";
constexpr char kRightTip[] = "right_gripper_tcp";

void stopExecutor(
  rclcpp::executors::MultiThreadedExecutor & executor, std::thread & spinner)
{
  executor.cancel();
  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const bool execute = argc > 1 && std::string(argv[1]) == "--execute";
  auto node = std::make_shared<rclcpp::Node>(
    "x1_coffee_bimanual_cartesian_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface both_group(node, kBothGroup);
  both_group.setPlanningTime(10.0);
  both_group.setNumPlanningAttempts(3);
  both_group.setMaxVelocityScalingFactor(0.1);
  both_group.setMaxAccelerationScalingFactor(0.1);

  const auto current_state = both_group.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(node->get_logger(), "无法取得当前 16 轴关节状态");
    stopExecutor(executor, spinner);
    return 2;
  }

  const auto * left_model = current_state->getJointModelGroup(kLeftGroup);
  const auto * right_model = current_state->getJointModelGroup(kRightGroup);
  const auto * both_model = current_state->getJointModelGroup(kBothGroup);
  if (left_model == nullptr || right_model == nullptr || both_model == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "SRDF 缺少 left_arm、right_arm 或 both_arms");
    stopExecutor(executor, spinner);
    return 3;
  }

  moveit::core::RobotState target_state(*current_state);
  auto left_target = current_state->getGlobalLinkTransform(kLeftTip);
  auto right_target = current_state->getGlobalLinkTransform(kRightTip);

  // 验收演示使用当前末端附近 2 cm 的小幅目标；项目集成时由甲方 VR/重定向层替换。
  left_target.translation().z() += 0.02;
  right_target.translation().z() += 0.02;

  if (!target_state.setFromIK(left_model, left_target, kLeftTip, 0.2)) {
    RCLCPP_ERROR(node->get_logger(), "左末端 IK 求解失败");
    stopExecutor(executor, spinner);
    return 4;
  }
  target_state.update();
  if (!target_state.setFromIK(right_model, right_target, kRightTip, 0.2)) {
    RCLCPP_ERROR(node->get_logger(), "右末端 IK 求解失败");
    stopExecutor(executor, spinner);
    return 5;
  }
  target_state.update();

  moveit_msgs::msg::RobotState target_message;
  moveit::core::robotStateToRobotStateMsg(target_state, target_message);
  auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity");
  if (!validity_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt 状态有效性检查服务不可用");
    stopExecutor(executor, spinner);
    return 6;
  }
  auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  validity_request->robot_state = target_message;
  validity_request->group_name = kBothGroup;
  auto validity_future = validity_client->async_send_request(validity_request);
  if (validity_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "跨臂碰撞/状态有效性检查超时");
    stopExecutor(executor, spinner);
    return 7;
  }
  const auto validity = validity_future.get();
  if (!validity->valid) {
    RCLCPP_ERROR(
      node->get_logger(), "双末端 IK 合并状态存在碰撞或约束错误，联系人数量=%zu",
      validity->contacts.size());
    stopExecutor(executor, spinner);
    return 8;
  }

  std::vector<double> joint_target;
  target_state.copyJointGroupPositions(both_model, joint_target);
  both_group.setStartState(*current_state);
  if (!both_group.setJointValueTarget(joint_target)) {
    RCLCPP_ERROR(node->get_logger(), "无法设置 14 轴 both_arms 联合目标");
    stopExecutor(executor, spinner);
    return 9;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = both_group.plan(plan);
  if (planned != moveit::core::MoveItErrorCode::SUCCESS ||
    plan.trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "14 轴双臂联合规划失败");
    stopExecutor(executor, spinner);
    return 10;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "双末端 IK、跨臂状态检查和 both_arms 联合规划通过：joints=%zu points=%zu",
    plan.trajectory.joint_trajectory.joint_names.size(),
    plan.trajectory.joint_trajectory.points.size());

  if (execute) {
    RCLCPP_WARN(node->get_logger(), "即将请求真实双臂同步执行；必须先完成实机安全验收");
    const auto executed = both_group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "双臂联合轨迹执行失败");
      stopExecutor(executor, spinner);
      return 11;
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "当前为只规划模式；实机验收后才可传入 --execute");
  }

  stopExecutor(executor, spinner);
  return 0;
}
