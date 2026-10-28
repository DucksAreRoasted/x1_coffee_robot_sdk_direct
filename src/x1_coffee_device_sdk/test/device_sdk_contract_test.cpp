#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "x1_coffee_device_sdk/axis_clients.hpp"
#include "x1_coffee_device_sdk/gripper_client.hpp"
#include "x1_coffee_interfaces/srv/safety_command.hpp"

#include <csignal>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char ** environ;

namespace
{

struct RosDomainGuard
{
  RosDomainGuard()
  {
    setenv("ROS_DOMAIN_ID", "47", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    std::filesystem::create_directories("contract_test_ros_log");
    setenv("ROS_LOG_DIR", "contract_test_ros_log", 1);
  }
};

RosDomainGuard ros_domain_guard;

std::filesystem::path safetyServerExecutable()
{
  const char * raw = std::getenv("AMENT_PREFIX_PATH");
  if (raw == nullptr) {
    return {};
  }
  std::stringstream stream{raw};
  std::string entry;
  while (std::getline(stream, entry, ':')) {
    const std::filesystem::path candidate =
      std::filesystem::path(entry) / "lib" / "x1_coffee_device_sdk" / "safety_service_server";
    if (std::filesystem::is_regular_file(candidate)) {
      return candidate;
    }
  }
  return {};
}

class SafetyServerProcess
{
public:
  bool start(const std::filesystem::path & executable, bool simulate = false)
  {
    const std::string binary = executable.string();
    // 参数字符串必须活过 posix_spawn；该调用返回前会把参数拷进子进程。
    const std::string simulate_arg = simulate ? "simulate:=true" : "simulate:=false";
    const char * argv[] = {
      binary.c_str(),
      "--ros-args",
      "-p",
      simulate_arg.c_str(),
      nullptr};
    const int rc = posix_spawn(
      &pid_, binary.c_str(), nullptr, nullptr, const_cast<char **>(argv), environ);
    return rc == 0 && pid_ > 0;
  }

  ~SafetyServerProcess()
  {
    stop();
  }

  void stop()
  {
    if (pid_ <= 0) {
      return;
    }
    kill(pid_, SIGINT);
    waitpid(pid_, nullptr, 0);
    pid_ = -1;
  }

private:
  pid_t pid_{-1};
};

class DeviceSdkContract : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("device_sdk_contract_test");
  }

  static void TearDownTestSuite()
  {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static rclcpp::Node::SharedPtr node_;
};

rclcpp::Node::SharedPtr DeviceSdkContract::node_;

constexpr const char kSafetyBackendMissing[] =
  "real safety backend is not implemented; connect the GTN/safety adapter first";

// 同一节点上订阅 Float64MultiArray，用来核对“成功”是否真的把原值交出去。
class Float64Capture
{
public:
  explicit Float64Capture(const std::string & topic)
  {
    subscription_ = DeviceSdkContract::node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
        latest_ = std::move(message);
      });
  }

  bool waitForPublisher(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (subscription_->get_publisher_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(DeviceSdkContract::node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return subscription_->get_publisher_count() > 0;
  }

  std::shared_ptr<const std_msgs::msg::Float64MultiArray> waitForSample(
    std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!latest_ && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(DeviceSdkContract::node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto received = latest_;
    latest_.reset();
    return received;
  }

  void clear()
  {
    latest_.reset();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  std::shared_ptr<const std_msgs::msg::Float64MultiArray> latest_;
};

struct SafetyReply
{
  bool completed{false};
  bool accepted{false};
  std::string message;
};

SafetyReply callSafetyCommand(uint8_t command, const std::string & reason)
{
  SafetyReply reply;
  using SafetyCommand = x1_coffee_interfaces::srv::SafetyCommand;
  auto client = DeviceSdkContract::node_->create_client<SafetyCommand>("/x1/safety/command");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    return reply;
  }
  auto request = std::make_shared<SafetyCommand::Request>();
  request->command = command;
  request->reason = reason;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      DeviceSdkContract::node_, future, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return reply;
  }
  const auto response = future.get();
  reply.completed = true;
  reply.accepted = response->accepted;
  reply.message = response->message;
  return reply;
}

}  // namespace

TEST_F(DeviceSdkContract, waist_rejects_non_finite_radians)
{
  x1_coffee_device_sdk::WaistClient waist(node_);
  const x1_coffee_device_sdk::DeviceResult result =
    waist.commandRadians(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(result);
  EXPECT_EQ(result.detail, "position must be finite");
}

TEST_F(DeviceSdkContract, waist_accepts_finite_radians)
{
  x1_coffee_device_sdk::WaistClient waist(node_);
  EXPECT_TRUE(waist.commandRadians(0.0));
}

TEST_F(DeviceSdkContract, gripper_move_to_rejects_position_outside_configured_range)
{
  x1_coffee_device_sdk::GripperClient gripper(node_, "/contract_test/gripper", 0.0, 1.0);
  const x1_coffee_device_sdk::DeviceResult result = gripper.moveTo(1.5);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.detail, "gripper position is outside configured range");
}

TEST_F(DeviceSdkContract, gripper_move_to_accepts_position_inside_configured_range)
{
  x1_coffee_device_sdk::GripperClient gripper(node_, "/contract_test/gripper", 0.0, 1.0);
  EXPECT_TRUE(gripper.moveTo(0.25));
}

TEST_F(DeviceSdkContract, safety_service_rejects_when_simulate_is_false)
{
  const std::filesystem::path executable = safetyServerExecutable();
  ASSERT_FALSE(executable.empty());
  SafetyServerProcess server;
  ASSERT_TRUE(server.start(executable));

  using SafetyCommand = x1_coffee_interfaces::srv::SafetyCommand;
  auto client = node_->create_client<SafetyCommand>("/x1/safety/command");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<SafetyCommand::Request>();
  request->command = SafetyCommand::Request::EMERGENCY_STOP;
  request->reason = "contract-test";
  auto future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();
  EXPECT_FALSE(response->accepted);
  EXPECT_EQ(
    response->message,
    "real safety backend is not implemented; connect the GTN/safety adapter first");
}

::testing::AssertionResult sampleEquals(
  const std::shared_ptr<const std_msgs::msg::Float64MultiArray> & sample, double value)
{
  if (!sample) {
    return ::testing::AssertionFailure() << "发布器没有交出样本";
  }
  if (sample->data.size() != 1u) {
    return ::testing::AssertionFailure() << "data 长度不是 1，实际 " << sample->data.size();
  }
  if (sample->data[0] != value) {
    return ::testing::AssertionFailure() << sample->data[0] << " != " << value;
  }
  return ::testing::AssertionSuccess();
}

TEST_F(DeviceSdkContract, waist_rejects_infinities_and_does_not_rescale_large_radians)
{
  Float64Capture capture("/waist_forward_position_controller/commands");
  x1_coffee_device_sdk::WaistClient waist(node_);
  ASSERT_TRUE(capture.waitForPublisher(std::chrono::seconds(3)));

  for (const double illegal : {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    capture.clear();
    const x1_coffee_device_sdk::DeviceResult rejected = waist.commandRadians(illegal);
    EXPECT_FALSE(rejected);
    EXPECT_EQ(rejected.detail, "position must be finite");
    EXPECT_EQ(capture.waitForSample(std::chrono::milliseconds(200)), nullptr);
  }

  // 7 rad 约 401°，规格明确不把 0–360° 写进腰客户端，也不做角度换算。
  const x1_coffee_device_sdk::DeviceResult large = waist.commandRadians(7.0);
  EXPECT_TRUE(large);
  EXPECT_TRUE(large.detail.empty());
  EXPECT_TRUE(sampleEquals(capture.waitForSample(std::chrono::seconds(2)), 7.0));

  const x1_coffee_device_sdk::DeviceResult negative = waist.commandRadians(-0.4);
  EXPECT_TRUE(negative);
  EXPECT_TRUE(negative.detail.empty());
  EXPECT_TRUE(sampleEquals(capture.waitForSample(std::chrono::seconds(2)), -0.4));
}

TEST_F(DeviceSdkContract, gripper_move_to_rejects_non_finite_position)
{
  Float64Capture capture("/contract_test/gripper_non_finite");
  x1_coffee_device_sdk::GripperClient gripper(node_, "/contract_test/gripper_non_finite", 0.0, 1.0);
  ASSERT_TRUE(capture.waitForPublisher(std::chrono::seconds(3)));
  for (const double illegal : {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    capture.clear();
    const x1_coffee_device_sdk::DeviceResult result = gripper.moveTo(illegal);
    EXPECT_FALSE(result);
    EXPECT_EQ(result.detail, "gripper position must be finite");
    EXPECT_EQ(capture.waitForSample(std::chrono::milliseconds(200)), nullptr);
  }
}

TEST_F(DeviceSdkContract, gripper_closed_interval_includes_endpoints_when_limits_are_swapped)
{
  Float64Capture capture("/contract_test/gripper_bounds");
  x1_coffee_device_sdk::GripperClient gripper(node_, "/contract_test/gripper_bounds", 1.0, 0.0);
  ASSERT_TRUE(capture.waitForPublisher(std::chrono::seconds(3)));

  const x1_coffee_device_sdk::DeviceResult lower = gripper.moveTo(0.0);
  EXPECT_TRUE(lower);
  EXPECT_TRUE(lower.detail.empty());
  EXPECT_TRUE(sampleEquals(capture.waitForSample(std::chrono::seconds(2)), 0.0));

  const x1_coffee_device_sdk::DeviceResult upper = gripper.moveTo(1.0);
  EXPECT_TRUE(upper);
  EXPECT_TRUE(sampleEquals(capture.waitForSample(std::chrono::seconds(2)), 1.0));

  EXPECT_TRUE(gripper.open());
  EXPECT_TRUE(sampleEquals(capture.waitForSample(std::chrono::seconds(2)), 1.0));
  EXPECT_TRUE(gripper.close());
  EXPECT_TRUE(sampleEquals(capture.waitForSample(std::chrono::seconds(2)), 0.0));

  capture.clear();
  const x1_coffee_device_sdk::DeviceResult below = gripper.moveTo(std::nextafter(0.0, -1.0));
  EXPECT_FALSE(below);
  EXPECT_EQ(below.detail, "gripper position is outside configured range");
  EXPECT_EQ(capture.waitForSample(std::chrono::milliseconds(200)), nullptr);
  capture.clear();
  const x1_coffee_device_sdk::DeviceResult above = gripper.moveTo(std::nextafter(1.0, 2.0));
  EXPECT_FALSE(above);
  EXPECT_EQ(above.detail, "gripper position is outside configured range");
  EXPECT_EQ(capture.waitForSample(std::chrono::milliseconds(200)), nullptr);
}

TEST_F(DeviceSdkContract, gripper_constructor_rejects_invalid_configuration)
{
  EXPECT_THROW(
    x1_coffee_device_sdk::GripperClient(node_, "", 0.0, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    x1_coffee_device_sdk::GripperClient(node_, "/contract_test/gripper_ctor", 0.2, 0.2),
    std::invalid_argument);
  EXPECT_THROW(
    x1_coffee_device_sdk::GripperClient(node_, "/contract_test/gripper_ctor", -0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    x1_coffee_device_sdk::GripperClient(
      node_, "/contract_test/gripper_ctor", std::numeric_limits<double>::quiet_NaN(), 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    x1_coffee_device_sdk::GripperClient(nullptr, "/contract_test/gripper_ctor", 0.0, 1.0),
    std::invalid_argument);
}

TEST_F(DeviceSdkContract, safety_codes_stay_one_through_four_and_simulate_flag_is_honored)
{
  using SafetyCommand = x1_coffee_interfaces::srv::SafetyCommand;
  EXPECT_EQ(SafetyCommand::Request::PAUSE, 1);
  EXPECT_EQ(SafetyCommand::Request::RESUME, 2);
  EXPECT_EQ(SafetyCommand::Request::FAULT_RESET, 3);
  EXPECT_EQ(SafetyCommand::Request::EMERGENCY_STOP, 4);

  const std::filesystem::path executable = safetyServerExecutable();
  ASSERT_FALSE(executable.empty());

  std::shared_ptr<const std_msgs::msg::String> backend_command;
  auto backend = node_->create_subscription<std_msgs::msg::String>(
    "/x1/safety/backend_command", rclcpp::SystemDefaultsQoS(),
    [&backend_command](std_msgs::msg::String::SharedPtr message) {
      backend_command = std::move(message);
    });

  auto wait_for_backend_publisher = [&backend]() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (backend->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return backend->get_publisher_count() > 0;
  };

  {
    SafetyServerProcess server;
    ASSERT_TRUE(server.start(executable, false));
    ASSERT_TRUE(wait_for_backend_publisher());
    backend_command.reset();
    for (const uint8_t command : {
        SafetyCommand::Request::PAUSE,
        SafetyCommand::Request::RESUME,
        SafetyCommand::Request::FAULT_RESET,
        SafetyCommand::Request::EMERGENCY_STOP})
    {
      const SafetyReply reply = callSafetyCommand(command, "contract-regression");
      ASSERT_TRUE(reply.completed) << static_cast<int>(command);
      EXPECT_FALSE(reply.accepted) << static_cast<int>(command);
      EXPECT_EQ(reply.message, kSafetyBackendMissing) << static_cast<int>(command);
    }
    for (const uint8_t command : {uint8_t{0}, uint8_t{5}, uint8_t{255}}) {
      const SafetyReply reply = callSafetyCommand(command, "contract-regression");
      ASSERT_TRUE(reply.completed) << static_cast<int>(command);
      EXPECT_FALSE(reply.accepted) << static_cast<int>(command);
      EXPECT_EQ(reply.message, "unknown safety command") << static_cast<int>(command);
    }

    using Trigger = std_srvs::srv::Trigger;
    auto trigger = node_->create_client<Trigger>("/x1/safety/emergency_stop");
    ASSERT_TRUE(trigger->wait_for_service(std::chrono::seconds(5)));
    auto future = trigger->async_send_request(std::make_shared<Trigger::Request>());
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
    const auto trigger_response = future.get();
    EXPECT_FALSE(trigger_response->success);
    EXPECT_EQ(trigger_response->message, kSafetyBackendMissing);
    EXPECT_EQ(backend_command, nullptr);
  }

  {
    const auto gone = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (backend->get_publisher_count() != 0 && std::chrono::steady_clock::now() < gone) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // simulate=true 只发布字符串，不打开控制卡。用来确认参数确实被读到。
    SafetyServerProcess server;
    ASSERT_TRUE(server.start(executable, true));
    ASSERT_TRUE(wait_for_backend_publisher());
    backend_command.reset();
    const SafetyReply reply = callSafetyCommand(
      SafetyCommand::Request::EMERGENCY_STOP, "contract-accept");
    ASSERT_TRUE(reply.completed);
    EXPECT_TRUE(reply.accepted);
    EXPECT_EQ(reply.message, "accepted by explicit fake safety backend");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!backend_command && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_NE(backend_command, nullptr);
    EXPECT_EQ(backend_command->data, "4:contract-accept");
  }
}
