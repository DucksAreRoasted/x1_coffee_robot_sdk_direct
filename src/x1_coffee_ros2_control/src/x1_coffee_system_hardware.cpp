#include "x1_coffee_ros2_control/x1_coffee_system_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "x1_coffee_hardware_core/coffee_axis_model.hpp"
#include "x1_coffee_hardware_core/pending_gtn_transport.hpp"
#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"

namespace x1_coffee_ros2_control
{
namespace
{
using hardware_interface::CallbackReturn;
constexpr auto kPosition = hardware_interface::HW_IF_POSITION;

void debugf(const char * text)
{
  if (std::getenv("X1_ECAT_DEBUG") == nullptr) {
    return;
  }
  std::fprintf(stderr, "[调试] %s\n", text);
  std::fflush(stderr);
}
constexpr auto kVelocity = hardware_interface::HW_IF_VELOCITY;
constexpr std::array<const char *, 14> kDiagnosticInterfaces = {
  "encoder_pulse_raw", "status_bits", "current_permille", "torque_permille",
  "enabled", "moving", "alarm", "follow_error", "emergency_input_active",
  "encoder_valid", "position_valid", "velocity_valid", "current_valid", "torque_valid"};

bool hasInterface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const auto & interface) { return interface.name == name; });
}

bool parseDouble(const std::string & text, double * value)
{
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseInteger(const std::string & text, int minimum, int maximum, int * value)
{
  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(text, &consumed);
    if (consumed != text.size() || parsed < minimum || parsed > maximum) {
      return false;
    }
    *value = static_cast<int>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseBool(const std::string & text, bool * value)
{
  if (text == "true" || text == "True" || text == "TRUE" || text == "1") {
    *value = true;
    return true;
  }
  if (text == "false" || text == "False" || text == "FALSE" || text == "0") {
    *value = false;
    return true;
  }
  return false;
}
}  // namespace

CallbackReturn X1CoffeeSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  debugf("on_init enter");
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    debugf("on_init base failed");
    return CallbackReturn::ERROR;
  }

  std::vector<std::string> names;
  names.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    names.push_back(joint.name);
  }
  std::string error;
  if (!x1_coffee_hardware_core::validateCoffeeJointSet(names, &error)) {
    RCLCPP_ERROR(rclcpp::get_logger("X1CoffeeSystemHardware"), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }

  const auto profile_parameter = info_.hardware_parameters.find("hardware_profile");
  const std::string profile_name = profile_parameter == info_.hardware_parameters.end() ?
    std::string(x1_coffee_hardware_core::defaultHardwareLayout().name) :
    profile_parameter->second;
  const auto profile = x1_coffee_hardware_core::parseHardwareProfile(profile_name);
  if (!profile) {
    RCLCPP_ERROR(
      rclcpp::get_logger("X1CoffeeSystemHardware"),
      "Invalid hardware_profile '%s'; expected coffee_15_no_slide only",
      profile_name.c_str());
    return CallbackReturn::ERROR;
  }
  layout_ = &x1_coffee_hardware_core::hardwareLayout(*profile);

  for (std::size_t declared = 0; declared < info_.joints.size(); ++declared) {
    const auto & joint = info_.joints[declared];
    const auto canonical = *x1_coffee_hardware_core::canonicalIndexForJoint(joint.name);
    const bool physical = x1_coffee_hardware_core::isPhysicalJoint(*layout_, canonical);
    const bool command_interfaces_valid = physical ?
      (joint.command_interfaces.size() == 1 &&
      joint.command_interfaces.front().name == kPosition) :
      joint.command_interfaces.empty();
    if (!command_interfaces_valid || !hasInterface(joint.state_interfaces, kPosition) ||
      !hasInterface(joint.state_interfaces, kVelocity))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("X1CoffeeSystemHardware"),
        "Joint '%s' interface mismatch for hardware_profile=%s (physical=%s)",
        joint.name.c_str(), std::string(layout_->name).c_str(), physical ? "true" : "false");
      return CallbackReturn::ERROR;
    }
    declared_to_canonical_[declared] = canonical;
  }

  if (!parseLimits()) {
    return CallbackReturn::ERROR;
  }
  if (!parseRuntimeParameters()) {
    return CallbackReturn::ERROR;
  }
  debugf("on_init before createTransport");
  guard_ = std::make_unique<x1_coffee_hardware_core::CommandGuard>(limits_);
  if (!createTransport()) {
    debugf("on_init createTransport failed");
    return CallbackReturn::ERROR;
  }
  debugf("on_init done");
  return CallbackReturn::SUCCESS;
}

X1CoffeeSystemHardware::~X1CoffeeSystemHardware()
{
  if (motion_service_thread_.joinable()) {
    motion_service_thread_.join();
  }
  if (motion_executor_) {
    motion_executor_->cancel();
  }
  if (motion_spin_thread_.joinable()) {
    motion_spin_thread_.join();
  }
}

void X1CoffeeSystemHardware::scheduleMotionService()
{
  if (motion_service_thread_.joinable() || motion_node_) {
    return;
  }
  // controller_manager 的构造函数还没返回。这时再创建节点，FastDDS 会在
  // 未完成的 participant 上加锁并段错误。等构造结束再提供 /x1/start_motion。
  motion_service_thread_ = std::thread([this]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!rclcpp::ok()) {
      debugf("startMotionService skipped, rclcpp stopped");
      return;
    }
    if (!startMotionService()) {
      debugf("startMotionService failed");
    }
  });
}

bool X1CoffeeSystemHardware::startMotionService()
{
  // Humble 的 SystemInterface 没有节点。服务必须留在本进程里，开机程序才能在
  // 轨迹控制器 active 之后调用 startMotion。
  debugf("startMotionService before Node");
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("X1CoffeeSystemHardware"),
      "rclcpp 尚未初始化，无法提供 /x1/start_motion");
    return false;
  }
  motion_node_ = std::make_shared<rclcpp::Node>(
    "x1_coffee_system_hardware", rclcpp::NodeOptions().use_global_arguments(false));
  debugf("startMotionService node created");
  // 自动采样的异常出口，服务成功仍须由调用方检查实际运动是否停止。
  cancel_pvt_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/cancel_pvt",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      auto * direct = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(transport_.get());
      if (direct == nullptr) {
        response->success = false;
        response->message = "没有直连 PVT 后端";
        return;
      }
      const auto result = direct->cancelPvtHold();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  // 独立入口，不复用旧的Nm标定/保存服务；加载不会启用扭矩控制。
  const std::array<std::string, 5> current_services{
    "/x1/current_gravity_load", "/x1/current_gravity_enter", "/x1/current_gravity_status",
    "/x1/current_gravity_exit", "/x1/current_gravity_hold_status"};
  for (std::size_t i = 0; i < current_services.size(); ++i) {
    current_gravity_services_[i] = motion_node_->create_service<std_srvs::srv::Trigger>(
      current_services[i], [this, i](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto * direct = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(transport_.get());
        if (direct == nullptr) {
          response->success = false;
          response->message = "当前后端不支持电流补偿";
          return;
        }
        x1_coffee_hardware_core::Result result;
        if (i == 0) {
          const char * path = std::getenv("X1_CURRENT_GRAVITY_FILE");
          if (path == nullptr || path[0] == '\0') {
            response->success = false;
            response->message = "下位机未设置X1_CURRENT_GRAVITY_FILE参数文件路径";
            return;
          }
          result = direct->loadCurrentGravity(path);
        } else if (i == 1) {result = direct->enterCurrentGravity();
        } else if (i == 2) {result = direct->currentGravityStatus();
        } else if (i == 3) {result = direct->exitMitHold();
        } else {result = direct->currentGravityHoldStatus();}
        response->success = static_cast<bool>(result);
        response->message = result.detail;
      });
  }
  start_motion_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/start_motion",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      allowMotionFromMeasurement(response);
    });
  arm_tracked_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/arm_tracked_motion",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      allowMotionFromMeasurement(response);
    });
  zero_torque_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/zero_torque",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!transport_) {
        response->success = false;
        response->message = "传输层尚未创建";
        return;
      }
      const auto result = transport_->requestZeroTorque();
      response->success = static_cast<bool>(result);
      response->message = result.detail.empty() ? std::string("零力矩请求已处理") : result.detail;
    });
  brake_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/brake_hold",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!transport_) {
        response->success = false;
        response->message = "传输层尚未创建";
        return;
      }
      const auto result = transport_->requestEngageBrakes();
      response->success = static_cast<bool>(result);
      response->message = result.detail.empty() ? std::string("已请求关断") : result.detail;
    });
  mit_hold_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/mit_hold",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      auto * transport = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      if (transport == nullptr) {
        response->success = false;
        response->message = "当前传输层没有 MIT";
        return;
      }
      const auto result = transport->enterMitHold();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  mit_exit_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/mit_exit",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      auto * transport = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      if (transport == nullptr) {
        response->success = false;
        response->message = "当前传输层没有 MIT";
        return;
      }
      const auto result = transport->exitMitHold();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  gravity_calib_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/right_arm_gravity_calib",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      auto * transport = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      if (transport == nullptr) {
        response->success = false;
        response->message = "当前传输层不能做右臂重力标定";
        return;
      }
      const auto result = transport->startRightArmGravityCalibration();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  gravity_reset_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/right_arm_gravity_reset",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      auto * transport = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      if (transport == nullptr) {
        response->success = false;
        response->message = "当前传输层不能恢复右臂重力初值";
        return;
      }
      const auto result = transport->resetRightArmGravityToUrdf();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  gravity_save_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/right_arm_gravity_save",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      auto * transport = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      if (transport == nullptr) {
        response->success = false;
        response->message = "当前传输层不能保存右臂重力";
        return;
      }
      const auto result = transport->saveRightArmGravity();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  zero_gravity_service_ = motion_node_->create_service<std_srvs::srv::Trigger>(
    "/x1/zero_gravity",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      auto * transport = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      if (transport == nullptr) {
        response->success = false;
        response->message = "当前传输层不能做零重力";
        return;
      }
      const auto result = transport->enterZeroGravity();
      response->success = static_cast<bool>(result);
      response->message = result.detail;
    });
  pvt_result_pub_ = motion_node_->create_publisher<std_msgs::msg::String>("/x1/pvt_result", 10);
  pvt_move_sub_ = motion_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/x1/pvt_move", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr message)
    {
      std_msgs::msg::String result_message;
      const auto fail = [&](const std::string & detail) {
          result_message.data = detail;
          if (pvt_result_pub_) {
            pvt_result_pub_->publish(result_message);
          }
        };
      constexpr std::size_t kAxisCount = x1_coffee_hardware_core::kCoffeeAxisCount;
      constexpr std::size_t kMaxWaypoints = 500;
      if (!message || message->data.size() < 3 + kAxisCount) {
        fail("0 fail PVT 数据至少要有序号、秒、路点数和 15 个关节位置");
        return;
      }
      const double sequence = message->data[0];
      const double duration_sec = message->data[1];
      const double count_value = message->data[2];
      const auto waypoint_count = static_cast<std::size_t>(std::llround(count_value));
      const std::string prefix = std::to_string(sequence) + " ";
      const bool timed = message->data.size() == 3 + waypoint_count * (1 + kAxisCount);
      const bool plain = message->data.size() == 3 + waypoint_count * kAxisCount;
      if (waypoint_count < 1 || waypoint_count > kMaxWaypoints ||
        std::abs(count_value - static_cast<double>(waypoint_count)) > 1.0e-6 ||
        (!timed && !plain))
      {
        fail(prefix + "fail PVT 路点数量必须在 1 到 500，且带上每段时间或共用总时间");
        return;
      }
      std::vector<std::array<double, kAxisCount>> waypoints(waypoint_count);
      std::vector<double> segment_sec(waypoint_count, 0.0);
      if (timed) {
        for (std::size_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index) {
          const std::size_t base = 3 + waypoint_index * (1 + kAxisCount);
          segment_sec[waypoint_index] = message->data[base];
          for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
            waypoints[waypoint_index][joint_index] = message->data[base + 1 + joint_index];
          }
        }
      } else if (!(duration_sec > 0.0)) {
        fail(prefix + "fail 位置段时间必须为正数");
        return;
      } else {
        const double slice = duration_sec / static_cast<double>(waypoint_count);
        for (std::size_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index) {
          segment_sec[waypoint_index] = slice;
          for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
            waypoints[waypoint_index][joint_index] =
              message->data[3 + waypoint_index * kAxisCount + joint_index];
          }
        }
      }
      if (!transport_) {
        fail(prefix + "fail 传输层尚未创建");
        return;
      }
      auto * coordinated = dynamic_cast<x1_coffee_hardware_core::SoemEthercatTransport *>(
        transport_.get());
      const auto result = coordinated != nullptr ?
        coordinated->requestMovePvtSegments(waypoints, segment_sec) :
        transport_->requestMovePvt(waypoints, duration_sec);
      fail(prefix + (result ? "ok" : "fail " + result.detail));
    });
  motion_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  motion_executor_->add_node(motion_node_);
  motion_spin_thread_ = std::thread([this]() {motion_executor_->spin();});
  RCLCPP_INFO(
    rclcpp::get_logger("X1CoffeeSystemHardware"),
    "✓ 运动服务已就绪。保持期间不下发位置");
  debugf("startMotionService ready");
  return true;
}

void X1CoffeeSystemHardware::allowMotionFromMeasurement(
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!transport_) {
    response->success = false;
    response->message = "传输层尚未创建";
    return;
  }
  // The next write copies the live encoder into the command baseline before
  // any PT point is streamed, so arming motion cannot jump back to an older target.
  snap_commands_to_measurement_.store(true);
  const auto result = transport_->startMotion();
  response->success = static_cast<bool>(result);
  response->message = result ? std::string("已从当前关节角允许运动") : result.detail;
}

bool X1CoffeeSystemHardware::parseRuntimeParameters()
{
  const auto read_integer = [this](const char * key, int minimum, int maximum, int * target) {
      const auto iterator = info_.hardware_parameters.find(key);
      if (iterator == info_.hardware_parameters.end()) {
        return true;
      }
      if (!parseInteger(iterator->second, minimum, maximum, target)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("X1CoffeeSystemHardware"),
          "Invalid hardware parameter %s='%s'", key, iterator->second.c_str());
        return false;
      }
      return true;
    };
  const auto read_bool = [this](const char * key, bool * target) {
      const auto iterator = info_.hardware_parameters.find(key);
      if (iterator == info_.hardware_parameters.end()) {
        return true;
      }
      if (!parseBool(iterator->second, target)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("X1CoffeeSystemHardware"),
          "Invalid hardware parameter %s='%s'", key, iterator->second.c_str());
        return false;
      }
      return true;
    };
  return read_integer("state_timeout_ms", 1, 60000, &state_timeout_ms_) &&
         read_integer("command_timeout_ms", 1, 60000, &command_timeout_ms_) &&
         read_integer("motion_startup_grace_ms", 0, 60000, &motion_startup_grace_ms_) &&
         read_bool("motion_enabled", &motion_enabled_) &&
         read_bool("stop_on_axis_alarm", &stop_on_axis_alarm_) &&
         read_bool("stop_on_emergency_input", &stop_on_emergency_input_);
}

bool X1CoffeeSystemHardware::createTransport()
{
  const auto iterator = info_.hardware_parameters.find("transport");
  debugf("createTransport");
  const std::string backend = iterator == info_.hardware_parameters.end() ?
    "ethercat" : iterator->second;
  // 旧参数名仍指这条网口主站，避免启动时再去打开固高库。
  if (backend == "ethercat" || backend == "gtn_ethercat") {
    transport_ = std::make_unique<x1_coffee_hardware_core::SoemEthercatTransport>();
    return true;
  }
  if (backend == "pending") {
    transport_ = std::make_unique<x1_coffee_hardware_core::PendingGtnTransport>();
    return true;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger("X1CoffeeSystemHardware"),
    "Unsupported transport backend '%s'; expected ethercat or pending", backend.c_str());
  return false;
}

bool X1CoffeeSystemHardware::parseLimits()
{
  for (std::size_t declared = 0; declared < info_.joints.size(); ++declared) {
    const auto canonical = declared_to_canonical_[declared];
    if (!x1_coffee_hardware_core::isPhysicalJoint(*layout_, canonical)) {
      continue;
    }
    const auto & command = info_.joints[declared].command_interfaces.front();
    auto & limit = limits_[canonical];
    if (!command.min.empty() && !parseDouble(command.min, &limit.minimum)) {
      RCLCPP_ERROR(rclcpp::get_logger("X1CoffeeSystemHardware"), "Invalid min for %s", info_.joints[declared].name.c_str());
      return false;
    }
    if (!command.max.empty() && !parseDouble(command.max, &limit.maximum)) {
      RCLCPP_ERROR(rclcpp::get_logger("X1CoffeeSystemHardware"), "Invalid max for %s", info_.joints[declared].name.c_str());
      return false;
    }
    const auto velocity_key = std::string("max_velocity.") + info_.joints[declared].name;
    const auto velocity = info_.hardware_parameters.find(velocity_key);
    if (velocity != info_.hardware_parameters.end() &&
      !parseDouble(velocity->second, &limit.maximum_velocity))
    {
      RCLCPP_ERROR(rclcpp::get_logger("X1CoffeeSystemHardware"), "Invalid %s", velocity_key.c_str());
      return false;
    }
    if (limit.minimum > limit.maximum || limit.maximum_velocity <= 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("X1CoffeeSystemHardware"), "Invalid limits for %s", info_.joints[declared].name.c_str());
      return false;
    }
  }
  return true;
}

CallbackReturn X1CoffeeSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  debugf("on_configure enter");
  const auto result = transport_->configure(info_.hardware_parameters);
  if (!result) {
    logTransportError("configure", result);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn X1CoffeeSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  debugf("on_activate enter");
  // 激活失败如果从这里返回，controller_manager 会抛异常，进程以退出码 -6 结束，
  // 进程内的上电重试就没机会再跑。初始反馈无效时先断开，再整段上电。
  x1_coffee_hardware_core::HardwareSnapshot snapshot;
  for (;;) {
    auto result = transport_->activate();
    if (!result) {
      logTransportError("activate", result);
      return CallbackReturn::ERROR;
    }
    result = transport_->read(&snapshot);
    bool sample_ok = static_cast<bool>(result);
    if (sample_ok) {
      for (std::size_t i = 0; i < positions_.size(); ++i) {
        if (!snapshot.axes[i].position_valid || !snapshot.axes[i].velocity_valid ||
          !std::isfinite(snapshot.axes[i].position) || !std::isfinite(snapshot.axes[i].velocity))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger("X1CoffeeSystemHardware"), "✗ 轴 %zu 的初始状态无效", i + 1);
          sample_ok = false;
          break;
        }
      }
    } else {
      logTransportError("initial read", result);
    }
    if (sample_ok) {
      break;
    }
    transport_->stop(false);
    transport_->deactivate();
    RCLCPP_ERROR(
      rclcpp::get_logger("X1CoffeeSystemHardware"), "初始反馈无效，断开后重新上电");
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }
  for (std::size_t i = 0; i < positions_.size(); ++i) {
    positions_[i] = snapshot.axes[i].position;
    velocities_[i] = snapshot.axes[i].velocity;
    commands_[i] = positions_[i];
    last_sent_commands_[i] = positions_[i];
  }
  has_read_sample_ = false;
  has_write_sample_ = false;
  read_call_count_ = 0;
  write_call_count_ = 0;
  last_read_call_at_ = std::chrono::steady_clock::time_point{};
  last_write_call_at_ = std::chrono::steady_clock::time_point{};
  cycle_report_started_at_ = std::chrono::steady_clock::now();
  cycle_report_read_start_ = 0;
  cycle_report_write_start_ = 0;
  last_read_warning_at_ = std::chrono::steady_clock::time_point{};
  last_write_warning_at_ = std::chrono::steady_clock::time_point{};
  active_ = true;
  motion_started_ = false;
  activation_completed_at_ = std::chrono::steady_clock::now();
  scheduleMotionService();
  debugf("on_activate done");
  return CallbackReturn::SUCCESS;
}

CallbackReturn X1CoffeeSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  debugf("on_deactivate enter");
  active_ = false;
  motion_started_ = false;
  has_read_sample_ = false;
  has_write_sample_ = false;
  read_call_count_ = 0;
  write_call_count_ = 0;
  last_read_call_at_ = std::chrono::steady_clock::time_point{};
  last_write_call_at_ = std::chrono::steady_clock::time_point{};
  cycle_report_started_at_ = std::chrono::steady_clock::time_point{};
  cycle_report_read_start_ = 0;
  cycle_report_write_start_ = 0;
  last_read_warning_at_ = std::chrono::steady_clock::time_point{};
  last_write_warning_at_ = std::chrono::steady_clock::time_point{};
  activation_completed_at_ = std::chrono::steady_clock::time_point{};
  const auto stop = transport_->stop(false);
  const auto deactivate = transport_->deactivate();
  if (!stop) {
    logTransportError("stop", stop);
  }
  if (!deactivate) {
    logTransportError("deactivate", deactivate);
  }
  return (stop && deactivate) ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn X1CoffeeSystemHardware::on_error(const rclcpp_lifecycle::State &)
{
  active_ = false;
  motion_started_ = false;
  has_read_sample_ = false;
  has_write_sample_ = false;
  read_call_count_ = 0;
  write_call_count_ = 0;
  last_read_call_at_ = std::chrono::steady_clock::time_point{};
  last_write_call_at_ = std::chrono::steady_clock::time_point{};
  cycle_report_started_at_ = std::chrono::steady_clock::time_point{};
  cycle_report_read_start_ = 0;
  cycle_report_write_start_ = 0;
  last_read_warning_at_ = std::chrono::steady_clock::time_point{};
  last_write_warning_at_ = std::chrono::steady_clock::time_point{};
  activation_completed_at_ = std::chrono::steady_clock::time_point{};
  if (transport_) {
    // 总线已经关掉时不再补一次停止。
    if (!transport_->controllerClosed()) {
      const auto stop = transport_->stop(true);
      if (!stop) {
        logTransportError("error stop", stop);
      }
    }
    const auto deactivate = transport_->deactivate();
    if (!deactivate) {
      logTransportError("error deactivate", deactivate);
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn X1CoffeeSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  std::fill(positions_.begin(), positions_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(velocities_.begin(), velocities_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(commands_.begin(), commands_.end(), std::numeric_limits<double>::quiet_NaN());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> X1CoffeeSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * (2 + kDiagnosticInterfaces.size()));
  for (std::size_t declared = 0; declared < info_.joints.size(); ++declared) {
    const auto canonical = declared_to_canonical_[declared];
    interfaces.emplace_back(info_.joints[declared].name, kPosition, &positions_[canonical]);
    interfaces.emplace_back(info_.joints[declared].name, kVelocity, &velocities_[canonical]);
    if (!x1_coffee_hardware_core::isPhysicalJoint(*layout_, canonical)) {
      continue;
    }
    for (std::size_t field = 0; field < kDiagnosticInterfaces.size(); ++field) {
      interfaces.emplace_back(
        info_.joints[declared].name, kDiagnosticInterfaces[field],
        &diagnostic_states_[canonical][field]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> X1CoffeeSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size());
  for (std::size_t declared = 0; declared < info_.joints.size(); ++declared) {
    const auto canonical = declared_to_canonical_[declared];
    if (!x1_coffee_hardware_core::isPhysicalJoint(*layout_, canonical)) {
      continue;
    }
    interfaces.emplace_back(info_.joints[declared].name, kPosition, &commands_[canonical]);
  }
  return interfaces;
}

hardware_interface::return_type X1CoffeeSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  if (has_read_sample_) {
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_at_).count();
    // The periodic cycle-rate summary reports scheduler behaviour.  Avoid
    // ROS logging from each delayed real-time update.
    (void)elapsed_ms;
  }
  x1_coffee_hardware_core::HardwareSnapshot snapshot;
  const auto result = transport_->read(&snapshot);
  if (transport_->consumeBlockingCardWork()) {
    // 零力矩和抱闸在这次 read 里同步切模式，SDO 可能超过 command_timeout_ms。
    // 刷新写时间，避免紧接着的 write 把这次耗时当成指令超时再 AxisOff，抱闸会重新锁上。
    last_write_at_ = std::chrono::steady_clock::now();
    has_write_sample_ = true;
  }
  if (!result) {
    logTransportError("read", result);
    const bool emergency = result.error == x1_coffee_hardware_core::CoreError::kEmergencyStop ||
      result.error == x1_coffee_hardware_core::CoreError::kAxisAlarm ||
      result.error == x1_coffee_hardware_core::CoreError::kStateTimeout;
    const auto stopped = transport_->stop(emergency);
    if (!stopped) {
      logTransportError("fault stop", stopped);
    }
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t i = 0; i < positions_.size(); ++i) {
    if (!snapshot.axes[i].position_valid || !snapshot.axes[i].velocity_valid ||
      !std::isfinite(snapshot.axes[i].position) || !std::isfinite(snapshot.axes[i].velocity))
    {
      RCLCPP_ERROR(rclcpp::get_logger("X1CoffeeSystemHardware"), "✗ 轴 %zu 的反馈失效", i + 1);
      transport_->stop(stop_on_axis_alarm_ || stop_on_emergency_input_);
      return hardware_interface::return_type::ERROR;
    }
    positions_[i] = snapshot.axes[i].position;
    velocities_[i] = snapshot.axes[i].velocity;
    const auto & axis = snapshot.axes[i];
    diagnostic_states_[i] = {
      static_cast<double>(axis.encoder_pulse_raw), static_cast<double>(axis.status_bits),
      static_cast<double>(axis.current_permille), static_cast<double>(axis.torque_permille),
      axis.enabled ? 1.0 : 0.0, axis.moving ? 1.0 : 0.0, axis.alarm ? 1.0 : 0.0,
      axis.follow_error ? 1.0 : 0.0, axis.emergency_input_active ? 1.0 : 0.0,
      axis.encoder_valid ? 1.0 : 0.0, axis.position_valid ? 1.0 : 0.0,
      axis.velocity_valid ? 1.0 : 0.0, axis.current_valid ? 1.0 : 0.0,
      axis.torque_valid ? 1.0 : 0.0};
  }
  last_read_at_ = now;
  has_read_sample_ = true;
  ++read_call_count_;
  const auto report_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - cycle_report_started_at_).count();
  if (report_elapsed >= 10000) {
    const double seconds = static_cast<double>(report_elapsed) / 1000.0;
    const double read_hz = static_cast<double>(read_call_count_ - cycle_report_read_start_) / seconds;
    const double write_hz = static_cast<double>(write_call_count_ - cycle_report_write_start_) / seconds;
    if (motion_enabled_) {
      RCLCPP_INFO(
        rclcpp::get_logger("X1CoffeeSystemHardware"),
        "● 控制周期：读 %.0f Hz，写 %.0f Hz", read_hz, write_hz);
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("X1CoffeeSystemHardware"),
        "● 控制周期：读 %.0f Hz，写 %.0f Hz（保持当前位置）", read_hz, write_hz);
    }
    cycle_report_started_at_ = now;
    cycle_report_read_start_ = read_call_count_;
    cycle_report_write_start_ = write_call_count_;
  }
  last_read_call_at_ = now;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type X1CoffeeSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }
  // 只上电时控制器仍会周期 write。这里直接返回，不把保持当成轨迹。
  if (!motion_enabled_) {
    return hardware_interface::return_type::OK;
  }
  const auto write_call_at = std::chrono::steady_clock::now();
  ++write_call_count_;
  if (last_write_call_at_ != std::chrono::steady_clock::time_point{}) {
    const auto write_call_gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      write_call_at - last_write_call_at_).count();
    // In read-only mode, do not emit a warning per delayed callback.  During
    // enabled motion, the watchdog below remains authoritative and safe.
    (void)write_call_gap_ms;
  }
  last_write_call_at_ = write_call_at;
  if (snap_commands_to_measurement_.exchange(false)) {
    commands_ = positions_;
    last_sent_commands_ = positions_;
  }
  if (motion_enabled_ && !motion_started_) {
    bool command_changed = false;
    for (std::size_t i = 0; i < commands_.size(); ++i) {
      if (std::abs(commands_[i] - last_sent_commands_[i]) > 1.0e-9) {
        command_changed = true;
        break;
      }
    }
    if (!command_changed) {
      return hardware_interface::return_type::OK;
    }
    motion_started_ = true;
  }
  x1_coffee_hardware_core::PositionCommandFrame frame;
  frame.positions = commands_;
  frame.generated_at = std::chrono::steady_clock::now();
  if (has_write_sample_) {
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      frame.generated_at - last_write_at_).count();
    if (elapsed_ms > command_timeout_ms_) {
      const auto startup_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        frame.generated_at - activation_completed_at_).count();
      const bool within_startup_grace = motion_enabled_ &&
        startup_elapsed_ms >= 0 && startup_elapsed_ms <= motion_startup_grace_ms_;
      if (motion_enabled_ && !within_startup_grace) {
        RCLCPP_ERROR(
          rclcpp::get_logger("X1CoffeeSystemHardware"),
          "Hardware write watchdog timeout during enabled motion: "
          "elapsed_ms=%ld threshold_ms=%d",
          static_cast<long>(elapsed_ms), command_timeout_ms_);
        transport_->stop(true);
        return hardware_interface::return_type::ERROR;
      }
      if (within_startup_grace) {
        RCLCPP_WARN(
          rclcpp::get_logger("X1CoffeeSystemHardware"),
          "Hardware write scheduling gap during motion startup grace: "
          "elapsed_ms=%ld threshold_ms=%d grace_elapsed_ms=%ld grace_ms=%d; continuing",
          static_cast<long>(elapsed_ms), command_timeout_ms_,
          static_cast<long>(startup_elapsed_ms), motion_startup_grace_ms_);
      }
    }
  }
  guard_->clampVelocity(frame, last_sent_commands_, period.seconds());
  const auto validation = guard_->validate(frame, last_sent_commands_, period.seconds());
  if (!validation) {
    // 超限位或非有限值才整帧保持。超速已经在上面按本周期上限截断。
    const auto now = std::chrono::steady_clock::now();
    const bool log_hold = last_guard_hold_log_at_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_guard_hold_log_at_).count() >= 1000;
    if (log_hold) {
      last_guard_hold_log_at_ = now;
      RCLCPP_WARN(
        rclcpp::get_logger("X1CoffeeSystemHardware"),
        "command guard holding last position, bus left up (%u): %s",
        static_cast<unsigned>(validation.error), validation.detail.c_str());
    }
    frame.positions = last_sent_commands_;
  }
  const auto result = transport_->write(frame);
  if (!result) {
    logTransportError("write", result);
    return hardware_interface::return_type::ERROR;
  }
  if (validation) {
    last_sent_commands_ = frame.positions;
  }
  // PT 启动在这次 write 里同步完成，可能超过 command_timeout_ms。
  // 用返回后的时间计时，避免下一周期把启动耗时误判成指令超时并急停。
  last_write_at_ = std::chrono::steady_clock::now();
  has_write_sample_ = true;
  return hardware_interface::return_type::OK;
}

void X1CoffeeSystemHardware::logTransportError(
  const char * operation, const x1_coffee_hardware_core::Result & result)
{
  (void)operation;
  RCLCPP_ERROR(
    rclcpp::get_logger("X1CoffeeSystemHardware"), "✗ 传输失败：%s", result.detail.c_str());
}

}  // namespace x1_coffee_ros2_control

PLUGINLIB_EXPORT_CLASS(
  x1_coffee_ros2_control::X1CoffeeSystemHardware, hardware_interface::SystemInterface)
