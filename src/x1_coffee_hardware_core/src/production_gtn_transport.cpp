#include "x1_coffee_hardware_core/production_gtn_transport.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <limits>
#include <sstream>
#include <thread>

namespace x1_coffee_hardware_core
{
namespace
{

// 《编程手册之基本功能》对 GTN_LoadConfig：文件名不得超过 125 个字，且不在可执行目录时必须给绝对路径。
std::string loadConfigPath(const std::string & path)
{
  if (path.size() <= 125) {
    return path;
  }
  const auto short_path = std::filesystem::path("/tmp/x1_gtn_core1.cfg");
  std::error_code error;
  std::filesystem::copy_file(
    path, short_path, std::filesystem::copy_options::overwrite_existing, error);
  if (error) {
    return path;
  }
  return short_path.string();
}

constexpr std::uint32_t kAlarmMask = 0x02;
constexpr std::uint32_t kFollowErrorMask = 0x10;
constexpr std::uint32_t kEmergencyInputMask = 0x100;
constexpr std::uint32_t kEnabledMask = 0x200;
constexpr std::uint32_t kMovingMask = 0x400;
constexpr short kPtSegmentNormal = 0;
constexpr int kCardZeroTorque = 1;
constexpr int kCardBrake = 2;
constexpr int kCardPvt = 3;
// gxn.h: CMD_ERROR_UNKNOWN (8) means "unsupported command".  The GTN runtime
// used by the existing robot software does not support McEcat axis-table queries.
constexpr short kUnsupportedCommand = 8;
// gxn.h: CMD_LOCK_ERROR. The vendor library's EtherCAT thread holds this lock
// briefly after the bus starts, so one encoder read can fail while the
// controller is otherwise open.
constexpr short kControllerLockBusy = -8;
// The first read runs before the cyclic loop. A short EtherCAT-thread lock
// there used to abort activation. Later reads must not block the controller
// manager; if the lock is still busy, keep the last encoder.
constexpr int kEncoderBusyAttemptsInitial = 8;
constexpr int kEncoderBusyWaitInitialMs = 50;
constexpr int kEncoderBusyAttemptsCyclic = 4;
constexpr int kEncoderBusyWaitCyclicMs = 1;

bool parseBool(const std::string & text, bool * value)
{
  std::string normalized = text;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (normalized == "true" || normalized == "1") {
    *value = true;
    return true;
  }
  if (normalized == "false" || normalized == "0") {
    *value = false;
    return true;
  }
  return false;
}

template<typename Integer>
bool parseInteger(const std::string & text, Integer minimum, Integer maximum, Integer * value)
{
  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(text, &consumed, 10);
    if (consumed != text.size() || parsed < minimum || parsed > maximum) {
      return false;
    }
    *value = static_cast<Integer>(parsed);
    return true;
  } catch (...) {
    return false;
  }
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
  } catch (...) {
    return false;
  }
}

double normalizePulseDelta(double delta, std::int32_t wrap_period)
{
  if (wrap_period <= 0) {
    return delta;
  }
  const double period = static_cast<double>(wrap_period);
  double wrapped = std::fmod(delta, period);
  if (wrapped < 0.0) {
    wrapped += period;
  }
  if (wrapped > period / 2.0) {
    wrapped -= period;
  }
  return wrapped;
}

double nearestEquivalentPulse(double canonical_target, std::int32_t current, std::int32_t wrap_period)
{
  if (wrap_period <= 0) {
    return canonical_target;
  }
  const double normalized_delta = normalizePulseDelta(
    static_cast<double>(current) - canonical_target, wrap_period);
  return static_cast<double>(current) - normalized_delta;
}

const std::string * findParameter(const HardwareParameters & parameters, const std::string & key)
{
  const auto iterator = parameters.find(key);
  return iterator == parameters.end() ? nullptr : &iterator->second;
}

Result requireExistingFile(
  const HardwareParameters & parameters, const std::string & key, std::string * destination)
{
  const std::string * value = findParameter(parameters, key);
  if (value == nullptr || value->empty()) {
    return Result::fail(CoreError::kInvalidConfiguration, "缺少硬件参数: " + key);
  }
  std::error_code error;
  if (!std::filesystem::is_regular_file(*value, error)) {
    return Result::fail(
      CoreError::kInvalidConfiguration, "硬件文件不存在或不是普通文件: " + key + "=" + *value);
  }
  *destination = *value;
  return Result::ok();
}

}  // namespace

ProductionGtnTransport::ProductionGtnTransport() = default;

ProductionGtnTransport::ProductionGtnTransport(std::unique_ptr<GtnNativeApi> native_api)
: native_api_(std::move(native_api))
{
}

ProductionGtnTransport::~ProductionGtnTransport()
{
  deactivate();
}

Result ProductionGtnTransport::configure(const HardwareParameters & parameters)
{
  if (state_ == State::kActive) {
    return Result::fail(CoreError::kInvalidConfiguration, "活动状态下禁止重新配置 GTN 传输层");
  }
  if (state_ == State::kConfigured) {
    deactivate();
  }

  const Result parsed = parseParameters(parameters);
  if (!parsed) {
    return parsed;
  }
  if (!native_api_) {
    std::string detail;
    native_api_ = createDynamicGtnNativeApi(library_path_, &detail);
    if (!native_api_) {
      return Result::fail(CoreError::kTransportUnavailable, detail);
    }
  }
  state_ = State::kConfigured;
  return Result::ok();
}

Result ProductionGtnTransport::parseParameters(const HardwareParameters & parameters)
{
  HardwareProfile profile = HardwareProfile::kCoffee15NoSlide;
  if (const std::string * value = findParameter(parameters, "hardware_profile")) {
    const auto parsed_profile = parseHardwareProfile(*value);
    if (!parsed_profile) {
      return Result::fail(
        CoreError::kInvalidConfiguration,
        "hardware_profile 必须为 coffee_15_no_slide");
    }
    profile = *parsed_profile;
  }
  layout_ = &hardwareLayout(profile);
  all_axes_mask_ = static_cast<std::int32_t>(layout_->all_axes_mask);

  const std::string * axis_count = findParameter(parameters, "axis_count");
  int parsed_axis_count = 0;
  if (axis_count == nullptr || !parseInteger(*axis_count, 1, 64, &parsed_axis_count) ||
    parsed_axis_count != static_cast<int>(layout_->physical_axis_count))
  {
    std::ostringstream stream;
    stream << "axis_count 必须与 hardware_profile=" << layout_->name <<
      " 一致，expected=" << layout_->physical_axis_count;
    return Result::fail(CoreError::kInvalidConfiguration, stream.str());
  }

  if (!native_api_) {
    const Result library = requireExistingFile(parameters, "gtn_library_path", &library_path_);
    if (!library) {
      return library;
    }
  }
  const Result gtn_config = requireExistingFile(parameters, "gtn_config_path", &gtn_config_path_);
  if (!gtn_config) {
    return gtn_config;
  }
  const Result ecat_config = requireExistingFile(parameters, "ecat_config_path", &ecat_config_path_);
  if (!ecat_config) {
    return ecat_config;
  }

  if (const std::string * value = findParameter(parameters, "core")) {
    if (!parseInteger(*value, static_cast<short>(1), static_cast<short>(2), &core_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "core 必须为 1 或 2");
    }
  }
  if (const std::string * value = findParameter(parameters, "ecat_ready_retries")) {
    if (!parseInteger(*value, 1, 10000, &ready_retries_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "ecat_ready_retries 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "ecat_ready_interval_ms")) {
    if (!parseInteger(*value, 1, 10000, &ready_interval_ms_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "ecat_ready_interval_ms 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "ecat_step_delay_ms")) {
    if (!parseInteger(*value, 0, 10000, &ecat_step_delay_ms_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "ecat_step_delay_ms 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "ecat_start_delay_ms")) {
    if (!parseInteger(*value, 0, 10000, &ecat_start_delay_ms_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "ecat_start_delay_ms 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "ecat_op_wait_ms")) {
    if (!parseInteger(*value, 0, 60000, &ecat_op_wait_ms_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "ecat_op_wait_ms 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "ecat_init_attempts")) {
    if (!parseInteger(*value, 1, 5, &ecat_init_attempts_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "ecat_init_attempts 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "motion_enabled")) {
    if (!parseBool(*value, &motion_enabled_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "motion_enabled 必须为 true/false 或 1/0");
    }
  }
  if (const std::string * value = findParameter(parameters, "drives_enabled")) {
    if (!parseBool(*value, &drives_enabled_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "drives_enabled 必须为 true/false 或 1/0");
    }
  }
  if (const std::string * value = findParameter(parameters, "pt_cycle_ms")) {
    if (!parseInteger(*value, 2, 1000, &pt_cycle_ms_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "pt_cycle_ms 必须在 2 到 1000 之间");
    }
  }
  if (const std::string * value = findParameter(parameters, "pt_prebuffer_points")) {
    if (!parseInteger(*value, 2, 512, &pt_prebuffer_points_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "pt_prebuffer_points 必须在 2 到 512 之间");
    }
  }
  if (const std::string * value = findParameter(parameters, "read_current_torque")) {
    if (!parseBool(*value, &read_current_torque_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "read_current_torque 必须为 true/false 或 1/0");
    }
  }
  if (const std::string * value = findParameter(parameters, "diagnostic_poll_div")) {
    if (!parseInteger(*value, 1, 1000, &diagnostic_poll_div_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "diagnostic_poll_div 无效");
    }
  }
  if (const std::string * value = findParameter(parameters, "timing_log_div")) {
    if (!parseInteger(*value, 0, 1000000, &timing_log_div_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "timing_log_div 无效");
    }
  }

  const auto & axes = coffeeAxisDescriptors();
  for (std::size_t index = 0; index < axes.size(); ++index) {
    calibration_[index] = AxisCalibration{};
    if (!isPhysicalJoint(*layout_, index)) {
      continue;
    }
    const std::string joint(axes[index].joint_name);
    const std::string scale_key = "pulses_per_unit." + joint;
    const std::string offset_key = "zero_offset_pulse." + joint;
    const std::string model_zero_key = "model_zero." + joint;
    const std::string wrap_key = "wrap_period_pulse." + joint;
    const std::string * scale = findParameter(parameters, scale_key);
    const std::string * offset = findParameter(parameters, offset_key);
    if (scale == nullptr || !parseDouble(*scale, &calibration_[index].pulses_per_unit) ||
      std::fabs(calibration_[index].pulses_per_unit) < 1.0)
    {
      return Result::fail(CoreError::kInvalidConfiguration, "缺少或无效的标定参数: " + scale_key);
    }
    if (offset == nullptr || !parseDouble(*offset, &calibration_[index].zero_offset_pulse)) {
      return Result::fail(CoreError::kInvalidConfiguration, "缺少或无效的标定参数: " + offset_key);
    }
    calibration_[index].model_zero = 0.0;
    if (const std::string * model_zero = findParameter(parameters, model_zero_key)) {
      if (!parseDouble(*model_zero, &calibration_[index].model_zero)) {
        return Result::fail(CoreError::kInvalidConfiguration, "无效的标定参数: " + model_zero_key);
      }
    }
    const std::string * wrap = findParameter(parameters, wrap_key);
    if (wrap == nullptr || !parseInteger(
        *wrap, static_cast<std::int32_t>(0), std::numeric_limits<std::int32_t>::max(),
        &calibration_[index].wrap_period_pulse))
    {
      return Result::fail(CoreError::kInvalidConfiguration, "缺少或无效的标定参数: " + wrap_key);
    }
  }
  return Result::ok();
}

Result ProductionGtnTransport::activate()
{
  if (state_ != State::kConfigured || !native_api_) {
    return Result::fail(CoreError::kInvalidConfiguration, "GTN 传输层尚未完成配置");
  }
  // 允许运动只来自之后的 startMotion。上电本身、包括 motion_enabled，都不武装 PT。
  motion_armed_.store(false);
  zero_torque_ = false;
  blocking_card_work_.store(false);
  pt_fifo_stall_ = 0;
  const Result initialized = initializeController();
  if (!initialized) {
    return failActivation(initialized);
  }

  HardwareSnapshot initial_snapshot;
  state_ = State::kActive;
  const Result initial_read = read(&initial_snapshot);
  if (!initial_read) {
    return failActivation(initial_read);
  }
  // 运动仍等到真正的位置指令。上电使能只在 drives_enabled 时做 GTN_AxisOn，不进 PT。
  stop_latched_ = false;
  emergency_latched_ = false;
  startup_snapshot_ = initial_snapshot;
  has_startup_snapshot_ = true;
  if (!drives_enabled_) {
    motion_armed_.store(false);
    return Result::ok();
  }
  const Result enabled = enableDrives();
  if (!enabled) {
    return failActivation(enabled);
  }
  motion_armed_.store(false);
  return Result::ok();
}

Result ProductionGtnTransport::startMotion()
{
  motion_armed_.store(true);
  return Result::ok();
}

void ProductionGtnTransport::serviceCardRequest()
{
  int request = 0;
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request = card_request_;
    card_request_ = 0;
  }
  if (request == 0) {
    return;
  }
  Result result = Result::ok();
  if (request == kCardZeroTorque) {
    result = applyZeroTorque();
  } else if (request == kCardBrake) {
    result = applyBrakeHold();
  } else if (request == kCardPvt) {
    result = applyPvtMove();
  } else {
    result = Result::fail(CoreError::kInvalidConfiguration, "未知的卡请求");
  }
  blocking_card_work_.store(true);
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    request_result_ = result;
    request_done_ = true;
  }
  request_cv_.notify_all();
}

bool ProductionGtnTransport::consumeBlockingCardWork()
{
  return blocking_card_work_.exchange(false);
}

Result ProductionGtnTransport::dispatchCardRequest(int request)
{
  if (state_ != State::kActive || !controller_open_ || !ecat_started_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 尚未处于活动状态");
  }
  std::unique_lock<std::mutex> lock(request_mutex_);
  request_done_ = false;
  request_result_ = Result::ok();
  card_request_ = request;
  const bool finished = request_cv_.wait_for(
    lock, std::chrono::seconds(3), [this]() {return request_done_;});
  if (!finished) {
    card_request_ = 0;
    return Result::fail(CoreError::kStateTimeout, "控制周期没有执行卡请求");
  }
  return request_result_;
}

Result ProductionGtnTransport::requestZeroTorque()
{
  return dispatchCardRequest(kCardZeroTorque);
}

Result ProductionGtnTransport::requestEngageBrakes()
{
  return dispatchCardRequest(kCardBrake);
}

Result ProductionGtnTransport::requestMovePvt(
  const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints, double duration_sec)
{
  if (state_ != State::kActive || !controller_open_ || !ecat_started_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 尚未处于活动状态");
  }
  if (waypoints.empty() || waypoints.size() > 8) {
    return Result::fail(CoreError::kInvalidConfiguration, "PVT 路点数量必须在 1 到 8 之间");
  }
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    pvt_waypoints_ = waypoints;
    pvt_duration_sec_ = duration_sec;
  }
  return dispatchCardRequest(kCardPvt);
}

Result ProductionGtnTransport::enterZeroTorque()
{
  return applyZeroTorque();
}

Result ProductionGtnTransport::engageBrakes()
{
  return applyBrakeHold();
}

Result ProductionGtnTransport::applyBrakeHold()
{
  if (!native_api_ || !controller_open_) {
    return Result::fail(CoreError::kTransportUnavailable, "GTN 控制器未打开");
  }
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short code = native_api_->axisOff(core_, static_cast<short>(physical_index + 1));
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_AxisOff(brake)", code, physical_index + 1);
    }
  }
  axes_enabled_ = false;
  zero_torque_ = false;
  pt_started_ = false;
  motion_armed_.store(false);
  std::fprintf(stderr, "[X1CoffeeSystemHardware] 已关断，抱闸锁住\n");
  std::fflush(stderr);
  return Result::ok();
}

Result ProductionGtnTransport::applyPvtMove()
{
  if (!native_api_ || !controller_open_ || !ecat_started_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 尚未处于活动状态");
  }
  if (pt_started_) {
    return Result::fail(
      CoreError::kWriteFailed, "动态 PT 已经在跑，不能再装 PVT。请重启控制栈后再用键盘");
  }
  if (!axes_enabled_) {
    return Result::fail(CoreError::kAxisAlarm, "伺服未上电，PVT 不会动，抱闸保持锁住");
  }
  if (!has_previous_sample_) {
    return Result::fail(CoreError::kReadFailed, "还没有编码器样本，不能装 PVT");
  }
  if (!std::isfinite(pvt_duration_sec_) || pvt_duration_sec_ < 0.2 || pvt_duration_sec_ > 30.0) {
    return Result::fail(CoreError::kInvalidConfiguration, "PVT 时间必须在 0.2 到 30 秒之间");
  }

  const std::size_t waypoint_count = pvt_waypoints_.size();
  const double segment_sec = pvt_duration_sec_ / static_cast<double>(waypoint_count);
  std::vector<std::array<double, kCoffeeAxisCount>> waypoint_pulse(waypoint_count);
  for (std::size_t model_index = 0; model_index < kCoffeeAxisCount; ++model_index) {
    double previous = pulseToPosition(
      model_index, static_cast<std::int32_t>(latest_encoder_pulses_[model_index]));
    for (std::size_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index) {
      const double target = pvt_waypoints_[waypoint_index][model_index];
      const double velocity = std::abs(target - previous) / segment_sec;
      if (!std::isfinite(target) || velocity > 1.0) {
        std::ostringstream stream;
        stream << "PVT 关节速度超过 1 rad/s，未写卡 joint=" <<
          coffeeAxisDescriptors()[model_index].joint_name << " requested_velocity=" << velocity;
        return Result::fail(CoreError::kVelocityLimitViolation, stream.str());
      }
      const Result converted = positionToPulse(
        model_index, target, &waypoint_pulse[waypoint_index][model_index]);
      if (!converted) {
        return converted;
      }
      previous = target;
    }
  }

  const double duration_ms = pvt_duration_sec_ * 1000.0;
  const double step_ms = duration_ms / static_cast<double>(waypoint_count);
  const std::size_t point_count = waypoint_count + 1;
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    const std::size_t model_index = layout_->physical_to_model_index[physical_index];
    std::vector<double> time_ms(point_count);
    std::vector<double> position_pulse(point_count);
    std::vector<double> velocity_pulse_per_ms(point_count, 0.0);
    position_pulse[0] = latest_encoder_pulses_[model_index];
    for (std::size_t waypoint_index = 0; waypoint_index < waypoint_count; ++waypoint_index) {
      time_ms[waypoint_index + 1] = step_ms * static_cast<double>(waypoint_index + 1);
      position_pulse[waypoint_index + 1] = waypoint_pulse[waypoint_index][model_index];
    }
    for (std::size_t point_index = 1; point_index + 1 < point_count; ++point_index) {
      const double span_ms = time_ms[point_index + 1] - time_ms[point_index - 1];
      if (span_ms > 0.0) {
        velocity_pulse_per_ms[point_index] =
          (position_pulse[point_index + 1] - position_pulse[point_index - 1]) / span_ms;
      }
    }
    short code = native_api_->prfPvt(core_, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_PrfPvt", code, axis);
    }
    code = native_api_->pvtTable(
      core_, axis, static_cast<long>(point_count), time_ms.data(), position_pulse.data(),
      velocity_pulse_per_ms.data());
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_PvtTable", code, axis);
    }
    code = native_api_->pvtTableSelect(core_, axis, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_PvtTableSelect", code, axis);
    }
  }
  const short code = native_api_->pvtStart(core_, all_axes_mask_);
  if (code != 0) {
    return nativeFailure(CoreError::kWriteFailed, "GTN_PvtStart", code);
  }
  std::fprintf(
    stderr, "[X1CoffeeSystemHardware] PVT 已启动 duration_s=%.3f\n", pvt_duration_sec_);
  std::fflush(stderr);
  return Result::ok();
}

Result ProductionGtnTransport::applyZeroTorque()
{
  // 接收 PDO 没有 0x6060。GTN_SetEcatAxisMode 会在库内部写邮箱，
  // Gecat 报 “SDO download wait_eventtimeout” 后 libgxn 段错误。
  // 加大 PDO 会被驱动器以 AL 0x0017 拒绝。这里不碰卡，抱闸保持原样。
  (void)native_api_;
  return Result::fail(
    CoreError::kTransportUnavailable,
    "当前过程数据没有模式字，切零力矩会让控制进程崩溃，未改变抱闸");
}

bool ProductionGtnTransport::controllerClosed() const
{
  return shutdown_complete_ && !controller_open_;
}

Result ProductionGtnTransport::enableDrives()
{
  // 手册例程 5-2：GTN_ClrSts、GTN_AxisOn，再等状态字 bit9（0x200）。
  // robot_yingzhi 在总线 OP 且 clearStatus 之后做 doEnableAllAxes，这里同样不改编码器、不发运动。
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    short code = native_api_->clearStatus(core_, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kAxisAlarm, "GTN_ClrSts(enable)", code, axis);
    }
    code = native_api_->axisOn(core_, axis);
    std::fprintf(stderr, "[X1CoffeeSystemHardware] GTN_AxisOn axis=%d code=%d\n", axis, code);
    std::fflush(stderr);
    if (code != 0) {
      return nativeFailure(CoreError::kAxisAlarm, "GTN_AxisOn", code, axis);
    }
  }
  axes_enabled_ = true;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_enabled = true;
    for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
      ++physical_index)
    {
      const short axis = static_cast<short>(physical_index + 1);
      std::int32_t status = 0;
      const short code = native_api_->getStatus(core_, axis, &status);
      if (code != 0) {
        return nativeFailure(CoreError::kReadFailed, "GTN_GetSts(enable)", code, axis);
      }
      const auto bits = static_cast<std::uint32_t>(status);
      if ((bits & (kAlarmMask | kFollowErrorMask | kEmergencyInputMask)) != 0) {
        std::ostringstream stream;
        stream << "GTN_AxisOn 后轴状态异常 axis=" << axis << " status_bits=0x" << std::hex << bits;
        return Result::fail(CoreError::kAxisAlarm, stream.str());
      }
      if ((bits & kEnabledMask) == 0) {
        all_enabled = false;
      }
    }
    if (all_enabled) {
      std::fprintf(
        stderr, "[X1CoffeeSystemHardware] 15 轴已使能，未下发运动\n");
      std::fflush(stderr);
      return Result::ok();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return Result::fail(CoreError::kAxisAlarm, "GTN_AxisOn 后 2 秒内状态字 bit9 未全部置位");
}

std::filesystem::path ProductionGtnTransport::gecatLogPath() const
{
  // 固高库把 Gecat.log 写在进程工作目录，每个进程从头写。
  std::error_code error;
  const auto cwd = std::filesystem::current_path(error);
  return error ? std::filesystem::path("Gecat.log") : cwd / "Gecat.log";
}

std::uintmax_t ProductionGtnTransport::currentGecatLogSize() const
{
  std::error_code error;
  const auto path = gecatLogPath();
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return 0;
  }
  const auto size = std::filesystem::file_size(path, error);
  return error ? 0 : size;
}

Result ProductionGtnTransport::waitForAllSlavesOp()
{
  if (ecat_op_wait_ms_ <= 0) {
    return Result::ok();
  }
  const auto path = gecatLogPath();
  std::uintmax_t offset = op_log_offset_;
  std::string pending;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(ecat_op_wait_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    std::error_code error;
    if (std::filesystem::is_regular_file(path, error) && !error) {
      const auto size = std::filesystem::file_size(path, error);
      if (!error) {
        if (size < offset) {
          offset = 0;
        }
        if (size > offset) {
          std::ifstream input(path, std::ios::binary);
          input.seekg(static_cast<std::streamoff>(offset));
          std::string chunk(static_cast<std::size_t>(size - offset), '\0');
          input.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
          offset = size;
          pending.append(chunk);
        }
      }
    }
    if (pending.find("All slaves are in OP") != std::string::npos) {
      std::fprintf(stderr, "[X1CoffeeSystemHardware] EtherCAT 15 个从站已进入 OP\n");
      std::fflush(stderr);
      return Result::ok();
    }
    if (pending.find("SDO upload") != std::string::npos) {
      return Result::fail(
        CoreError::kEthercatNotReady, "等待 OP 时 Gecat.log 出现 SDO 上传超时");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::ostringstream stream;
  stream << ecat_op_wait_ms_ << " ms 内 Gecat.log 没有出现 All slaves are in OP: " << path.string();
  return Result::fail(CoreError::kEthercatNotReady, stream.str());
}

Result ProductionGtnTransport::initializeController()
{
  // 例程 5-1 的主干是 Open、Terminate、InitEcatCommEx、IsEcatReady、Start、Reset。
  // 2026-10-23 16:54 robot_yingzhi 到达 OP 的记录在 Terminate 之前还有 Reset 和
  // LoadConfig，IsEcatReady 返回后约 500 ms 才 Start，Start 返回后约 200 ms 才第二次 Reset，
  // 然后一次清完全部轴状态。省掉初始化前的 Reset/LoadConfig 时，从站停在 SDO 超时，进不了 OP。
  // 不调用 GTN_AlarmOff，也不调用 GTN_AxisOn。
  short code = native_api_->open();
  if (code != 0) {
    return nativeFailure(CoreError::kTransportUnavailable, "GTN_Open", code);
  }
  controller_open_ = true;
  shutdown_complete_ = false;
  native_stop_sent_ = false;

  // 不把工作目录改到 eni 所在目录。robot_yingzhi 的工作目录本身没有 Gecat.xml，
  // eni 用绝对路径传给 GTN_InitEcatCommEx。切到厂商目录会让库再读到另一份 XML，
  // 操作线程随后在 “Link is up” 处拿到状态非法的数据报并段错误。
  code = native_api_->reset(core_);
  if (code != 0) {
    return nativeFailure(CoreError::kTransportUnavailable, "GTN_Reset", code);
  }
  const std::string config_path = loadConfigPath(gtn_config_path_);
  code = native_api_->loadConfig(core_, config_path.c_str());
  if (code != 0) {
    return nativeFailure(CoreError::kInvalidConfiguration, "GTN_LoadConfig", code);
  }
  code = native_api_->terminateEcat(core_);
  if (code != 0) {
    std::fprintf(
      stderr, "[X1CoffeeSystemHardware] GTN_TerminateEcatComm(pre_init) returned %d\n", code);
    std::fflush(stderr);
  }
  // robot_yingzhi 的 initEcat 在 Terminate 返回后先 nanosleep 50 ms 再调用
  // GTN_InitEcatCommEx。紧接着初始化会在操作线程刚启动时撞上尚未释放的主站，
  // 数据报状态变成非法值，随后段错误。
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  Result init_failure = Result::ok();
  const int attempts = std::max(1, ecat_init_attempts_);
  for (int attempt = 0; attempt < attempts; ++attempt) {
    if (attempt > 0) {
      // 返回 -10 时对方的重试是 Terminate、100 ms、Reset、LoadConfig、100 ms，再 Init 一次。
      native_api_->terminateEcat(core_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      code = native_api_->reset(core_);
      if (code != 0) {
        return nativeFailure(CoreError::kTransportUnavailable, "GTN_Reset(ecat_retry)", code);
      }
      code = native_api_->loadConfig(core_, config_path.c_str());
      if (code != 0) {
        return nativeFailure(CoreError::kInvalidConfiguration, "GTN_LoadConfig(ecat_retry)", code);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    op_log_offset_ = currentGecatLogSize();
    code = native_api_->initEcat(core_, ecat_config_path_.c_str());
    if (code == 0) {
      init_failure = Result::ok();
      break;
    }
    std::fprintf(
      stderr,
      "[X1CoffeeSystemHardware] GTN_InitEcatCommEx returned %d (0 成功, -8 配置不匹配, "
      "-9 未找到 eni, -10 未找到从站) attempt %d/%d\n",
      code, attempt + 1, attempts);
    std::fflush(stderr);
    init_failure = nativeFailure(CoreError::kEthercatNotReady, "GTN_InitEcatCommEx", code);
    // 手册把 -10 定义为未找到从站。只有指令真正返回后才再试一次。
    if (code != -10) {
      break;
    }
  }
  if (!init_failure) {
    return init_failure;
  }
  ecat_initialized_ = true;

  short ready = 0;
  for (int retry = 0; retry < ready_retries_; ++retry) {
    code = native_api_->isEcatReady(core_, &ready);
    if (code == 0 && ready == 1) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(ready_interval_ms_));
  }
  if (code != 0 || ready != 1) {
    return nativeFailure(CoreError::kEthercatNotReady, "GTN_IsEcatReady", code);
  }
  if (ecat_start_delay_ms_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ecat_start_delay_ms_));
  }
  code = native_api_->startEcat(core_);
  if (code != 0) {
    return nativeFailure(CoreError::kEthercatNotReady, "GTN_StartEcatComm", code);
  }
  ecat_started_ = true;
  if (ecat_step_delay_ms_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ecat_step_delay_ms_));
  }
  code = native_api_->reset(core_);
  if (code != 0) {
    return nativeFailure(CoreError::kTransportUnavailable, "GTN_Reset(post_ecat)", code);
  }
  // GTN_InitEcatCommEx 和 GTN_StartEcatComm 返回时从站还在 PREOP，主站线程随后才把它们切到 OP。
  // 这期间逐轴清状态、查轴表和读电流力矩，会让从站 0 的 SDO 上传超时，库在 list_del 里段错误。
  const Result op = waitForAllSlavesOp();
  if (!op) {
    return op;
  }
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    code = native_api_->clearStatus(core_, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kAxisAlarm, "GTN_ClrSts(post_ecat)", code, axis);
    }
  }
  const Result bus = verifyConfiguredAxisBus("启动后");
  if (!bus) {
    return bus;
  }
  return Result::ok();
}

Result ProductionGtnTransport::verifyConfiguredAxisBus(const char * stage)
{
  short motion_slaves = -1;
  short io_slaves = -1;
  short code = native_api_->getEcatSlaves(core_, &motion_slaves, &io_slaves);
  if (code != 0) {
    return nativeFailure(CoreError::kEthercatNotReady, "GTN_GetEcatSlaves", code);
  }
  if (motion_slaves != static_cast<short>(layout_->physical_axis_count)) {
    std::ostringstream stream;
    stream << stage << " EtherCAT 运动从站数量错误: actual=" << motion_slaves <<
      " expected=" << layout_->physical_axis_count << " profile=" << layout_->name;
    return Result::fail(CoreError::kEthercatNotReady, stream.str());
  }

  short axis_count = -1;
  code = native_api_->getAxisCount(core_, &axis_count);
  if (code == kUnsupportedCommand) {
    // The motion-slave count above is still checked strictly.  Only the known
    // unsupported axis-table query is allowed to fall back to that safety gate.
    return Result::ok();
  }
  if (code != 0) {
    return nativeFailure(CoreError::kEthercatNotReady, "GTN_GetMcEcatAxisNum", code);
  }
  if (axis_count != static_cast<short>(layout_->physical_axis_count)) {
    code = native_api_->setAxisCount(core_, static_cast<short>(layout_->physical_axis_count));
    if (code != 0) {
      return nativeFailure(CoreError::kEthercatNotReady, "GTN_SetMcEcatAxisNum", code);
    }
    code = native_api_->getAxisCount(core_, &axis_count);
    if (code != 0 || axis_count != static_cast<short>(layout_->physical_axis_count)) {
      std::ostringstream stream;
      stream << "GTN 轴表无法严格设置为 " << layout_->physical_axis_count << " 轴";
      return Result::fail(CoreError::kEthercatNotReady, stream.str());
    }
  }
  return Result::ok();
}

Result ProductionGtnTransport::enableMotionAndStartPt(const HardwareSnapshot & initial_snapshot)
{
  // 上电保持时伺服已经在闭环。此时改 GTN_SetEncPos 会把绝对编码器计数写进正在跟踪的坐标，
  // 14 个轴会一起报警并掉使能。先 AxisOff（抱闸锁住），再按当前编码器对齐，然后重新 AxisOn。
  if (axes_enabled_) {
    for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
      ++physical_index)
    {
      const short code = native_api_->axisOff(
        core_, static_cast<short>(physical_index + 1));
      if (code != 0) {
        return nativeFailure(CoreError::kWriteFailed, "GTN_AxisOff(before PT)", code, physical_index + 1);
      }
    }
    axes_enabled_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  HardwareSnapshot synced = initial_snapshot;
  if (has_previous_sample_) {
    for (std::size_t model_index = 0; model_index < kCoffeeAxisCount; ++model_index) {
      synced.axes[model_index].encoder_pulse_raw = latest_encoder_pulses_[model_index];
    }
  }

  // 在使能前把规划器和编码器对齐到当前绝对编码器位置，避免切入 PT 时首点跳变。
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const std::size_t model_index = layout_->physical_to_model_index[physical_index];
    const short axis = static_cast<short>(physical_index + 1);
    const auto raw_position = static_cast<std::int32_t>(
      synced.axes[model_index].encoder_pulse_raw);
    short code = native_api_->setPlannedPosition(core_, axis, raw_position);
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_SetPrfPos", code, axis);
    }
    code = native_api_->setEncoderPosition(core_, axis, raw_position);
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_SetEncPos", code, axis);
    }
    code = native_api_->synchronizeAxisPosition(
      core_, static_cast<std::int32_t>(1U << physical_index));
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_SynchAxisPos", code, axis);
    }
  }

  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    short code = native_api_->clearStatus(core_, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kAxisAlarm, "GTN_ClrSts", code, axis);
    }
    code = native_api_->axisOn(core_, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kAxisAlarm, "GTN_AxisOn", code, axis);
    }
  }
  axes_enabled_ = true;

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    std::int32_t status = 0;
    const short code = native_api_->getStatus(core_, axis, &status);
    if (code != 0) {
      return nativeFailure(CoreError::kReadFailed, "GTN_GetSts(post-enable)", code, axis);
    }
    const std::uint32_t bits = static_cast<std::uint32_t>(status);
    if ((bits & kEnabledMask) == 0 ||
      (bits & (kAlarmMask | kFollowErrorMask | kEmergencyInputMask)) != 0)
    {
      std::ostringstream stream;
      stream << "轴使能后状态不安全 axis=" << axis << " status_bits=0x" <<
        std::hex << bits;
      return Result::fail(CoreError::kAxisAlarm, stream.str());
    }
  }

  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    const short code = native_api_->configureDynamicPt(core_, axis);
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_PrfPt(dynamic)", code, axis);
    }
  }

  pt_time_ms_ = 0;
  for (int point = 0; point < pt_prebuffer_points_; ++point) {
    pt_time_ms_ += pt_cycle_ms_;
    for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
      ++physical_index)
    {
      const std::size_t model_index = layout_->physical_to_model_index[physical_index];
      const short axis = static_cast<short>(physical_index + 1);
      // PT FIFO capacity is controller/firmware dependent.  Never assume that
      // the configured prebuffer fits: querying space immediately before each
      // point prevents the startup burst from overflowing the FIFO and
      // latching an emergency stop (which otherwise makes all command
      // interfaces unavailable to controller_manager).
      short space = 0;
      const short space_code = native_api_->ptSpace(core_, axis, &space);
      if (space_code != 0) {
        return nativeFailure(CoreError::kWriteFailed, "GTN_PtSpace(prebuffer)", space_code, axis);
      }
      if (space < 1) {
        return Result::fail(
          CoreError::kWriteFailed,
          "动态 PT 启动预填充超过控制器 FIFO 容量，请降低 pt_prebuffer_points");
      }
      const short code = native_api_->ptData(
        core_, axis, static_cast<double>(synced.axes[model_index].encoder_pulse_raw),
        pt_time_ms_, kPtSegmentNormal);
      if (code != 0) {
        return nativeFailure(CoreError::kWriteFailed, "GTN_PtData(prebuffer)", code, axis);
      }
    }
  }
  const short code = native_api_->ptStart(core_, all_axes_mask_);
  if (code != 0) {
    return nativeFailure(CoreError::kWriteFailed, "GTN_PtStart", code);
  }
  pt_started_ = true;
  std::fprintf(stderr, "[X1CoffeeSystemHardware] PT started mask=0x%x prebuffer_ms=%d\n",
    static_cast<unsigned>(all_axes_mask_), pt_time_ms_);
  std::fflush(stderr);
  return Result::ok();
}

Result ProductionGtnTransport::read(HardwareSnapshot * snapshot)
{
  if (snapshot == nullptr) {
    return Result::fail(CoreError::kInvalidConfiguration, "read 的 snapshot 不能为空");
  }
  if (state_ != State::kActive || !ecat_started_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 尚未处于活动状态");
  }
  serviceCardRequest();

  const auto read_started_at = std::chrono::steady_clock::now();
  HardwareSnapshot next;
  next.sampled_at = std::chrono::steady_clock::now();
  const double elapsed = has_previous_sample_ ?
    std::chrono::duration<double>(next.sampled_at - previous_sample_time_).count() : 0.0;
  bool any_alarm = false;
  bool any_follow_error = false;
  bool any_emergency = false;
  std::ostringstream fault_axes;
  bool first_fault_axis = true;
  std::size_t alarm_count = 0;
  std::size_t follow_error_count = 0;
  std::size_t emergency_count = 0;

  std::array<std::int32_t, kCoffeeMaxPhysicalAxisCount> status_values{};
  const auto status_started_at = std::chrono::steady_clock::now();
  // The target GTN library/controller combination returned inconsistent data
  // for GTN_GetSts(count > 1).  Keep safety-critical status reads per-axis,
  // matching the proven Qt production path.
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const short axis = static_cast<short>(physical_index + 1);
    const short status_result = native_api_->getStatus(
      core_, axis, &status_values[physical_index]);
    if (status_result != 0) {
      return nativeFailure(CoreError::kReadFailed, "GTN_GetSts", status_result, axis);
    }
  }
  const auto status_finished_at = std::chrono::steady_clock::now();
  const bool refresh_diagnostics = read_current_torque_ && (!has_diagnostic_sample_ ||
    (diagnostic_poll_tick_ % static_cast<std::uint64_t>(diagnostic_poll_div_) == 0));
  double diagnostics_ms = 0.0;
  if (refresh_diagnostics) {
    const auto diagnostics_started_at = std::chrono::steady_clock::now();
    for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
      ++physical_index)
    {
      short current = 0;
      short torque = 0;
      const short act_result = native_api_->getAxisAct(
        core_, static_cast<short>(physical_index + 1), &current, &torque);
      if (act_result == 0) {
        latest_currents_[physical_index] = current;
        latest_torques_[physical_index] = torque;
        latest_current_valid_[physical_index] = true;
        latest_torque_valid_[physical_index] = true;
      } else {
        latest_current_valid_[physical_index] = false;
        latest_torque_valid_[physical_index] = false;
      }
    }
    has_diagnostic_sample_ = true;
    const auto diagnostics_finished_at = std::chrono::steady_clock::now();
    diagnostics_ms = std::chrono::duration<double, std::milli>(
      diagnostics_finished_at - diagnostics_started_at).count();
  }

  const auto encoder_started_at = std::chrono::steady_clock::now();
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const std::size_t model_index = layout_->physical_to_model_index[physical_index];
    const short axis = static_cast<short>(physical_index + 1);
    std::int32_t encoder = 0;
    short code = 0;
    const int attempts = has_previous_sample_ ?
      kEncoderBusyAttemptsCyclic : kEncoderBusyAttemptsInitial;
    const int wait_ms = has_previous_sample_ ?
      kEncoderBusyWaitCyclicMs : kEncoderBusyWaitInitialMs;
    for (int attempt = 0; attempt < attempts; ++attempt) {
      code = native_api_->getEncoder(core_, axis, &encoder);
      if (code != kControllerLockBusy) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
    if (code == kControllerLockBusy && has_previous_sample_) {
      encoder = latest_encoder_pulses_[model_index];
      code = 0;
      static int lock_busy_reused = 0;
      ++lock_busy_reused;
      if (lock_busy_reused == 1 || lock_busy_reused % 100 == 0) {
        std::fprintf(
          stderr,
          "[X1CoffeeSystemHardware] GTN_GetEcatEncPos lock busy, reused last encoder "
          "axis=%d count=%d\n",
          axis, lock_busy_reused);
        std::fflush(stderr);
      }
    }
    if (code != 0) {
      return nativeFailure(CoreError::kReadFailed, "GTN_GetEcatEncPos", code, axis);
    }

    AxisState & state = next.axes[model_index];
    state.encoder_pulse_raw = encoder;
    state.position = pulseToPosition(model_index, encoder);
    state.velocity = has_previous_sample_ && elapsed > 0.0 ?
      (state.position - previous_positions_[model_index]) / elapsed : 0.0;
    state.status_bits = static_cast<std::uint32_t>(status_values[physical_index]);
    state.enabled = (state.status_bits & kEnabledMask) != 0;
    state.moving = (state.status_bits & kMovingMask) != 0;
    state.alarm = (state.status_bits & kAlarmMask) != 0;
    state.follow_error = (state.status_bits & kFollowErrorMask) != 0;
    state.emergency_input_active = (state.status_bits & kEmergencyInputMask) != 0;
    state.encoder_valid = true;
    state.position_valid = std::isfinite(state.position);
    state.velocity_valid = std::isfinite(state.velocity);

    state.current_permille = latest_currents_[physical_index];
    state.torque_permille = latest_torques_[physical_index];
    state.current_valid = latest_current_valid_[physical_index];
    state.torque_valid = latest_torque_valid_[physical_index];
    previous_positions_[model_index] = state.position;
    latest_encoder_pulses_[model_index] = encoder;
    any_alarm = any_alarm || state.alarm;
    any_follow_error = any_follow_error || state.follow_error;
    any_emergency = any_emergency || state.emergency_input_active;
    alarm_count += state.alarm ? 1U : 0U;
    follow_error_count += state.follow_error ? 1U : 0U;
    emergency_count += state.emergency_input_active ? 1U : 0U;
    if (state.alarm || state.follow_error || state.emergency_input_active) {
      if (!first_fault_axis) {
        fault_axes << ",";
      }
      first_fault_axis = false;
      fault_axes << "axis=" << axis <<
        " joint=" << coffeeAxisDescriptors()[model_index].joint_name <<
        " status=0x" << std::hex << state.status_bits << std::dec <<
        " alarm=" << (state.alarm ? 1 : 0) <<
        " follow_error=" << (state.follow_error ? 1 : 0) <<
        " emergency=" << (state.emergency_input_active ? 1 : 0) <<
        " enabled=" << (state.enabled ? 1 : 0);
    }
  }
  const auto encoder_finished_at = std::chrono::steady_clock::now();

  previous_sample_time_ = next.sampled_at;
  has_previous_sample_ = true;
  ++diagnostic_poll_tick_;
  const auto read_finished_at = std::chrono::steady_clock::now();
  ++read_timing_tick_;
  if (timing_log_div_ > 0 && read_timing_tick_ % static_cast<std::uint64_t>(timing_log_div_) == 0) {
    const auto to_ms = [](const auto duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
      };
    const double status_ms = to_ms(status_finished_at - status_started_at);
    const double encoder_ms = to_ms(encoder_finished_at - encoder_started_at);
    const double total_ms = to_ms(read_finished_at - read_started_at);
    std::fprintf(
      stderr,
      "[X1CoffeeSystemHardware] GTN read timing: status_ms=%.3f encoder_ms=%.3f "
      "diagnostics_ms=%.3f diagnostics=%s total_ms=%.3f\n",
      status_ms, encoder_ms, diagnostics_ms,
      refresh_diagnostics ? "refreshed" : "cached", total_ms);
    std::fflush(stderr);
  }
  *snapshot = next;
  // 只读标零要能读到限位和编码器。报警、跟随误差、急停留在状态字里发布。
  // 允许运动时这些标志仍然拒绝继续，避免在故障状态下使能。
  if (!motion_enabled_) {
    return Result::ok();
  }
  if (any_emergency) {
    std::ostringstream stream;
    stream << "GTN 轴状态字检测到急停输入（bit8）: emergency_count=" << emergency_count <<
      " fault_axes=[" << fault_axes.str() << "]";
    return Result::fail(CoreError::kEmergencyStop, stream.str());
  }
  if (any_alarm || any_follow_error) {
    std::ostringstream stream;
    stream << "GTN 轴状态字清除后仍存在驱动报警或跟随误差: alarm_count=" << alarm_count <<
      " follow_error_count=" << follow_error_count <<
      " fault_axes=[" << fault_axes.str() << "]";
    return Result::fail(CoreError::kAxisAlarm, stream.str());
  }
  return Result::ok();
}

Result ProductionGtnTransport::write(const PositionCommandFrame & command)
{
  if (state_ != State::kActive || !ecat_started_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 尚未处于活动状态");
  }
  serviceCardRequest();
  if (zero_torque_) {
    return Result::ok();
  }
  if (stop_latched_) {
    return Result::fail(CoreError::kEmergencyStop, "Stop is latched; explicit control-stack reinitialization required");
  }
  if (!motion_enabled_) {
    // controller_manager 即使没有活动运动控制器也会周期调用 write()。
    // 只读模式必须保持硬件循环健康，但绝不向驱动器下发运动数据。
    // startMotion 之后仍然如此：motion_enabled 为假时不下发 PT。
    return Result::ok();
  }
  if (!motion_armed_.load()) {
    // 轨迹控制器尚未就绪时，第一帧命令差也不能去开 PT。
    return Result::ok();
  }
  if (!pt_started_) {
    if (!has_startup_snapshot_) {
      return Result::fail(CoreError::kWriteFailed, "动态 PT 启动快照尚未就绪");
    }
    const Result motion = enableMotionAndStartPt(startup_snapshot_);
    if (!motion) {
      return motion;
    }
  }
  if (pt_time_ms_ > std::numeric_limits<std::int32_t>::max() - pt_cycle_ms_) {
    return Result::fail(CoreError::kWriteFailed, "动态 PT 时间戳即将溢出，需要安全重启控制栈");
  }

  std::array<double, kCoffeeAxisCount> pulse_targets{};
  short minimum_space = std::numeric_limits<short>::max();
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const std::size_t model_index = layout_->physical_to_model_index[physical_index];
    const Result converted = positionToPulse(
      model_index, command.positions[model_index], &pulse_targets[model_index]);
    if (!converted) {
      return converted;
    }
    short space = 0;
    const short code = native_api_->ptSpace(
      core_, static_cast<short>(physical_index + 1), &space);
    if (code != 0) {
      return nativeFailure(CoreError::kWriteFailed, "GTN_PtSpace", code, physical_index + 1);
    }
    minimum_space = std::min(minimum_space, space);
    if (space < 1) {
      // 启动预填充刚占满 FIFO 时，这一拍跳过，等控制器消耗后再写。
      // 连续约 0.5 秒仍没有空间才急停。
      ++pt_fifo_stall_;
      if (pt_fifo_stall_ == 1 || pt_fifo_stall_ == 50) {
        std::fprintf(stderr,
          "[X1CoffeeSystemHardware] PT frame not queued: axis=%zu space=%d stalled_cycles=%d\n",
          physical_index + 1, static_cast<int>(space), pt_fifo_stall_);
        std::fflush(stderr);
      }
      if (pt_fifo_stall_ > 50) {
        stop(true);
        return Result::fail(CoreError::kStateTimeout, "动态 PT FIFO 无可用空间，已触发急停");
      }
      return Result::ok();
    }
  }
  // Reset only after every axis has space. Resetting inside the loop masks
  // persistent backpressure on any axis after axis 1.
  pt_fifo_stall_ = 0;

  pt_time_ms_ += pt_cycle_ms_;
  for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
    ++physical_index)
  {
    const std::size_t model_index = layout_->physical_to_model_index[physical_index];
    const short axis = static_cast<short>(physical_index + 1);
    const short code = native_api_->ptData(
      core_, axis, pulse_targets[model_index], pt_time_ms_, kPtSegmentNormal);
    if (code != 0) {
      stop(true);
      return nativeFailure(CoreError::kWriteFailed, "GTN_PtData", code, axis);
    }
  }
  if (pt_time_ms_ % 1000 < pt_cycle_ms_) {
    std::size_t largest_error = 0;
    for (std::size_t i = 1; i < kCoffeeAxisCount; ++i) {
      if (std::abs(pulse_targets[i] - latest_encoder_pulses_[i]) >
        std::abs(pulse_targets[largest_error] - latest_encoder_pulses_[largest_error]))
      {
        largest_error = i;
      }
    }
    // Read one axis per report, individually like the status/encoder path.
    // These diagnostics are optional and never change card state.
    const auto physical = layout_->model_to_physical_axis[largest_error];
    double planner_position = std::numeric_limits<double>::quiet_NaN();
    std::int32_t planner_mode = -1;
    const short position_code = native_api_->getProfilePosition(
      core_, static_cast<short>(physical), &planner_position);
    const short mode_code = native_api_->getProfileMode(
      core_, static_cast<short>(physical), &planner_mode);
    std::fprintf(stderr,
      "[X1CoffeeSystemHardware] PT frame queued: time_ms=%d min_space=%d "
      "max_error_joint=%s command_pulse=%.3f encoder_pulse=%d "
      "planner_pulse=%.3f planner_rc=%d planner_mode=%d mode_rc=%d\n",
      pt_time_ms_, static_cast<int>(minimum_space),
      std::string(coffeeAxisDescriptors()[largest_error].joint_name).c_str(),
      pulse_targets[largest_error], latest_encoder_pulses_[largest_error],
      planner_position, static_cast<int>(position_code), planner_mode,
      static_cast<int>(mode_code));
    std::fflush(stderr);
  }
  return Result::ok();
}

Result ProductionGtnTransport::stop(bool emergency)
{
  // 卡已经 Close 时再 GTN_Stop 会返回 -8，并且曾经跟着段错误。这里直接成功返回。
  if (!controller_open_ && shutdown_complete_) {
    return Result::ok();
  }
  stop_latched_ = true;
  emergency_latched_ = emergency_latched_ || emergency;
  if (!native_api_ || !controller_open_ || !ecat_started_) {
    return Result::fail(CoreError::kTransportUnavailable, "GTN controller/bus is not active; stop cannot be acknowledged");
  }
  // GEN basic programming manual, command 185: option is a per-axis bitmask.
  // 0 = smooth stop; set bits = emergency stop. Never downgrade a prior emergency.
  const short code = native_api_->stop(
    core_, all_axes_mask_, emergency_latched_ ? all_axes_mask_ : 0);
  native_stop_sent_ = true;
  pt_started_ = false;
  return code == 0 ? Result::ok() : nativeFailure(CoreError::kWriteFailed, "GTN_Stop", code);
}

Result ProductionGtnTransport::deactivate()
{
  Result first_error = Result::ok();
  const bool was_open = controller_open_;
  if (native_api_) {
    // stop 已经调用过 GTN_Stop 时不再补一次。关断仍然只走 AxisOff、Terminate、Close。
    if (ecat_started_ && controller_open_ && !native_stop_sent_) {
      const Result stopped = stop(false);
      if (!stopped) {
        first_error = stopped;
      }
    }
    if (axes_enabled_ && controller_open_) {
      for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
        ++physical_index)
      {
        const short code = native_api_->axisOff(
          core_, static_cast<short>(physical_index + 1));
        if (code != 0 && first_error) {
          first_error = nativeFailure(
            CoreError::kWriteFailed, "GTN_AxisOff", code, physical_index + 1);
        }
      }
    }
    if (ecat_initialized_ && controller_open_) {
      const short code = native_api_->terminateEcat(core_);
      if (code != 0 && first_error) {
        first_error = nativeFailure(CoreError::kEthercatNotReady, "GTN_TerminateEcatComm", code);
      }
    }
    if (controller_open_) {
      const short code = native_api_->close();
      // 旧运行时将 -8 视为“已关闭”的兼容返回值。
      if (code != 0 && code != -8 && first_error) {
        first_error = nativeFailure(CoreError::kTransportUnavailable, "GTN_Close", code);
      }
    }
  }

  if (was_open) {
    shutdown_complete_ = true;
  }
  controller_open_ = false;
  ecat_initialized_ = false;
  ecat_started_ = false;
  axes_enabled_ = false;
  pt_started_ = false;
  motion_armed_.store(false);
  zero_torque_ = false;
  has_previous_sample_ = false;
  has_diagnostic_sample_ = false;
  diagnostic_poll_tick_ = 0;
  latest_current_valid_.fill(false);
  latest_torque_valid_.fill(false);
  if (state_ != State::kUnconfigured) {
    state_ = State::kConfigured;
  }
  return first_error;
}

Result ProductionGtnTransport::nativeFailure(
  CoreError error, const char * api, short code, int axis) const
{
  std::ostringstream stream;
  stream << api << " 返回错误 code=" << code << " core=" << core_;
  if (axis > 0) {
    stream << " axis=" << axis;
  }
  return Result::fail(error, stream.str());
}

Result ProductionGtnTransport::failActivation(const Result & cause)
{
  const bool was_open = controller_open_;
  if (native_api_ && controller_open_) {
    native_api_->stop(core_, all_axes_mask_, all_axes_mask_);
    native_stop_sent_ = true;
    if (axes_enabled_) {
      for (std::size_t physical_index = 0; physical_index < layout_->physical_axis_count;
        ++physical_index)
      {
        native_api_->axisOff(core_, static_cast<short>(physical_index + 1));
      }
    }
    if (ecat_initialized_) {
      native_api_->terminateEcat(core_);
    }
    native_api_->close();
  }
  if (was_open) {
    shutdown_complete_ = true;
  }
  controller_open_ = false;
  ecat_initialized_ = false;
  ecat_started_ = false;
  axes_enabled_ = false;
  pt_started_ = false;
  motion_armed_.store(false);
  zero_torque_ = false;
  has_previous_sample_ = false;
  state_ = State::kConfigured;
  return cause;
}

double ProductionGtnTransport::pulseToPosition(std::size_t index, std::int32_t pulse) const
{
  const AxisCalibration & item = calibration_[index];
  const double delta = normalizePulseDelta(
    static_cast<double>(pulse) - item.zero_offset_pulse, item.wrap_period_pulse);
  return item.model_zero + delta / item.pulses_per_unit;
}

Result ProductionGtnTransport::positionToPulse(
  std::size_t index, double position, double * pulse) const
{
  if (pulse == nullptr || !std::isfinite(position)) {
    return Result::fail(CoreError::kCommandNotFinite, "位置命令不是有限数值");
  }
  const AxisCalibration & item = calibration_[index];
  const double canonical = item.zero_offset_pulse +
    (position - item.model_zero) * item.pulses_per_unit;
  const double converted = nearestEquivalentPulse(
    canonical, latest_encoder_pulses_[index], item.wrap_period_pulse);
  if (!std::isfinite(converted) || converted < std::numeric_limits<std::int32_t>::min() ||
    converted > std::numeric_limits<std::int32_t>::max())
  {
    return Result::fail(CoreError::kWriteFailed, "位置命令转换后超出 GTN int32 脉冲范围");
  }
  *pulse = converted;
  return Result::ok();
}

}  // namespace x1_coffee_hardware_core
