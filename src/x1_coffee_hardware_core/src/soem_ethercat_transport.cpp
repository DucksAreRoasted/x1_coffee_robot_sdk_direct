#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>

#include <pthread.h>
#include <dlfcn.h>
#include <sched.h>
#include <sys/socket.h>
#include <time.h>

#include "soem/soem.h"
#include "x1_coffee_hardware_core/zeroerr_drive.hpp"
#include "enable_trace.hpp"

namespace x1_coffee_hardware_core
{
namespace
{

// 主站节拍：按绝对时间睡眠，并用 DC 时间把发帧时刻锁到 SYNC0 两个脉冲的中间。
// usleep(500) 实测平均约 560 µs，比 SYNC0 慢，帧会周期性撞上驱动器采样时刻。
class DcCycleTimer
{
public:
  static constexpr std::int64_t kCycleNs = 500000;

  void start()
  {
    clock_gettime(CLOCK_MONOTONIC, &next_);
    integral_ = 0;
    overruns_ = 0;
    max_late_ns_ = 0;
  }

  // dc_time 为上一帧带回的参考时钟时间（ns），为 0 时只按固定周期睡眠。
  void wait(std::int64_t dc_time)
  {
    std::int64_t offset = 0;
    if (dc_time > 0) {
      // 目标相位 = 周期一半，与 SYNC0（相位 0）错开。
      std::int64_t delta = (dc_time - kCycleNs / 2) % kCycleNs;
      if (delta > kCycleNs / 2) {
        delta -= kCycleNs;
      }
      if (delta > 0) {
        ++integral_;
      } else if (delta < 0) {
        --integral_;
      }
      offset = -(delta / 100) - (integral_ / 20);
    }
    addNs(kCycleNs + offset);
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    const std::int64_t late = toNs(now) - toNs(next_);
    if (late > 0) {
      // 已经错过本周期：记一次超时，从当前时刻重新起算，不连发补帧。
      ++overruns_;
      if (late > max_late_ns_) {
        max_late_ns_ = late;
      }
      next_ = now;
      return;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_, nullptr);
  }

  long overruns() const {return overruns_;}
  std::int64_t maxLateNs() const {return max_late_ns_;}
  void resetStats() {overruns_ = 0; max_late_ns_ = 0;}

private:
  static std::int64_t toNs(const timespec & t)
  {
    return static_cast<std::int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
  }

  void addNs(std::int64_t ns)
  {
    const std::int64_t total = toNs(next_) + ns;
    next_.tv_sec = static_cast<time_t>(total / 1000000000LL);
    next_.tv_nsec = static_cast<long>(total % 1000000000LL);
  }

  timespec next_{};
  std::int64_t integral_{0};
  long overruns_{0};
  std::int64_t max_late_ns_{0};
};

void say(const char * fmt, ...);

// 周期线程提成 SCHED_FIFO。非 root 时失败只打印，不影响原有行为。
void raiseCycleThreadPriority()
{
  sched_param param{};
  param.sched_priority = 80;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (rc == 0) {
    say("✓ 周期线程已设为实时优先级");
  } else {
    say("! 周期线程没能设为实时优先级，返回 %d", rc);
  }
}

struct TorqueOutput
{
  std::uint16_t control_word;
  std::int16_t target_torque;
} __attribute__((packed));

// 手册 5.9.1 的状态字、实际扭矩、实际电流，再加 0x6064，激活后仍能读关节位置。
struct TorqueInput
{
  std::uint16_t status_word;
  std::int16_t actual_torque;
  std::int16_t actual_current;
  std::int32_t actual_position;
} __attribute__((packed));

void debugf(const char * fmt, ...)
{
  // 英文调试默认不进终端。需要时再设 X1_ECAT_DEBUG=1。
  if (std::getenv("X1_ECAT_DEBUG") == nullptr) {
    return;
  }
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  std::fprintf(stderr, "[调试] %s\n", buffer);
  std::fflush(stderr);
}

void say(const char * fmt, ...)
{
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  std::fprintf(stderr, "%s\n", buffer);
  std::fflush(stderr);
}

std::atomic<int> g_wake_socket{-1};
std::atomic<bool> g_force_stop{false};
struct sigaction g_old_int {};
struct sigaction g_old_term {};
std::once_flag g_signal_once;

void chain_previous_signal(int sig, siginfo_t * info, void * context)
{
  const struct sigaction & previous = sig == SIGINT ? g_old_int : g_old_term;
  if ((previous.sa_flags & SA_SIGINFO) != 0 && previous.sa_sigaction != nullptr) {
    previous.sa_sigaction(sig, info, context);
    return;
  }
  if (previous.sa_handler != nullptr && previous.sa_handler != SIG_DFL &&
    previous.sa_handler != SIG_IGN)
  {
    previous.sa_handler(sig);
  }
}

void note_stop_signal(int sig, siginfo_t * info, void * context)
{
  const char message[] = "[x1-debug] signal stop\n";
  const ssize_t written = write(STDERR_FILENO, message, sizeof(message) - 1);
  (void)written;
  g_force_stop.store(true);
  const int fd = g_wake_socket.load();
  if (fd >= 0) {
    shutdown(fd, SHUT_RDWR);
  }
  chain_previous_signal(sig, info, context);
}

void install_stop_signals()
{
  struct sigaction action {};
  action.sa_sigaction = note_stop_signal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_SIGINFO;
  sigaction(SIGINT, &action, &g_old_int);
  sigaction(SIGTERM, &action, &g_old_term);
}

const std::string * findParameter(const HardwareParameters & parameters, const std::string & key)
{
  const auto iterator = parameters.find(key);
  return iterator == parameters.end() ? nullptr : &iterator->second;
}

bool parseInteger(const std::string & text, int minimum, int maximum, int * value)
{
  try {
    std::size_t used = 0;
    const int parsed = std::stoi(text, &used);
    if (used != text.size() || parsed < minimum || parsed > maximum) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parseDouble(const std::string & text, double * value)
{
  try {
    std::size_t used = 0;
    const double parsed = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parseBool(const std::string & text, bool * value)
{
  if (text == "1" || text == "true" || text == "True") {
    *value = true;
    return true;
  }
  if (text == "0" || text == "false" || text == "False") {
    *value = false;
    return true;
  }
  return false;
}

// 沿用原传输层的多圈换算，不改算法。
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

double nearestEquivalentPulse(
  double canonical_target, std::int32_t current, std::int32_t wrap_period)
{
  if (wrap_period <= 0) {
    return canonical_target;
  }
  const double normalized_delta = normalizePulseDelta(
    static_cast<double>(current) - canonical_target, wrap_period);
  return static_cast<double>(current) - normalized_delta;
}

class SoemLink final : public EthercatLink
{
public:
  SoemLink()
  {
    debugf("SoemLink ctor sizeof context=%zu", sizeof(context_));
    std::memset(&context_, 0, sizeof(context_));
    debugf("SoemLink ctor done");
  }

  ~SoemLink() override
  {
    shutdownToInit();
    stopEnableTrace();
  }

  Result openAndReachOp(
    const std::string & ifname,
    const std::array<bool, kCoffeeAxisCount> & torque_axis,
    const Cycle & on_cycle) override
  {
    if (opened_) {
      return Result::fail(CoreError::kTransportUnavailable, "网口已经打开");
    }
    torque_axis_ = torque_axis;
    on_cycle_ = on_cycle;
    stop_.store(false);
    {
      std::lock_guard<std::mutex> lock(wait_mu_);
      op_seen_ = false;
      failed_ = false;
      failure_detail_.clear();
    }
    debugf("open begin ifname=%s", ifname.c_str());
    // 记录实际加载的库，不能用 source 的目录或源码一致代替运行产物一致。
    Dl_info library{};
    dladdr(reinterpret_cast<const void *>(&debugf), &library);
    debugf(
      "enable_trace=v1 library=%s build=%s %s rmw=%s domain=%s",
      library.dli_fname != nullptr ? library.dli_fname : "unknown", __DATE__, __TIME__,
      std::getenv("RMW_IMPLEMENTATION") != nullptr ? std::getenv("RMW_IMPLEMENTATION") : "unset",
      std::getenv("ROS_DOMAIN_ID") != nullptr ? std::getenv("ROS_DOMAIN_ID") : "unset");
    say("● 打开网口 %s", ifname.c_str());
    g_force_stop.store(false);
    shutdown_started_.store(false);
    debugf("before ecx_init");
    if (!ecx_init(&context_, ifname.c_str())) {
      debugf("ecx_init failed");
      return Result::fail(CoreError::kTransportUnavailable, "ecx_init 失败，原始网口需要 root");
    }
    opened_ = true;
    g_wake_socket.store(context_.port.sockhandle);
    debugf("ecx_init ok sock=%d", context_.port.sockhandle);
    std::call_once(g_signal_once, install_stop_signals);
    debugf("before ecx_config_init");
    const int count = ecx_config_init(&context_);
    debugf("ecx_config_init count=%d", count);
    say(count == static_cast<int>(kCoffeeAxisCount) ?
      "✓ 找到 %d 台从站" : "✗ 从站数不是 15，读到 %d", count);
    if (count != static_cast<int>(kCoffeeAxisCount)) {
      const Result failed = Result::fail(
        CoreError::kEthercatNotReady,
        "从站数不是 15，读到 " + std::to_string(count));
      shutdownToInit();
      return failed;
    }
    active_link_ = this;
    for (int slave = 1; slave <= count; ++slave) {
      context_.slavelist[slave].PO2SOconfig = &SoemLink::pdoThunk;
    }
    debugf("before ecx_config_map_group");
    ecx_config_map_group(&context_, iomap_, 0);
    active_link_ = nullptr;
    say("✓ 15 轴过程数据已映射");
    debugf("before ecx_configdc");
    ecx_configdc(&context_);
    for (int slave = 1; slave <= count; ++slave) {
      ecx_dcsync0(&context_, static_cast<std::uint16_t>(slave), TRUE, 500000, 0);
      const bool torque = torque_axis_[static_cast<std::size_t>(slave - 1)];
      const int expect_out = torque ? static_cast<int>(sizeof(TorqueOutput)) :
        static_cast<int>(sizeof(ZeroErrOutput));
      const int expect_in = torque ? static_cast<int>(sizeof(TorqueInput)) :
        static_cast<int>(sizeof(ZeroErrInput));
      if (static_cast<int>(context_.slavelist[slave].Obytes) != expect_out ||
        static_cast<int>(context_.slavelist[slave].Ibytes) != expect_in ||
        context_.slavelist[slave].outputs == nullptr ||
        context_.slavelist[slave].inputs == nullptr)
      {
        const Result failed = Result::fail(
          CoreError::kEthercatNotReady, "从站 " + std::to_string(slave) + " 的过程数据长度不符");
        shutdownToInit();
        return failed;
      }
      if (torque) {
        torque_out_[slave] = reinterpret_cast<TorqueOutput *>(context_.slavelist[slave].outputs);
        torque_in_[slave] = reinterpret_cast<TorqueInput *>(context_.slavelist[slave].inputs);
        torque_out_[slave]->control_word = 0;
        torque_out_[slave]->target_torque = 0;
      } else {
        position_out_[slave] = reinterpret_cast<ZeroErrOutput *>(context_.slavelist[slave].outputs);
        position_in_[slave] = reinterpret_cast<ZeroErrInput *>(context_.slavelist[slave].inputs);
        position_out_[slave]->control_word = 0;
        position_out_[slave]->digital_output = 0;
        position_out_[slave]->target_position = 0;
        position_out_[slave]->target_torque = 0;
      }
    }

    // 请求 OP 的全程都发过程数据，避免同步管理器看门狗。
    startEnableTrace();
    debugf("request SAFE_OP and start cycle thread");
    context_.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&context_, 0);
    thread_done_.store(false);
    thread_ = std::thread([this]() {cycleLoop();});
    debugf("cycle thread started");
    bool saw_op = false;
    bool bringup_failed = false;
    std::string bringup_detail;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() < deadline && !stopped()) {
      std::unique_lock<std::mutex> lock(wait_mu_);
      wait_cv_.wait_for(
        lock, std::chrono::milliseconds(200), [this]() {return op_seen_ || failed_ || stopped();});
      saw_op = op_seen_;
      bringup_failed = failed_;
      bringup_detail = failure_detail_;
      if (saw_op || bringup_failed || stopped()) {
        break;
      }
    }
    debugf(
      "wait op done saw=%d failed=%d stopped=%d detail=%s",
      saw_op ? 1 : 0, bringup_failed ? 1 : 0, stopped() ? 1 : 0, bringup_detail.c_str());
    if (stopped() || bringup_failed || !saw_op) {
      const std::string detail = stopped() ? "收到中断，停止等待 OP" :
        (bringup_detail.empty() ? "15 秒内没有进入 OP" : bringup_detail);
      shutdownToInit();
      return Result::fail(CoreError::kEthercatNotReady, detail);
    }
    return Result::ok();
  }

  void requestDriveMode(std::uint8_t mode, std::uint16_t axis_mask) override
  {
    const std::uint32_t packed =
      (static_cast<std::uint32_t>(axis_mask) << 8) | static_cast<std::uint32_t>(mode);
    pending_drive_mode_.store(packed);
  }

  Result shutdownToInit() override
  {
    if (shutdown_started_.exchange(true)) {
      return Result::ok();
    }
    debugf("shutdown begin");
    stop_.store(true);
    const int fd = g_wake_socket.load();
    if (fd >= 0) {
      shutdown(fd, SHUT_RDWR);
    }
    wait_cv_.notify_all();
    const auto join_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (thread_.joinable() && !thread_done_.load() &&
      std::chrono::steady_clock::now() < join_deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (thread_.joinable()) {
      if (thread_done_.load()) {
        thread_.join();
      } else {
        debugf("cycle thread still running after 1s, detach");
        say("! 总线线程没有在 1 秒内退出，不再等待");
        thread_.detach();
        opened_ = false;
        g_wake_socket.store(-1);
        return Result::ok();
      }
    }
    if (!opened_) {
      stopEnableTrace();
      return Result::ok();
    }
    for (int slave = 1; slave <= context_.slavecount && slave <= static_cast<int>(kCoffeeAxisCount);
      ++slave)
    {
      if (torque_out_[slave] != nullptr) {
        torque_out_[slave]->target_torque = 0;
        torque_out_[slave]->control_word = 0x0006;
      }
      if (position_out_[slave] != nullptr) {
        position_out_[slave]->control_word = 0x0006;
      }
    }
    for (int n = 0; n < 20; ++n) {
      ecx_send_processdata(&context_);
      ecx_receive_processdata(&context_, EC_TIMEOUTRET);
      usleep(500);
    }
    context_.slavelist[0].state = EC_STATE_INIT | EC_STATE_ACK;
    ecx_writestate(&context_, 0);
    ecx_statecheck(&context_, 0, EC_STATE_INIT, 200000);
    g_wake_socket.store(-1);
    ecx_readstate(&context_);
    say("● 总线已关闭，状态 0x%04x", context_.slavelist[0].state);
    ecx_close(&context_);
    opened_ = false;
    std::memset(position_out_, 0, sizeof(position_out_));
    std::memset(position_in_, 0, sizeof(position_in_));
    std::memset(torque_out_, 0, sizeof(torque_out_));
    std::memset(torque_in_, 0, sizeof(torque_in_));
    // 总线已收回 INIT 并关闭，再等待日志排空；日志不能拖延原有退使能流程。
    stopEnableTrace();
    return Result::ok();
  }

  int configurePdo(ecx_contextt * context, std::uint16_t slave)
  {
    if (slave >= 1 && slave <= kCoffeeAxisCount &&
      torque_axis_[static_cast<std::size_t>(slave - 1)])
    {
      return configureTorquePdo(context, slave);
    }
    return configurePositionPdo(context, slave);
  }

private:
  void startEnableTrace()
  {
    // 此处尚未启动周期线程；日志线程保持普通调度，不继承周期线程的 FIFO 优先级。
    enable_trace_ = std::make_unique<detail::EnableTrace>();
    expected_wkc_ = context_.grouplist[0].outputsWKC * 2 + context_.grouplist[0].inputsWKC;
    trace_stop_.store(false);
    trace_thread_ = std::thread([this]() {
        sched_param priority{};
        const int rc = pthread_setschedparam(pthread_self(), SCHED_OTHER, &priority);
        debugf("enable logger SCHED_OTHER rc=%d", rc);
        detail::EnableTraceFrame frame;
        for (;;) {
          const bool stopping = trace_stop_.load(std::memory_order_acquire);
          while (enable_trace_->pop(&frame)) {
            printEnableTrace(frame);
          }
          if (stopping) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        debugf("enable trace stopped dropped=%u", enable_trace_->dropped());
      });
  }

  void stopEnableTrace()
  {
    trace_stop_.store(true, std::memory_order_release);
    if (trace_thread_.joinable()) {
      trace_thread_.join();
    }
  }

  static void printEnableTrace(const detail::EnableTraceFrame & frame)
  {
    if (std::getenv("X1_ECAT_DEBUG") == nullptr) {
      return;
    }
    std::fprintf(
      stderr,
      "[x1-enable] phase=%s tick=%llu elapsed_us=%llu summary=%d changed=0x%04x "
      "wkc=%d expected_wkc=%d min_wkc=%d bad_wkc_frames=%llu send_rc=%d "
      "nonpositive_send_frames=%llu max_cycle_gap_us=%llu dc_phase_ns=%lld dropped=%llu\n",
      frame.operational ? "op" : "bringup", static_cast<unsigned long long>(frame.tick),
      static_cast<unsigned long long>(frame.elapsed_us), frame.summary ? 1 : 0,
      frame.changed_axes, frame.wkc, frame.expected_wkc, frame.minimum_wkc,
      static_cast<unsigned long long>(frame.low_wkc_frames), frame.send_rc,
      static_cast<unsigned long long>(frame.nonpositive_send_frames),
      static_cast<unsigned long long>(frame.max_cycle_gap_us),
      static_cast<long long>(frame.dc_phase_ns), static_cast<unsigned long long>(frame.dropped));
    for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
      if (!frame.summary && (frame.changed_axes & (std::uint32_t{1} << i)) == 0) {
        continue;
      }
      const auto & feedback = frame.feedback[i];
      const auto & previous = frame.sent[i];
      const auto & submitted = frame.next[i];
      std::fprintf(
        stderr,
        "[x1-enable-axis] tick=%llu slave=%zu valid=%d sw=0x%04x state=0x%04x "
        "state_age_us=%llu raw_status_changes=%llu cw_prev_submitted=0x%04x cw_submitted=0x%04x "
        "actual=%d target_prev_submitted=%d target_submitted=%d\n",
        static_cast<unsigned long long>(frame.tick), i + 1, feedback.valid ? 1 : 0,
        feedback.status_word, feedback.status_word & 0x006f,
        static_cast<unsigned long long>(frame.state_age_us[i]),
        static_cast<unsigned long long>(frame.raw_status_changes[i]),
        previous.control_word, submitted.control_word, feedback.actual_position,
        previous.target_position, submitted.target_position);
    }
  }

  void traceCycle(
    const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
    const std::array<SlaveCommand, kCoffeeAxisCount> & command,
    bool operational, int wkc, int send_rc)
  {
    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    enable_trace_->capture(
      feedback, command, static_cast<std::uint64_t>(now_us), operational, wkc, expected_wkc_,
      send_rc, context_.DCtime % DcCycleTimer::kCycleNs);
  }

  static int writePdoEntry(
    ecx_contextt * context, std::uint16_t slave, std::uint16_t index,
    std::uint8_t subindex, int size, void * value)
  {
    // 仅给原有 SDO 写入补日志，不新增总线访问，不改变写入顺序或返回处理。
    const int rc = ecx_SDOwrite(
      context, slave, index, subindex, FALSE, size, value, EC_TIMEOUTRXM);
    if (rc <= 0) {
      say("✗ 轴 %u 过程数据写入失败，索引 0x%04x，返回 %d", slave, index, rc);
    }
    return rc;
  }

  static int pdoThunk(ecx_contextt * context, std::uint16_t slave)
  {
    if (active_link_ == nullptr) {
      return 0;
    }
    return active_link_->configurePdo(context, slave);
  }

  static int configureTorquePdo(ecx_contextt * context, std::uint16_t slave)
  {
    std::uint8_t value8 = 0;
    std::uint32_t value32 = 0;
    std::uint8_t mode = kZeroErrCyclicTorqueMode;
    std::int16_t torque = 0;
    std::uint8_t display = 0;
    int display_size = static_cast<int>(sizeof(display));
    int rc = ecx_SDOwrite(
      context, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u sdo 0x6060=0x0A rc=%d\n", slave, rc);
    }
    rc = ecx_SDOread(
      context, slave, 0x6061, 0x00, FALSE, &display_size, &display, EC_TIMEOUTRXM);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u sdo 0x6061=0x%02x rc=%d\n", slave, display, rc);
    }
    rc = ecx_SDOwrite(
      context, slave, 0x6071, 0x00, FALSE, sizeof(torque), &torque, EC_TIMEOUTRXM);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u sdo 0x6071=0 rc=%d\n", slave, rc);
    }

    writePdoEntry(context, slave, 0x1600, 0, sizeof(value8), &value8);
    value32 = kZeroErrRxControlWord;
    writePdoEntry(context, slave, 0x1600, 1, sizeof(value32), &value32);
    value32 = kZeroErrRxTargetTorque;
    writePdoEntry(context, slave, 0x1600, 2, sizeof(value32), &value32);
    value8 = 2;
    writePdoEntry(context, slave, 0x1600, 0, sizeof(value8), &value8);

    value8 = 0;
    writePdoEntry(context, slave, 0x1A00, 0, sizeof(value8), &value8);
    value32 = kZeroErrTxStatusWord;
    writePdoEntry(context, slave, 0x1A00, 1, sizeof(value32), &value32);
    value32 = kZeroErrTxActualTorque;
    writePdoEntry(context, slave, 0x1A00, 2, sizeof(value32), &value32);
    value32 = kZeroErrTxActualCurrent;
    writePdoEntry(context, slave, 0x1A00, 3, sizeof(value32), &value32);
    value32 = kZeroErrTxActualPosition;
    writePdoEntry(context, slave, 0x1A00, 4, sizeof(value32), &value32);
    value8 = 4;
    rc = writePdoEntry(context, slave, 0x1A00, 0, sizeof(value8), &value8);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u pdo map rc=%d\n", slave, rc);
    }
    return 1;
  }

  static int configurePositionPdo(ecx_contextt * context, std::uint16_t slave)
  {
    std::uint8_t value8 = 0;
    std::uint32_t value32 = 0;
    // 通信手册：切到 CSP 之前把 0x6064 抄到 0x607A，模式 0x08。轮廓速度不参与周期同步位置。
    std::int32_t actual = 0;
    int actual_size = static_cast<int>(sizeof(actual));
    int rc = ecx_SDOread(
      context, slave, 0x6064, 0x00, FALSE, &actual_size, &actual, EC_TIMEOUTRXM);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u sdo 0x6064=%d rc=%d\n", slave, actual, rc);
    }
    if (rc > 0) {
      rc = ecx_SDOwrite(
        context, slave, 0x607A, 0x00, FALSE, sizeof(actual), &actual, EC_TIMEOUTRXM);
      if (rc <= 0) {
        std::fprintf(stderr, "slave=%u sdo 0x607A=%d rc=%d\n", slave, actual, rc);
      }
    }
    std::uint8_t mode = kZeroErrCyclicPositionMode;
    rc = ecx_SDOwrite(
      context, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u sdo 0x6060=0x08 rc=%d\n", slave, rc);
    }
    std::uint8_t display = 0;
    int display_size = static_cast<int>(sizeof(display));
    rc = ecx_SDOread(
      context, slave, 0x6061, 0x00, FALSE, &display_size, &display, EC_TIMEOUTRXM);
    if (rc <= 0 || display != mode) {
      std::fprintf(stderr, "slave=%u sdo 0x6061=0x%02x rc=%d\n", slave, display, rc);
    }
    std::uint32_t rated_mnm = 0;
    int rated_size = static_cast<int>(sizeof(rated_mnm));
    rc = ecx_SDOread(
      context, slave, 0x6076, 0x00, FALSE, &rated_size, &rated_mnm, EC_TIMEOUTRXM);
    if (rc > 0 && rated_size == 2) {
      rated_mnm &= 0xFFFF;
    }
    if (rc > 0) {
      say("● 轴 %u 额定力矩 %.2f N·m", slave, static_cast<double>(rated_mnm) / 1000.0);
    } else {
      say("! 轴 %u 读不到额定力矩 0x6076", slave);
    }

    writePdoEntry(context, slave, 0x1600, 0, sizeof(value8), &value8);
    value32 = kZeroErrRxTargetPosition;
    writePdoEntry(context, slave, 0x1600, 1, sizeof(value32), &value32);
    value32 = kZeroErrRxDigitalOutput;
    writePdoEntry(context, slave, 0x1600, 2, sizeof(value32), &value32);
    value32 = kZeroErrRxControlWord;
    writePdoEntry(context, slave, 0x1600, 3, sizeof(value32), &value32);
    value32 = kZeroErrRxTargetTorque;
    writePdoEntry(context, slave, 0x1600, 4, sizeof(value32), &value32);
    value8 = 4;
    writePdoEntry(context, slave, 0x1600, 0, sizeof(value8), &value8);

    value8 = 0;
    writePdoEntry(context, slave, 0x1A00, 0, sizeof(value8), &value8);
    value32 = kZeroErrTxActualPosition;
    writePdoEntry(context, slave, 0x1A00, 1, sizeof(value32), &value32);
    value32 = kZeroErrTxDigitalInput;
    writePdoEntry(context, slave, 0x1A00, 2, sizeof(value32), &value32);
    value32 = kZeroErrTxStatusWord;
    writePdoEntry(context, slave, 0x1A00, 3, sizeof(value32), &value32);
    value32 = kZeroErrTxActualTorque;
    writePdoEntry(context, slave, 0x1A00, 4, sizeof(value32), &value32);
    value8 = 4;
    rc = writePdoEntry(context, slave, 0x1A00, 0, sizeof(value8), &value8);
    if (rc <= 0) {
      std::fprintf(stderr, "slave=%u pdo map rc=%d\n", slave, rc);
    }
    return 1;
  }

  bool stopped() const
  {
    return stop_.load() || g_force_stop.load();
  }

  void cycleLoop()
  {
    struct MarkDone
    {
      std::atomic<bool> * flag;
      ~MarkDone() {flag->store(true);}
    } mark_done{&thread_done_};
    raiseCycleThreadPriority();
    DcCycleTimer timer;
    timer.start();
    bool safeop_seen = false;
    bool op_requested = false;
    bool saw_op = false;
    for (int cycle = 0; !stopped() && cycle < 12000; ++cycle) {
      const int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTRET);
      std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
      int ready = 0;
      for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
        SlaveFeedback & item = feedback[static_cast<std::size_t>(slave - 1)];
        if (torque_in_[slave] != nullptr) {
          item.status_word = torque_in_[slave]->status_word;
          item.actual_position = torque_in_[slave]->actual_position;
          item.actual_torque = torque_in_[slave]->actual_torque;
          item.actual_current = torque_in_[slave]->actual_current;
        } else if (position_in_[slave] != nullptr) {
          item.status_word = position_in_[slave]->status_word;
          item.actual_position = position_in_[slave]->actual_position;
          item.actual_torque = position_in_[slave]->actual_torque;
        }
        item.valid = wkc > 0 && item.status_word != 0;
        if (item.valid) {
          ++ready;
        }
      }
      std::array<SlaveCommand, kCoffeeAxisCount> command{};
      if (on_cycle_) {
        on_cycle_(feedback, &command);
      }
      for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
        const SlaveCommand & item = command[static_cast<std::size_t>(slave - 1)];
        if (torque_out_[slave] != nullptr) {
          torque_out_[slave]->control_word = item.control_word;
          torque_out_[slave]->target_torque = item.target_torque;
        } else if (position_out_[slave] != nullptr) {
          position_out_[slave]->control_word = item.control_word;
          position_out_[slave]->digital_output = 0;
          position_out_[slave]->target_position = item.target_position;
          position_out_[slave]->target_torque = item.target_torque;
        }
      }
      const int send_rc = ecx_send_processdata(&context_);
      traceCycle(feedback, command, false, wkc, send_rc);
      if (cycle == 0 || (cycle % 2000) == 0) {
        say("● 等待从站进入运行，已有反馈 %d/15", ready);
      }
      if ((cycle % 20) == 0) {
        const std::uint16_t want = op_requested ? EC_STATE_OPERATIONAL : EC_STATE_SAFE_OP;
        ecx_statecheck(&context_, 0, want, 1000);
      }
      if (!safeop_seen && (context_.slavelist[0].state & 0x0f) >= EC_STATE_SAFE_OP) {
        safeop_seen = true;
        say("● 从站进入安全运行，%d/15 台有反馈", ready);
      }
      if (safeop_seen && !op_requested && ready == static_cast<int>(kCoffeeAxisCount)) {
        context_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&context_, 0);
        op_requested = true;
        say("● 请求 15 台从站进入运行");
      }
      if (op_requested && context_.slavelist[0].state == EC_STATE_OPERATIONAL) {
        say("✓ 15 台从站已进入运行");
        saw_op = true;
        {
          std::lock_guard<std::mutex> lock(wait_mu_);
          op_seen_ = true;
        }
        wait_cv_.notify_all();
        break;
      }
      timer.wait(context_.DCtime);
    }
    if (!saw_op && !stopped()) {
      ecx_readstate(&context_);
      std::ostringstream stream;
      stream << "bus state=0x" << std::hex << context_.slavelist[0].state;
      for (int slave = 1; slave <= context_.slavecount; ++slave) {
        stream << " slave" << std::dec << slave << "=0x" << std::hex
               << context_.slavelist[slave].state << "/0x"
               << context_.slavelist[slave].ALstatuscode;
      }
      {
        std::lock_guard<std::mutex> lock(wait_mu_);
        failed_ = true;
        failure_detail_ = stream.str();
      }
      wait_cv_.notify_all();
      return;
    }
    // OP 之后继续按同一节拍收发，直到关闭。
    say("● 周期通信已保持");
    int steady_cycle = 0;
    long told_overruns = 0;
    int last_overrun_log = 0;
    timer.resetStats();
    while (!stopped()) {
      const int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTRET);
      std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
      for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
        SlaveFeedback & item = feedback[static_cast<std::size_t>(slave - 1)];
        if (torque_in_[slave] != nullptr) {
          item.status_word = torque_in_[slave]->status_word;
          item.actual_position = torque_in_[slave]->actual_position;
          item.actual_torque = torque_in_[slave]->actual_torque;
          item.actual_current = torque_in_[slave]->actual_current;
        } else if (position_in_[slave] != nullptr) {
          item.status_word = position_in_[slave]->status_word;
          item.actual_position = position_in_[slave]->actual_position;
          item.actual_torque = position_in_[slave]->actual_torque;
        }
        item.valid = wkc > 0;
      }
      if (timer.overruns() != told_overruns && steady_cycle - last_overrun_log >= 2000) {
        told_overruns = timer.overruns();
        last_overrun_log = steady_cycle;
        say(
          "! 周期累计迟到 %ld 次，最晚 %lld 微秒",
          told_overruns, static_cast<long long>(timer.maxLateNs() / 1000));
      }
      ++steady_cycle;
      std::array<SlaveCommand, kCoffeeAxisCount> command{};
      if (on_cycle_) {
        on_cycle_(feedback, &command);
      }
      for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
        const SlaveCommand & item = command[static_cast<std::size_t>(slave - 1)];
        if (torque_out_[slave] != nullptr) {
          torque_out_[slave]->control_word = item.control_word;
          torque_out_[slave]->target_torque = item.target_torque;
        } else if (position_out_[slave] != nullptr) {
          position_out_[slave]->control_word = item.control_word;
          position_out_[slave]->digital_output = 0;
          position_out_[slave]->target_position = item.target_position;
          position_out_[slave]->target_torque = item.target_torque;
        }
      }
      const int send_rc = ecx_send_processdata(&context_);
      traceCycle(feedback, command, true, wkc, send_rc);
      applyPendingDriveMode();
      timer.wait(context_.DCtime);
    }
  }

  void applyPendingDriveMode()
  {
    const std::uint32_t packed = pending_drive_mode_.exchange(0);
    if (packed == 0) {
      return;
    }
    const int mode = static_cast<int>(packed & 0xFF);
    const std::uint16_t mask = static_cast<std::uint16_t>(packed >> 8);
    std::uint8_t value = static_cast<std::uint8_t>(mode);
    int failed = 0;
    int written = 0;
    for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
      if ((mask & (1u << (slave - 1))) == 0) {
        continue;
      }
      ++written;
      const int rc = ecx_SDOwrite(
        &context_, static_cast<std::uint16_t>(slave), 0x6060, 0x00, FALSE,
        sizeof(value), &value, EC_TIMEOUTRXM);
      if (rc <= 0) {
        ++failed;
      }
    }
    if (failed == 0) {
      say(mode == static_cast<int>(kZeroErrCyclicTorqueMode) ?
        "✓ 右臂 %d 轴已切到周期同步扭矩" : "✓ 右臂 %d 轴已切回周期同步位置", written);
    } else {
      say("✗ %d 台运行模式写入失败，目标 0x%02x", failed, mode);
    }
  }

  ecx_contextt context_{};
  std::atomic<std::uint32_t> pending_drive_mode_{0};
  std::uint8_t iomap_[8192]{};
  bool opened_{false};
  std::array<bool, kCoffeeAxisCount> torque_axis_{};
  Cycle on_cycle_;
  std::thread thread_;
  std::unique_ptr<detail::EnableTrace> enable_trace_;
  std::thread trace_thread_;
  std::atomic<bool> trace_stop_{false};
  int expected_wkc_{0};
  std::atomic<bool> stop_{false};
  std::atomic<bool> thread_done_{true};
  std::atomic<bool> shutdown_started_{false};
  std::mutex wait_mu_;
  std::condition_variable wait_cv_;
  bool op_seen_{false};
  bool failed_{false};
  std::string failure_detail_;
  ZeroErrOutput * position_out_[kCoffeeAxisCount + 1]{};
  ZeroErrInput * position_in_[kCoffeeAxisCount + 1]{};
  TorqueOutput * torque_out_[kCoffeeAxisCount + 1]{};
  TorqueInput * torque_in_[kCoffeeAxisCount + 1]{};
  static SoemLink * active_link_;
};

SoemLink * SoemLink::active_link_ = nullptr;

}  // namespace

// 与 cycleLoop 里的 usleep(500) 一致。CSP 每个通信周期写一次 0x607A。
constexpr double kEthercatCycleSec = 5.0e-4;
// 手册：同一台关节下使能到再使能的间隔应大于 300 ms。500 μs 一个周期。
constexpr int kEnableGapCycles = 600;
// 准备接通先保持关机后的接通字 100 ms。仍不前进就按手册第 3 步发 0x000F。
constexpr int kSwitchOnHoldCycles = 200;
// 使能字也停 0.5 秒才关电压重走。关电压最多 0.5 秒，直到变成禁止接通。
constexpr int kEnableStuckCycles = 1000;
constexpr int kEnableDisableCycles = 1000;

static int velocityStep(double pulses_per_unit, double velocity_rad_s)
{
  const double counts = std::fabs(pulses_per_unit) * velocity_rad_s * kEthercatCycleSec;
  if (!(counts > 0.0) || !std::isfinite(counts)) {
    return 1;
  }
  const int step = static_cast<int>(std::ceil(counts));
  return step < 1 ? 1 : step;
}

static void advanceToward(std::int32_t * current, std::int32_t goal, int step)
{
  if (current == nullptr || step < 1) {
    return;
  }
  const std::int64_t delta =
    static_cast<std::int64_t>(goal) - static_cast<std::int64_t>(*current);
  if (delta == 0) {
    return;
  }
  const std::int64_t magnitude = delta > 0 ? delta : -delta;
  const std::int64_t applied = magnitude < step ? magnitude : static_cast<std::int64_t>(step);
  *current = static_cast<std::int32_t>(
    static_cast<std::int64_t>(*current) + (delta > 0 ? applied : -applied));
}

SoemEthercatTransport::SoemEthercatTransport()
: SoemEthercatTransport(std::make_unique<SoemLink>())
{
}

SoemEthercatTransport::SoemEthercatTransport(std::unique_ptr<EthercatLink> link)
: link_(std::move(link))
{
}

SoemEthercatTransport::~SoemEthercatTransport()
{
  if (active_ && link_) {
    link_->shutdownToInit();
  }
}

Result SoemEthercatTransport::configure(const HardwareParameters & parameters)
{
  HardwareProfile profile = HardwareProfile::kCoffee15NoSlide;
  if (const std::string * value = findParameter(parameters, "hardware_profile")) {
    const auto parsed = parseHardwareProfile(*value);
    if (!parsed) {
      return Result::fail(CoreError::kInvalidConfiguration, "hardware_profile 必须为 coffee_15_no_slide");
    }
    profile = *parsed;
  }
  layout_ = &hardwareLayout(profile);
  const std::string * axis_count = findParameter(parameters, "axis_count");
  int parsed_axis_count = 0;
  if (axis_count == nullptr || !parseInteger(*axis_count, 1, 64, &parsed_axis_count) ||
    parsed_axis_count != static_cast<int>(layout_->physical_axis_count))
  {
    return Result::fail(
      CoreError::kInvalidConfiguration,
      "axis_count 必须与 hardware_profile 一致，expected=" +
      std::to_string(layout_->physical_axis_count));
  }
  const std::string * ifname = findParameter(parameters, "ethercat_ifname");
  if (ifname == nullptr || ifname->empty()) {
    return Result::fail(CoreError::kInvalidConfiguration, "缺少硬件参数: ethercat_ifname");
  }
  ifname_ = *ifname;
  if (const std::string * value = findParameter(parameters, "drives_enabled")) {
    if (!parseBool(*value, &drives_enabled_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "drives_enabled 必须为 true/false 或 1/0");
    }
  }
  gravity_ready_ = false;
  if (const std::string * urdf = findParameter(parameters, "gravity_urdf")) {
    std::string error;
    if (!gravity_.load(*urdf, &error)) {
      return Result::fail(CoreError::kInvalidConfiguration, error);
    }
    gravity_ready_ = true;
  }
  if (const std::string * rated = findParameter(parameters, "rated_torque_nm")) {
    if (!parseDouble(*rated, &rated_torque_nm_) || !(rated_torque_nm_ > 0.0)) {
      return Result::fail(CoreError::kInvalidConfiguration, "rated_torque_nm 必须为正数");
    }
  }
  if (const std::string * scale = findParameter(parameters, "gravity_scale")) {
    if (!parseDouble(*scale, &gravity_scale_) || !std::isfinite(gravity_scale_)) {
      return Result::fail(CoreError::kInvalidConfiguration, "gravity_scale 必须是有限数值");
    }
  }
  if (gravity_ready_) {
    if (const std::string * yaml = findParameter(parameters, "right_arm_gravity_yaml")) {
      std::array<double, 7> mass_scale{};
      mass_scale.fill(1.0);
      std::array<std::array<double, 3>, 7> com_delta{};
      std::ifstream input(*yaml);
      if (!input) {
        say("! 没有右臂重力修正文件，仍用 CAD：%s", yaml->c_str());
      } else {
        std::string line;
        while (std::getline(input, line)) {
          if (line.empty() || line[0] == '#') {
            continue;
          }
          std::istringstream stream(line);
          std::string key;
          stream >> key;
          if (!key.empty() && key.back() == ':') {
            key.pop_back();
          }
          double values[7]{};
          for (double & value : values) {
            stream >> value;
          }
          if (key == "mass_scale") {
            for (int slot = 0; slot < 7; ++slot) {
              mass_scale[static_cast<std::size_t>(slot)] = values[slot];
            }
          } else if (key == "com_dx" || key == "com_dy" || key == "com_dz") {
            const int axis = key == "com_dx" ? 0 : key == "com_dy" ? 1 : 2;
            for (int slot = 0; slot < 7; ++slot) {
              com_delta[static_cast<std::size_t>(slot)][axis] = values[slot];
            }
          }
        }
        if (gravity_.setRightArmCorrection(mass_scale, com_delta)) {
          say("✓ 已读右臂质量倍率和质心偏移");
        }
      }
    }
  }
  torque_axis_.fill(false);
  if (const std::string * axes = findParameter(parameters, "zero_torque_axes")) {
    std::stringstream stream(*axes);
    std::string token;
    while (std::getline(stream, token, ',')) {
      if (token.empty()) {
        continue;
      }
      int slave = 0;
      if (!parseInteger(token, 1, static_cast<int>(kCoffeeAxisCount), &slave)) {
        return Result::fail(CoreError::kInvalidConfiguration, "zero_torque_axes 含有无效从站号: " + token);
      }
      torque_axis_[static_cast<std::size_t>(slave - 1)] = true;
    }
  }
  const auto & descriptors = coffeeAxisDescriptors();
  for (std::size_t index = 0; index < descriptors.size(); ++index) {
    if (!isPhysicalJoint(*layout_, index)) {
      continue;
    }
    const std::string joint(descriptors[index].joint_name);
    const std::string scale_key = "pulses_per_unit." + joint;
    const std::string offset_key = "zero_offset_pulse." + joint;
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
    if (const std::string * model_zero = findParameter(parameters, "model_zero." + joint)) {
      if (!parseDouble(*model_zero, &calibration_[index].model_zero)) {
        return Result::fail(CoreError::kInvalidConfiguration, "无效的标定参数: model_zero." + joint);
      }
    }
    const std::string * wrap = findParameter(parameters, "wrap_period_pulse." + joint);
    int wrap_pulse = 0;
    if (wrap == nullptr || !parseInteger(*wrap, 0, std::numeric_limits<int>::max(), &wrap_pulse)) {
      return Result::fail(
        CoreError::kInvalidConfiguration, "缺少或无效的标定参数: wrap_period_pulse." + joint);
    }
    calibration_[index].wrap_period_pulse = wrap_pulse;
    max_velocity_rad_s_[index] = 1.0;
    const std::string velocity_key = "max_velocity." + joint;
    if (const std::string * velocity = findParameter(parameters, velocity_key)) {
      double parsed_velocity = 0.0;
      if (!parseDouble(*velocity, &parsed_velocity) || !(parsed_velocity > 0.0)) {
        return Result::fail(CoreError::kInvalidConfiguration, "无效的速度上限: " + velocity_key);
      }
      max_velocity_rad_s_[index] = parsed_velocity;
    }
    velocity_step_[index] = velocityStep(
      calibration_[index].pulses_per_unit, max_velocity_rad_s_[index]);
    active_step_[index] = velocity_step_[index];
  }
  motion_armed_ = false;
  configured_ = true;
  closed_ = false;
  debugf(
    "configure ok ifname=%s drives_enabled=%d", ifname_.c_str(), drives_enabled_ ? 1 : 0);
  return Result::ok();
}

Result SoemEthercatTransport::activate()
{
  if (!configured_ || link_ == nullptr) {
    return Result::fail(CoreError::kInvalidConfiguration, "EtherCAT 传输层尚未完成配置");
  }
  debugf("activate begin");
  shutdown_outputs_ = false;
  motion_armed_ = false;
  path_active_ = false;
  path_.clear();
  path_cycle_ = 0;
  enable_ready_cycles_.fill(0);
  enable_backoff_cycles_.fill(0);
  enable_retries_.fill(0);
  enable_phase_.fill(0);
  held_.fill(false);
  const Result opened = link_->openAndReachOp(
    ifname_, torque_axis_,
    [this](const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
      std::array<SlaveCommand, kCoffeeAxisCount> * command) {
      onCycle(feedback, command);
    });
  if (!opened) {
    debugf("activate failed: %s", opened.detail.c_str());
    return opened;
  }
  debugf("activate reached OP");
  if (dynamic_cast<SoemLink *>(link_.get()) != nullptr) {
    servo_ready_delay_ms_ = 5000;
    servo_wait_begin_ = std::chrono::steady_clock::now();
    servo_wait_logged_sec_ = -1;
    servo_wait_done_logged_ = false;
  }
  active_ = true;
  closed_ = false;
  return Result::ok();
}

Result SoemEthercatTransport::deactivate()
{
  if (link_) {
    link_->shutdownToInit();
  }
  active_ = false;
  closed_ = true;
  motion_armed_ = false;
  return Result::ok();
}

Result SoemEthercatTransport::read(HardwareSnapshot * snapshot)
{
  if (snapshot == nullptr) {
    return Result::fail(CoreError::kReadFailed, "快照指针为空");
  }
  if (!active_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  std::array<bool, kCoffeeAxisCount> held{};
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    feedback = latest_;
    held = held_;
  }
  snapshot->sampled_at = std::chrono::steady_clock::now();
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    AxisState & axis = snapshot->axes[index];
    const SlaveFeedback & item = feedback[index];
    if (!item.valid || !held[index]) {
      return Result::fail(CoreError::kReadFailed, "从站位置尚未锁存");
    }
    axis.encoder_pulse_raw = item.actual_position;
    axis.encoder_valid = true;
    axis.position = pulseToPosition(index, item.actual_position);
    axis.position_valid = std::isfinite(axis.position);
    if (has_previous_position_) {
      axis.velocity = axis.position - previous_position_[index];
    } else {
      axis.velocity = 0.0;
    }
    axis.velocity_valid = std::isfinite(axis.velocity);
    previous_position_[index] = axis.position;
    axis.status_bits = item.status_word;
    const bool operation_enabled = (item.status_word & 0x006F) == 0x0027;
    axis.enabled = operation_enabled;
    axis.alarm = (item.status_word & 0x0008) != 0;
    axis.moving = false;
    axis.follow_error = false;
    axis.emergency_input_active = false;
    axis.torque_permille = item.actual_torque;
    axis.torque_valid = true;
    if (torque_axis_[index]) {
      axis.current_permille = item.actual_current;
      axis.current_valid = true;
    }
  }
  has_previous_position_ = true;
  return Result::ok();
}

Result SoemEthercatTransport::write(const PositionCommandFrame & command)
{
  if (!active_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  // 轨迹控制器在允许运动之前就会周期写入。这里保持锁存位置，避免把节点打成错误。
  if (!motion_armed_) {
    return Result::ok();
  }
  if (!drives_enabled_) {
    return Result::fail(CoreError::kWriteFailed, "驱动器未使能，不能下发位置目标");
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (mit_active_) {
    return Result::ok();
  }
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    if (torque_axis_[index]) {
      return Result::fail(CoreError::kWriteFailed, "零力矩轴不能接收位置目标");
    }
    if (!held_[index]) {
      return Result::fail(CoreError::kReadFailed, "位置尚未锁存");
    }
    double pulse = 0.0;
    const Result converted = positionToPulse(
      index, command.positions[index], held_pulse_[index], &pulse);
    if (!converted) {
      return converted;
    }
    goal_pulse_[index] = static_cast<std::int32_t>(std::llround(pulse));
    active_step_[index] = velocity_step_[index] > 0 ? velocity_step_[index] : 1;
  }
  return Result{CoreError::kOk, "已更新周期同步位置目标"};
}

Result SoemEthercatTransport::stop(bool)
{
  std::lock_guard<std::mutex> lock(sample_mu_);
  motion_armed_ = false;
  path_active_ = false;
  path_.clear();
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    goal_pulse_[index] = held_pulse_[index];
  }
  shutdown_outputs_ = true;
  return Result::ok();
}

Result SoemEthercatTransport::startMotion()
{
  if (!active_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  if (!drives_enabled_) {
    return Result::fail(CoreError::kWriteFailed, "驱动器未使能，不能允许运动");
  }
  for (bool torque : torque_axis_) {
    if (torque) {
      return Result::fail(CoreError::kWriteFailed, "零力矩轴不能进入周期同步位置");
    }
  }
  if (mit_active_) {
    return Result::fail(CoreError::kWriteFailed, "MIT 占用中，先退出再允许位置运动");
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  // 只打开跟随。目标仍是锁存位置，下一次 write 才把 MoveIt 的位置写进 0x607A。
  motion_armed_ = true;
  return Result{CoreError::kOk, "已允许周期同步位置，后续位置指令按速度上限写入 0x607A"};
}

Result SoemEthercatTransport::enterZeroTorque()
{
  return requestZeroTorque();
}

Result SoemEthercatTransport::requestZeroTorque()
{
  if (!active_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  bool any = false;
  bool all = true;
  for (bool torque : torque_axis_) {
    any = any || torque;
    all = all && torque;
  }
  if (any && all) {
    return Result{CoreError::kOk, "15 轴已是周期同步扭矩，目标扭矩为 0"};
  }
  if (any) {
    return Result{CoreError::kOk, "zero_torque_axes 中的轴已是目标扭矩 0"};
  }
  return Result::fail(
    CoreError::kInvalidConfiguration,
    "零力矩要在激活前用 zero_torque_axes 映射 0x6071。当前过程数据是周期同步位置");
}

Result SoemEthercatTransport::engageBrakes()
{
  return requestEngageBrakes();
}

Result SoemEthercatTransport::requestEngageBrakes()
{
  shutdown_outputs_ = true;
  return Result{CoreError::kOk, "已写控制字 0x0006，制动器由关节在退使能时自己操作"};
}

Result SoemEthercatTransport::requestMovePvt(
  const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
  double duration_sec)
{
  if (waypoints.empty()) {
    return Result::fail(CoreError::kInvalidConfiguration, "没有位置目标");
  }
  if (!(duration_sec > 0.0) || !std::isfinite(duration_sec)) {
    return Result::fail(CoreError::kInvalidConfiguration, "位置段时间必须为正数");
  }
  const double slice = duration_sec / static_cast<double>(waypoints.size());
  return requestMovePvtSegments(
    waypoints, std::vector<double>(waypoints.size(), slice));
}

Result SoemEthercatTransport::requestMovePvtSegments(
  const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
  const std::vector<double> & segment_sec)
{
  return installPath(waypoints, segment_sec);
}

Result SoemEthercatTransport::installPath(
  const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
  const std::vector<double> & segment_sec)
{
  if (!active_) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  if (!drives_enabled_) {
    return Result::fail(CoreError::kWriteFailed, "驱动器未使能，不能下发位置目标");
  }
  if (waypoints.empty() || waypoints.size() != segment_sec.size()) {
    return Result::fail(CoreError::kInvalidConfiguration, "路点和段时间数量不一致");
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (mit_active_) {
    return Result::fail(CoreError::kWriteFailed, "MIT 占用中，先按 n 回到位置模式");
  }
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    if (torque_axis_[index]) {
      return Result::fail(CoreError::kWriteFailed, "零力矩轴不能接收位置目标");
    }
    if (!held_[index]) {
      return Result::fail(CoreError::kReadFailed, "位置尚未锁存");
    }
  }
  std::vector<CoordinatedKnot> path;
  path.reserve(waypoints.size() + 1);
  CoordinatedKnot start;
  start.time_sec = 0.0;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    start.pulse[index] = static_cast<double>(held_pulse_[index]);
  }
  path.push_back(start);
  double time_sec = 0.0;
  for (std::size_t point = 0; point < waypoints.size(); ++point) {
    if (!(segment_sec[point] > 0.0) || !std::isfinite(segment_sec[point])) {
      return Result::fail(CoreError::kInvalidConfiguration, "每一段时间必须为正数");
    }
    time_sec += segment_sec[point];
    CoordinatedKnot knot;
    knot.time_sec = time_sec;
    for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
      double pulse = 0.0;
      const Result converted = positionToPulse(
        index, waypoints[point][index], held_pulse_[index], &pulse);
      if (!converted) {
        return converted;
      }
      knot.pulse[index] = pulse;
    }
    path.push_back(knot);
  }
  path_ = std::move(path);
  path_cycle_ = 0;
  path_active_ = true;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    active_step_[index] = velocity_step_[index] > 0 ? velocity_step_[index] : 1;
    goal_pulse_[index] = held_pulse_[index];
  }
  return Result{CoreError::kOk, "已装入多轴协同插补"};
}

void SoemEthercatTransport::samplePath()
{
  if (!path_active_ || path_.size() < 2) {
    return;
  }
  ++path_cycle_;
  const double time_sec = static_cast<double>(path_cycle_) * kEthercatCycleSec;
  const CoordinatedKnot & last = path_.back();
  if (time_sec >= last.time_sec) {
    for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
      goal_pulse_[index] = static_cast<std::int32_t>(std::llround(last.pulse[index]));
    }
    path_active_ = false;
    return;
  }
  std::size_t upper = 1;
  while (upper + 1 < path_.size() && path_[upper].time_sec < time_sec) {
    ++upper;
  }
  const CoordinatedKnot & begin = path_[upper - 1];
  const CoordinatedKnot & end = path_[upper];
  const double span = end.time_sec - begin.time_sec;
  const double ratio = span > 0.0 ? (time_sec - begin.time_sec) / span : 1.0;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const double pulse = begin.pulse[index] +
      (end.pulse[index] - begin.pulse[index]) * ratio;
    goal_pulse_[index] = static_cast<std::int32_t>(std::llround(pulse));
  }
}

bool SoemEthercatTransport::controllerClosed() const
{
  return closed_;
}

void SoemEthercatTransport::onCycle(
  const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
  std::array<SlaveCommand, kCoffeeAxisCount> * command)
{
  if (command == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  // 15 轴共用同一个时刻。本周期的目标是插补点，速度上限只防止单周期跳变。
  samplePath();
  if (mit_active_ && gravity_ready_ && rated_torque_nm_ > 0.0) {
    std::array<double, kCoffeeAxisCount> q{};
    for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
      if (feedback[index].valid) {
        q[index] = pulseToPosition(index, feedback[index].actual_position);
      }
    }
    std::array<double, kCoffeeAxisCount> tau{};
    if (gravity_.torques(q, &tau)) {
      for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
        mit_gravity_permille_[index] =
          gravity_scale_ * tau[index] / rated_torque_nm_ * 1000.0;
      }
    }
  }
  bool servo_waiting = false;
  if (servo_ready_delay_ms_ > 0) {
    const int elapsed_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - servo_wait_begin_).count());
    if (elapsed_ms < servo_ready_delay_ms_) {
      servo_waiting = true;
      const int remain_sec = (servo_ready_delay_ms_ - elapsed_ms + 999) / 1000;
      if (remain_sec != servo_wait_logged_sec_) {
        servo_wait_logged_sec_ = remain_sec;
        say("● 伺服准备中，还剩 %d 秒，暂不接通", remain_sec);
        std::fflush(stderr);
      }
    } else if (!servo_wait_done_logged_) {
      servo_wait_done_logged_ = true;
      say("● 伺服已准备好，开始接通");
      std::fflush(stderr);
    }
  }
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const SlaveFeedback & item = feedback[index];
    const bool operation_enabled = item.valid && (item.status_word & 0x006F) == 0x0027;
    if (item.valid && !held_[index]) {
      held_[index] = true;
      held_pulse_[index] = item.actual_position;
      goal_pulse_[index] = item.actual_position;
    }
    // 接通前每个周期把 0x607A 对齐当前 0x6064。只锁第一次的话，读到 0 之后驱动器会停在 0x1231。
    if (item.valid && held_[index] && !torque_axis_[index] && !operation_enabled && !shutdown_outputs_) {
      held_pulse_[index] = item.actual_position;
      goal_pulse_[index] = item.actual_position;
    }
    latest_[index] = item;
    SlaveCommand & out = (*command)[index];
    out.target_position = held_[index] ? held_pulse_[index] : 0;
    out.target_torque = 0;
    if (shutdown_outputs_ || !held_[index]) {
      out.control_word = shutdown_outputs_ ? 0x0006 : 0x0000;
      continue;
    }
    if (!drives_enabled_) {
      out.control_word = 0x0000;
      continue;
    }
    // 手册 5.1.1：上电后伺服还没准备好时不写接通，只保持关机字。
    if (!torque_axis_[index] && servo_waiting) {
      out.control_word = 0x0006;
      continue;
    }
    // 准备接通不前进时，先关电压回到禁止接通，再关机满 300 ms，然后重新接通。
    // 不设次数上限，上电检查的 60 秒内会一直重试。已经使能的轴不参与。
    const std::uint16_t masked_state = item.status_word & 0x006F;
    const bool ready_to_switch_on = item.valid && masked_state == 0x0021;
    if (!torque_axis_[index] && enable_phase_[index] == 1) {
      out.control_word = 0x0000;
      ++enable_backoff_cycles_[index];
      const bool switch_on_disabled = item.valid && masked_state == 0x0040;
      if (switch_on_disabled || enable_backoff_cycles_[index] >= kEnableDisableCycles) {
        enable_phase_[index] = 2;
        enable_backoff_cycles_[index] = 0;
        say("! 轴 %zu 已关电压，等待 300 ms 后再接通", index + 1);
      }
      continue;
    }
    if (!torque_axis_[index] && enable_phase_[index] == 2) {
      out.control_word = 0x0006;
      ++enable_backoff_cycles_[index];
      if (enable_backoff_cycles_[index] >= kEnableGapCycles) {
        enable_phase_[index] = 0;
        enable_backoff_cycles_[index] = 0;
        enable_ready_cycles_[index] = 0;
        say("● 轴 %zu 第 %d 次重新接通", index + 1, enable_retries_[index]);
      }
      continue;
    }
    if (!torque_axis_[index] && ready_to_switch_on) {
      ++enable_ready_cycles_[index];
      // 手册 5.5：0x0007 之后应发 0x000F。有的关节不把状态改成「已接通」，只认使能字。
      if (enable_ready_cycles_[index] > kSwitchOnHoldCycles &&
        enable_ready_cycles_[index] <= kSwitchOnHoldCycles + kEnableStuckCycles)
      {
        if (enable_ready_cycles_[index] == kSwitchOnHoldCycles + 1) {
          say("● 轴 %zu 仍是准备接通，按手册改发使能字", index + 1);
        }
        out.control_word = 0x000F;
        continue;
      }
      if (enable_ready_cycles_[index] > kSwitchOnHoldCycles + kEnableStuckCycles) {
        enable_phase_[index] = 1;
        enable_backoff_cycles_[index] = 0;
        enable_ready_cycles_[index] = 0;
        ++enable_retries_[index];
        out.control_word = 0x0000;
        say(
          "! 轴 %zu 使能字也停在准备接通，第 %d 次重试，状态 0x%04x",
          index + 1, enable_retries_[index], item.status_word);
        continue;
      }
    } else if (!torque_axis_[index]) {
      enable_ready_cycles_[index] = 0;
    }
    if (!torque_axis_[index] && operation_enabled) {
      const int step = active_step_[index] > 0 ? active_step_[index] : 1;
      advanceToward(&held_pulse_[index], goal_pulse_[index], step);
      out.target_position = held_pulse_[index];
    }
    out.control_word = torque_axis_[index] ?
      zeroErrTorqueControlWord(item.status_word) :
      zeroErrCspControlWord(item.status_word);
    out.target_torque = 0;
    // MIT 时位置目标始终贴着编码器，切回 CSP 才不会猛拉。扭矩由主站刚度律给出。
    const bool mit_axis = mit_active_ && (mit_mask_ & (1u << index)) != 0;
    if (mit_axis && !torque_axis_[index] && item.valid && operation_enabled) {
      held_pulse_[index] = item.actual_position;
      goal_pulse_[index] = item.actual_position;
      out.target_position = item.actual_position;
      out.target_torque = mitTorque(index, item.actual_position);
    }
  }
}

std::int16_t SoemEthercatTransport::mitTorque(std::size_t index, std::int32_t actual_pulse)
{
  const double measured = pulseToPosition(index, actual_pulse);
  const double desired = pulseToPosition(index, mit_hold_pulse_[index]);
  double velocity = 0.0;
  if (mit_have_sample_[index]) {
    velocity = (measured - mit_prev_rad_[index]) / kEthercatCycleSec;
  }
  mit_prev_rad_[index] = measured;
  mit_have_sample_[index] = true;
  mit_velocity_[index] += 0.05 * (velocity - mit_velocity_[index]);
  const double error = desired - measured;
  double command =
    mit_kp_ * error - mit_kd_ * mit_velocity_[index] + mit_gravity_permille_[index];
  if (command > mit_limit_) {
    command = mit_limit_;
  }
  if (command < -mit_limit_) {
    command = -mit_limit_;
  }
  return static_cast<std::int16_t>(std::lround(command));
}

Result SoemEthercatTransport::enterMitHold()
{
  if (!active_ || link_ == nullptr) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  for (bool torque : torque_axis_) {
    if (torque) {
      return Result::fail(CoreError::kInvalidConfiguration, "已有轴用纯扭矩映射启动，不能再切 MIT");
    }
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (!drives_enabled_) {
    return Result::fail(CoreError::kWriteFailed, "驱动器未使能，不能进入 MIT");
  }
  const auto & descriptors = coffeeAxisDescriptors();
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const SlaveFeedback & item = latest_[index];
    if (!item.valid || (item.status_word & 0x006F) != 0x0027) {
      return Result::fail(
        CoreError::kWriteFailed,
        std::string(descriptors[index].joint_name) + " 还没使能，不能进入 MIT");
    }
  }
  path_active_ = false;
  path_.clear();
  motion_armed_ = false;
  mit_have_sample_.fill(false);
  mit_velocity_.fill(0.0);
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    mit_hold_pulse_[index] = latest_[index].actual_position;
    held_pulse_[index] = latest_[index].actual_position;
    goal_pulse_[index] = latest_[index].actual_position;
  }
  mit_active_ = true;
  link_->requestDriveMode(kZeroErrCyclicTorqueMode, mit_mask_);
  return Result{
    CoreError::kOk,
    gravity_ready_ ?
    "只松开右臂。平衡点是按下 m 这一刻的角度，松手应回到那里。请扶着右臂" :
    "只松开右臂，左臂和腰仍按位置保持。请扶着右臂"};
}

Result SoemEthercatTransport::exitMitHold()
{
  if (!active_ || link_ == nullptr) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  mit_active_ = false;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    if (!latest_[index].valid) {
      continue;
    }
    held_pulse_[index] = latest_[index].actual_position;
    goal_pulse_[index] = latest_[index].actual_position;
  }
  link_->requestDriveMode(kZeroErrCyclicPositionMode, mit_mask_);
  return Result{CoreError::kOk, "右臂已回到位置保持"};
}

double SoemEthercatTransport::pulseToPosition(std::size_t index, std::int32_t pulse) const
{
  const AxisCalibration & item = calibration_[index];
  const double delta = normalizePulseDelta(
    static_cast<double>(pulse) - item.zero_offset_pulse, item.wrap_period_pulse);
  return item.model_zero + delta / item.pulses_per_unit;
}

Result SoemEthercatTransport::positionToPulse(
  std::size_t index, double position, std::int32_t current_pulse, double * pulse) const
{
  if (pulse == nullptr || !std::isfinite(position)) {
    return Result::fail(CoreError::kCommandNotFinite, "位置命令不是有限数值");
  }
  const AxisCalibration & item = calibration_[index];
  const double canonical = item.zero_offset_pulse +
    (position - item.model_zero) * item.pulses_per_unit;
  const double converted = nearestEquivalentPulse(
    canonical, current_pulse, item.wrap_period_pulse);
  if (!std::isfinite(converted)) {
    return Result::fail(CoreError::kWriteFailed, "位置命令转换后不是有限脉冲");
  }
  *pulse = converted;
  return Result::ok();
}

}  // namespace x1_coffee_hardware_core
