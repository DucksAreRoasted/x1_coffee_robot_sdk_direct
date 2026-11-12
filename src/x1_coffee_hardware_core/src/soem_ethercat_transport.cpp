#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>

#include <pthread.h>
#include <dlfcn.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>

#include "soem/soem.h"
#include "x1_coffee_hardware_core/coffee_axis_model.hpp"
#include "x1_coffee_hardware_core/zeroerr_drive.hpp"
#include "enable_trace.hpp"
#include "ethercat_startup.hpp"

// SOEM 把 15 台 AL 状态放进同一帧读回，头文件没有声明。
extern "C" int ecx_FPRD_multi(
  ecx_contextt * context, int n, uint16 * configlst, ec_alstatust * slstatlst, int timeout);

namespace x1_coffee_hardware_core
{
namespace
{

// issue 725 的相对关系：主站先发帧，从站过半个周期再取输出。
// 那篇把主站对齐周期起点、把 SYNC0 放在中点。eRob 的脉冲固定在周期起点，
// 偏移改成半周期会让 15 台停在安全操作，所以改由主站提前 500 µs 发出，差值相同。
// 周期收帧最多等 200 µs。
constexpr int kCycleFrameTimeoutUs = 200;
constexpr std::int64_t kSendLeadNs = 500000;
// 状态广播单独限时，并且几拍才读一次，不跟每拍的过程数据抢网口。
constexpr int kStatePollTimeoutUs = 200;
constexpr int kStatePollCycles = 200;
class DcCycleTimer
{
public:
  static constexpr std::int64_t kCycleNs = 1000000;
  static constexpr std::int32_t kSync0ShiftNs = 0;

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
      // 正误差表示收帧偏晚。把它拉回脉冲前 kSendLeadNs。
      const std::int64_t delta = detail::syncLeadErrorNs(dc_time, kCycleNs, kSendLeadNs);
      if (delta > 0) {
        ++integral_;
      } else if (delta < 0) {
        --integral_;
      }
      offset = -(delta / 100) - (integral_ / 20);
      // 相位修正最多 200 µs，避免一次异常时钟把睡眠拉长。
      constexpr std::int64_t kMaxOffsetNs = 200000;
      if (offset > kMaxOffsetNs) {
        offset = kMaxOffsetNs;
      } else if (offset < -kMaxOffsetNs) {
        offset = -kMaxOffsetNs;
      }
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
      wake_late_ns_ = late;
      next_ = now;
      return;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_, nullptr);
    // 睡醒后再次测量；只在睡前检查会漏掉调度唤醒延迟。不改变原有调度节拍。
    clock_gettime(CLOCK_MONOTONIC, &now);
    wake_late_ns_ = std::max<std::int64_t>(0, toNs(now) - toNs(next_));
  }

  std::int64_t wakeLateUs() const {return wake_late_ns_ / 1000;}

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

  std::int64_t wake_late_ns_{0};
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
    // 再次启动时必须重新逐台确认，不能沿用上一次连接的 OP 缓存或诊断请求。
    last_al_state_.fill(0);
    last_error_logged_.fill(0);
    al_error_logged_.fill(false);
    op_quiet_cycles_.fill(0);
    op_waiting_.fill(false);
    op_requested_ = false;
    pending_enable_probe_.store(0);
    pending_drive_mode_.store(0);
    torque_axis_ = torque_axis;
    on_cycle_ = on_cycle;
    if (bus_operational_flag_ != nullptr) {
      bus_operational_flag_->store(false, std::memory_order_release);
    }
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
    if (g_force_stop.load()) {
      return Result::fail(CoreError::kEthercatNotReady, "收到中断，停止等待 OP");
    }
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
    // 每轮重试恢复自动 INIT→PRE-OP，邮箱/PDO 配置需要 PRE-OP。
    context_.manualstatechange = 0;
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
    // 映射默认会请求 SAFE-OP。此时尚无周期帧，必须禁止这次隐式切换；
    // 后续由 cycleLoop 在连续收发后显式请求 SAFE-OP 和 OP。
    context_.manualstatechange = 1;
    ecx_config_map_group(&context_, iomap_, 0);
    active_link_ = nullptr;
    say("✓ 15 轴过程数据已映射");
    debugf("before ecx_configdc");
    ecx_configdc(&context_);
    say(
      "● 分布时钟 SYNC0 计划周期 %lld ns，偏移 %d ns，等时钟锁定后再启动",
      static_cast<long long>(DcCycleTimer::kCycleNs), DcCycleTimer::kSync0ShiftNs);
    std::int64_t shortest_watchdog_ns = -1;
    int watchdog_unknown = 0;
    for (int slave = 1; slave <= count; ++slave) {
      if (!context_.slavelist[slave].hasdc) {
        say("! 轴 %d 不支持分布时钟，过程数据输出可能不会按 SYNC0 生效", slave);
      }
      // SYNC0 要等周期帧把参考时钟发给各从站、0x092C 收进 1 µs 之后再开。
      // 过程数据看门狗短于约 100 ms 时，读一次状态就会让从站报 0x001B 并停在安全操作。
      const std::int64_t watchdog_ns = widenProcessDataWatchdog(slave);
      if (watchdog_ns < 0) {
        ++watchdog_unknown;
      } else if (shortest_watchdog_ns < 0 || watchdog_ns < shortest_watchdog_ns) {
        shortest_watchdog_ns = watchdog_ns;
      }
      // 启动周期前读取 SM2/SM3 原始寄存器；返回码不足时不当作有效配置。
      for (int sm = 2; sm <= 3; ++sm) {
        std::array<std::uint8_t, 8> raw{};
        const int rc = ecx_FPRD(&context_.port, context_.slavelist[slave].configadr,
          0x0800 + sm * 8, raw.size(), raw.data(), EC_TIMEOUTRET);
        say("[x1-sm] slave=%d sm=%d rc=%d raw=%02x %02x %02x %02x %02x %02x %02x %02x",
          slave, sm, rc, raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7]);
      }
      // 只读取设备实际同步类型；不改已有 DC 模式。失败时保留 rc 和长度供诊断。
      std::uint16_t sync_type = 0;
      int sync_size = sizeof(sync_type);
      const int sync_rc = ecx_SDOread(&context_, slave, 0x1C32, 0x01, FALSE,
        &sync_size, &sync_type, EC_TIMEOUTRXM);
      say("[x1-dc] slave=%d hasdc=%d sync_type_rc=%d size=%d sync_type=0x%04x planned_cycle_ns=%lld shift_ns=%d",
        slave, context_.slavelist[slave].hasdc, sync_rc, sync_size, etohs(sync_type),
        static_cast<long long>(DcCycleTimer::kCycleNs), DcCycleTimer::kSync0ShiftNs);
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
      if (!torque) {
        // 校验新增电流、模式和目标扭矩字段，不能只凭总字节数认定映射有效。
        struct MappingCheck {std::uint16_t index; std::uint8_t sub; std::uint32_t expected;};
        for (const auto & check : {MappingCheck{0x1A00, 5, kZeroErrTxActualCurrent},
          MappingCheck{0x1A00, 6, kZeroErrTxModeDisplay}, MappingCheck{0x1600, 3, kZeroErrRxTargetTorque}})
        {
          std::uint32_t entry = 0;
          int size = sizeof(entry);
          const int rc = ecx_SDOread(&context_, slave, check.index, check.sub, FALSE,
            &size, &entry, EC_TIMEOUTRXM);
          if (rc <= 0 || size != static_cast<int>(sizeof(entry)) || etohl(entry) != check.expected) {
            const Result failed = Result::fail(CoreError::kEthercatNotReady,
              "从站 " + std::to_string(slave) + " 的位置/扭矩共用PDO读回不符");
            shutdownToInit();
            return failed;
          }
        }
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
        position_out_[slave]->target_position = 0;
        position_out_[slave]->target_torque = 0;
        position_out_[slave]->mode_of_operation = kZeroErrCyclicPositionMode;
        position_out_[slave]->reserved = 0;
      }
    }
    if (watchdog_unknown != 0) {
      say("! %d 台读不到过程数据看门狗", watchdog_unknown);
    } else {
      say(
        "● 过程数据看门狗最短 %lld ms",
        static_cast<long long>(shortest_watchdog_ns / 1000000));
    }

    // 725：周期线程先按 DC 节拍收发，再在线程里请求安全操作和运行。
    // 不能在收发开始前用 ecx_writestate 切状态，那段时间没有周期帧。
    startEnableTrace();
    debugf("start cycle thread before SAFE_OP");
    thread_done_.store(false);
    thread_ = std::thread([this]() {cycleLoop();});
    debugf("cycle thread started");
    bool saw_op = false;
    bool bringup_failed = false;
    std::string bringup_detail;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(18);
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
        (bringup_detail.empty() ? "18 秒内没有进入 OP" : bringup_detail);
      const Result closed = shutdownToInit();
      if (!closed) {
        return closed;
      }
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

  void requestStuckEnableProbe(std::uint16_t axis_mask) override
  {
    if (axis_mask == 0) {
      return;
    }
    pending_enable_probe_.fetch_or(axis_mask);
  }

  void bindBusOperationalFlag(std::atomic<bool> * flag) override
  {
    bus_operational_flag_ = flag;
  }

  void bindCyclicEnableFlag(std::atomic<bool> * flag) override
  {
    cyclic_enable_flag_ = flag;
    if (cyclic_enable_flag_ != nullptr) {
      cyclic_enable_flag_->store(false, std::memory_order_release);
    }
  }

  void bindOperationalHook(std::function<void()> hook) override
  {
    operational_hook_ = std::move(hook);
  }

  void copyRatedTorqueNm(std::array<double, kCoffeeAxisCount> * out) const override
  {
    if (out != nullptr) {
      *out = rated_nm_;
    }
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
        return Result::fail(
          CoreError::kTransportUnavailable, "周期线程未退出，不能再次打开网口");
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
    const int result =
      (slave >= 1 && slave <= kCoffeeAxisCount &&
      torque_axis_[static_cast<std::size_t>(slave - 1)]) ?
      configureTorquePdo(context, slave) :
      configurePositionPdo(context, slave);
    configureDcSyncMode(context, slave);
    return result;
  }

  // eRob 通信手册 8.2.28：DC 同步模式须设 0x1C32:01h=0x2，周期写 0x1C32:0Ah。
  // 实测只写 01h=2 时 15 台都进不了安全运行，因此默认不写，
  // 仅在设置环境变量 X1_ECAT_DC_SYNC_TYPE2=1 时做对照实验。
  static void configureDcSyncMode(ecx_contextt * context, std::uint16_t slave)
  {
    const char * flag = std::getenv("X1_ECAT_DC_SYNC_TYPE2");
    if (flag == nullptr || std::string(flag) != "1") {
      return;
    }
    std::uint32_t cycle_ns = static_cast<std::uint32_t>(DcCycleTimer::kCycleNs);
    const int cycle_rc = ecx_SDOwrite(
      context, slave, 0x1C32, 0x0A, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTRXM);
    say("● 轴 %u 实验：写 0x1C32:0A=%u ns rc=%d", slave, cycle_ns, cycle_rc);
    std::uint16_t sync_type = 0x0002;
    const int write_rc = ecx_SDOwrite(
      context, slave, 0x1C32, 0x01, FALSE, sizeof(sync_type), &sync_type, EC_TIMEOUTRXM);
    std::uint16_t readback = 0;
    int size = static_cast<int>(sizeof(readback));
    const int read_rc = ecx_SDOread(
      context, slave, 0x1C32, 0x01, FALSE, &size, &readback, EC_TIMEOUTRXM);
    if (write_rc <= 0 || read_rc <= 0 || readback != sync_type) {
      say(
        "! 轴 %u 设 DC 同步 0x1C32:01=2 未生效，写 rc=%d 读回 0x%04x rc=%d",
        slave, write_rc, readback, read_rc);
    }
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
    // 默认记录初始化汇总及全部从站 AL 变化；详细模式仍可记录全部事件。
    if (std::getenv("X1_ECAT_DEBUG") == nullptr &&
      (frame.operational || (!frame.summary && frame.al_changed == 0))) {
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
    std::fprintf(stderr,
      "[x1-cycle] tick=%llu max_send_call_gap_us=%llu max_wake_late_us=%llu "
      "al_age_us=%lld al_changed=0x%04x\n",
      static_cast<unsigned long long>(frame.tick),
      static_cast<unsigned long long>(frame.max_send_gap_us),
      static_cast<unsigned long long>(frame.max_wake_late_us),
      static_cast<long long>(frame.diagnostics.al_age_us), frame.al_changed);
    for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
      if (frame.summary || (frame.al_changed & (std::uint32_t{1} << i)) != 0) {
        std::fprintf(stderr,
          "[x1-al] tick=%llu slave=%zu state=0x%04x code=0x%04x age_us=%lld\n",
          static_cast<unsigned long long>(frame.tick), i + 1,
          frame.diagnostics.al_state[i], frame.diagnostics.al_code[i],
          static_cast<long long>(frame.diagnostics.al_age_us));
      }
      if (!frame.summary &&
        ((frame.changed_axes | frame.al_changed) & (std::uint32_t{1} << i)) == 0) {
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
    bool operational, int wkc, int send_rc,
    const detail::EnableCycleDiagnostics & diagnostics = {})
  {
    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    enable_trace_->capture(
      feedback, command, static_cast<std::uint64_t>(now_us), operational, wkc, expected_wkc_,
      send_rc, context_.DCtime % DcCycleTimer::kCycleNs, diagnostics);
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
    // 仅显式开启时在启动配置阶段读取诊断对象，不在周期线程中插入邮箱访问。
    // 版本字符串只作原始证据，不能当成电机结构版本或据此自动选择力矩常数。
    const char * motor_diagnostics = std::getenv("X1_MOTOR_DIAGNOSTICS");
    if (motor_diagnostics != nullptr && std::string(motor_diagnostics) == "1") {
      std::uint32_t rated_ma = 0;
      int current_size = sizeof(rated_ma);
      const int current_rc = ecx_SDOread(
        context, slave, 0x6075, 0, FALSE, &current_size, &rated_ma, EC_TIMEOUTRXM);
      // 通信手册 8.2.95：UDINT、mA；不接受截断值或零值作为有效额定电流。
      if (current_rc > 0 && current_size == sizeof(rated_ma) && rated_ma > 0) {
        say("[电机诊断] axis=%u index=0x6075 rated_current_ma=%u", slave, rated_ma);
      } else {
        say("[电机诊断] axis=%u index=0x6075 unavailable rc=%d size=%d",
          slave, current_rc, current_size);
      }
      for (const std::uint16_t object : {0x1008, 0x1009, 0x100A}) {
        char value[128]{};
        int size = sizeof(value) - 1;
        const int read_rc = ecx_SDOread(
          context, slave, object, 0, FALSE, &size, value, EC_TIMEOUTRXM);
        if (read_rc > 0 && size > 0 && size < static_cast<int>(sizeof(value))) {
          // 清除换行及控制字符，避免设备字符串伪造多行日志。
          for (int i = 0; i < size; ++i) {
            if (static_cast<unsigned char>(value[i]) < 32) {value[i] = ' ';}
          }
          value[size] = '\0';
          say("[电机诊断] axis=%u index=0x%04X value=%s", slave, object, value);
        } else {
          say("[电机诊断] axis=%u index=0x%04X unavailable rc=%d size=%d",
            slave, object, read_rc, size);
        }
      }
    }
    std::uint32_t rated_mnm = 0;
    int rated_size = static_cast<int>(sizeof(rated_mnm));
    rc = ecx_SDOread(
      context, slave, 0x6076, 0x00, FALSE, &rated_size, &rated_mnm, EC_TIMEOUTRXM);
    if (rc > 0 && rated_size == 2) {
      rated_mnm &= 0xFFFF;
    }
    // 手册 8.2.96 是 UDINT，单位 mN·m。长度不是 2 或 4 就不能拿来换算。
    if (rc > 0 && rated_mnm > 0 && (rated_size == 2 || rated_size == 4)) {
      const double rated_nm = static_cast<double>(rated_mnm) / 1000.0;
      if (active_link_ != nullptr) {
        active_link_->rated_nm_[static_cast<std::size_t>(slave - 1)] = rated_nm;
      }
      say("● 轴 %u 额定力矩 %.2f N·m", slave, rated_nm);
    } else {
      say("! 轴 %u 读不到额定力矩 0x6076", slave);
    }

    // eRob_CSP.cpp：控制字、目标位置、运行模式、8 位填充。
    writePdoEntry(context, slave, 0x1600, 0, sizeof(value8), &value8);
    value32 = kZeroErrRxControlWord;
    writePdoEntry(context, slave, 0x1600, 1, sizeof(value32), &value32);
    value32 = kZeroErrRxTargetPosition;
    writePdoEntry(context, slave, 0x1600, 2, sizeof(value32), &value32);
    value32 = kZeroErrRxTargetTorque;
    writePdoEntry(context, slave, 0x1600, 3, sizeof(value32), &value32);
    value32 = kZeroErrRxModeOfOperation;
    writePdoEntry(context, slave, 0x1600, 4, sizeof(value32), &value32);
    value32 = kZeroErrRxPadding;
    writePdoEntry(context, slave, 0x1600, 5, sizeof(value32), &value32);
    value8 = 5;
    writePdoEntry(context, slave, 0x1600, 0, sizeof(value8), &value8);
    value8 = 0;
    writePdoEntry(context, slave, 0x1C12, 0, sizeof(value8), &value8);
    std::uint16_t assign = 0x1600;
    writePdoEntry(context, slave, 0x1C12, 1, sizeof(assign), &assign);
    value8 = 1;
    writePdoEntry(context, slave, 0x1C12, 0, sizeof(value8), &value8);

    value8 = 0;
    writePdoEntry(context, slave, 0x1A00, 0, sizeof(value8), &value8);
    value32 = kZeroErrTxStatusWord;
    writePdoEntry(context, slave, 0x1A00, 1, sizeof(value32), &value32);
    value32 = kZeroErrTxActualPosition;
    writePdoEntry(context, slave, 0x1A00, 2, sizeof(value32), &value32);
    value32 = kZeroErrTxActualVelocity;
    writePdoEntry(context, slave, 0x1A00, 3, sizeof(value32), &value32);
    value32 = kZeroErrTxActualTorque;
    writePdoEntry(context, slave, 0x1A00, 4, sizeof(value32), &value32);
    // 通信手册8.2.98：0x6078可映射到TxPDO，单位额定电流千分比。
    value32 = kZeroErrTxActualCurrent;
    writePdoEntry(context, slave, 0x1A00, 5, sizeof(value32), &value32);
    value32 = kZeroErrTxModeDisplay;
    writePdoEntry(context, slave, 0x1A00, 6, sizeof(value32), &value32);
    value32 = kZeroErrRxPadding;
    writePdoEntry(context, slave, 0x1A00, 7, sizeof(value32), &value32);
    value8 = 7;
    writePdoEntry(context, slave, 0x1A00, 0, sizeof(value8), &value8);
    value8 = 0;
    writePdoEntry(context, slave, 0x1C13, 0, sizeof(value8), &value8);
    assign = 0x1A00;
    writePdoEntry(context, slave, 0x1C13, 1, sizeof(assign), &assign);
    value8 = 1;
    rc = writePdoEntry(context, slave, 0x1C13, 0, sizeof(value8), &value8);
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
    // 普通线程从启动脚本继承 CPU 2；仅 EtherCAT 周期线程切到隔离的 CPU 3。
    cpu_set_t cycle_cpus;
    CPU_ZERO(&cycle_cpus);
    CPU_SET(3, &cycle_cpus);
    const int affinity_rc = pthread_setaffinity_np(
      pthread_self(), sizeof(cycle_cpus), &cycle_cpus);
    if (affinity_rc != 0) {
      // 绑核失败不得继续上电，唤醒启动检查并返回明确原因。
      std::lock_guard<std::mutex> lock(wait_mu_);
      failed_ = true;
      failure_detail_ = std::string("EtherCAT 周期线程绑定 CPU 3 失败：") +
        std::strerror(affinity_rc);
      say("✗ %s", failure_detail_.c_str());
      wait_cv_.notify_all();
      return;
    }
    op_after_ack_.fill(false);
    say("✓ EtherCAT 周期线程已绑定 CPU 3");
    raiseCycleThreadPriority();
    // 锁住已映射内存，避免上电这一段缺页把 1 ms 拍打断。
    // 用户态关不了网卡中断；中断留在 CPU 0/1，本线程只占隔离的 CPU 3。
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      say("! 周期线程内存锁定失败：%s", std::strerror(errno));
    } else {
      say("✓ 周期线程内存已锁定");
    }
    // TwinCAT 在开 SYNC0 前会快速分发约一万次参考时钟。1 ms 节拍太慢，先紧凑发一串。
    // SYNC0 寄存器要在节拍开始前写完。写 15 台大约 20 ms，放进 1 ms 循环会把那一拍拖晚。
    const bool sync0_armed_early = distributeReferenceClock();
    DcCycleTimer timer;
    timer.start();
    bool sync0_armed = sync0_armed_early;
    bool safeop_requested = false;
    bool op_requested = false;
    bool saw_op = false;
    int pre_safeop_cycles = 0;
    int primed_cycles = 0;
    int dc_cursor = 1;
    int dc_stable = 0;
    std::array<std::int32_t, kCoffeeAxisCount + 1> dc_diff{};
    std::array<bool, kCoffeeAxisCount + 1> dc_seen{};
    std::array<int, kCoffeeAxisCount + 1> watchdog_acks{};
    // 保留周期仍在收发时的 AL 快照，避免停帧后再读制造新的看门狗故障。
    std::array<ec_alstatust, kCoffeeAxisCount> startup_al{};
    auto startup_al_time = std::chrono::steady_clock::time_point{};
    int op_broadcasts = 0;
    std::int64_t last_dc = 0;
    // 先发出一帧，再按 1 ms 节拍睡醒后收发。顺序和 eRob_CSP 的周期线程一致。
    ecx_send_processdata(&context_);
    // 用墙钟限时，并在外层 15 秒等待结束前留下逐轴失败原因。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(14);
    for (int cycle = 0; !stopped() && std::chrono::steady_clock::now() < deadline; ++cycle) {
      // 上电这一段只做定时、收、填、发。不在拍内广播读状态，避免运行请求被一次读失败挡住。
      timer.wait(last_dc);
      const int wkc = ecx_receive_processdata(&context_, kCycleFrameTimeoutUs);
      last_dc = context_.DCtime;
      // 只在已经请求运行之后确认状态。上一帧已收回，这时读不会把过程数据回帧冲掉。
      if (op_requested && (cycle % 200) == 0 &&
        recoverWatchdogSlaves(watchdog_acks, startup_al, startup_al_time) && op_broadcasts < 6)
      {
        std::uint16_t slstate = htoes(static_cast<std::uint16_t>(EC_STATE_OPERATIONAL));
        context_.slavelist[0].state = EC_STATE_OPERATIONAL;
        const int rc = ecx_BWR(
          &context_.port, 0, ECT_REG_ALCTL, sizeof(slstate), &slstate, EC_TIMEOUTRET);
        ++op_broadcasts;
        say("● 仍有轴在安全操作，第 %d 次广播请求运行 rc=%d", op_broadcasts, rc);
      }
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
          item.actual_current = position_in_[slave]->actual_current;
          item.mode_of_operation = position_in_[slave]->mode_of_operation;
        }
        // SAFE_OP 可能只有输入工作计数；请求 OP 后必须等完整计数，不能信任旧过程映像。
        const bool complete = detail::completeProcessData(wkc, expected_wkc_) ||
          (!op_requested && detail::completeProcessData(wkc, context_.grouplist[0].inputsWKC));
        item.valid = complete && item.status_word != 0;
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
          position_out_[slave]->target_position = item.target_position;
          position_out_[slave]->mode_of_operation = item.mode_of_operation;
          position_out_[slave]->target_torque = item.target_torque;
          position_out_[slave]->reserved = 0;
        }
      }
      // SOEM 维护者：参考时钟先随周期帧分发，0x092C 小于 1 µs 再开 SYNC0。
      if (!sync0_armed) {
        sync0_armed = armSync0WhenClocksLocked(cycle, dc_cursor, dc_diff, dc_seen, dc_stable);
      }
      // 先按 1 ms 收发 200 拍，再请求安全操作。15 台都有反馈就说明这一步已经完成。
      if (sync0_armed && !safeop_requested && ++pre_safeop_cycles >= 200) {
        std::uint16_t slstate = htoes(static_cast<std::uint16_t>(EC_STATE_SAFE_OP));
        context_.slavelist[0].state = EC_STATE_SAFE_OP;
        ecx_BWR(
          &context_.port, 0, ECT_REG_ALCTL, sizeof(slstate), &slstate, kStatePollTimeoutUs);
        safeop_requested = true;
        say("● 周期帧已按 DC 节拍发送，请求进入安全操作");
      } else if (safeop_requested && !op_requested &&
        ready < static_cast<int>(kCoffeeAxisCount) && (cycle % 1000) == 0)
      {
        std::uint16_t slstate = htoes(static_cast<std::uint16_t>(EC_STATE_SAFE_OP));
        context_.slavelist[0].state = EC_STATE_SAFE_OP;
        ecx_BWR(
          &context_.port, 0, ECT_REG_ALCTL, sizeof(slstate), &slstate, kStatePollTimeoutUs);
      }
      // 有反馈且相位在脉冲前，就请求运行。不再等一次可能失败的状态广播。
      const bool before_sync = detail::outputReadyBeforeSync(
        last_dc, DcCycleTimer::kCycleNs, kSendLeadNs);
      if (!before_sync || ready != static_cast<int>(kCoffeeAxisCount)) {
        primed_cycles = 0;
      }
      if (before_sync && safeop_requested && !op_requested &&
        ready == static_cast<int>(kCoffeeAxisCount) &&
        ++primed_cycles >= 1000)
      {
        std::uint16_t slstate = htoes(static_cast<std::uint16_t>(EC_STATE_OPERATIONAL));
        context_.slavelist[0].state = EC_STATE_OPERATIONAL;
        const int op_rc = ecx_BWR(
          &context_.port, 0, ECT_REG_ALCTL, sizeof(slstate), &slstate, kStatePollTimeoutUs);
        op_requested = true;
        op_requested_ = true;
        if (cyclic_enable_flag_ != nullptr) {
          // 官方示例在等待进入运行时，过程数据里已经是 0x0080，不是一直发 0x0006。
          cyclic_enable_flag_->store(true, std::memory_order_release);
        }
        say("● 已连续发送有效输出，请求 15 台从站进入运行 tick=%d rc=%d dc_phase_ns=%lld",
          cycle, op_rc, static_cast<long long>(detail::dcPhaseNs(last_dc, DcCycleTimer::kCycleNs)));
      }
      detail::EnableCycleDiagnostics diagnostics;
      diagnostics.wake_late_us = timer.wakeLateUs();
      for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
        diagnostics.al_state[i] = etohs(startup_al[i].alstatus);
        diagnostics.al_code[i] = etohs(startup_al[i].alstatuscode);
      }
      if (startup_al_time != std::chrono::steady_clock::time_point{}) {
        diagnostics.al_age_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - startup_al_time).count();
      }
      diagnostics.send_started_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      const int send_rc = ecx_send_processdata(&context_);
      traceCycle(feedback, command, false, wkc, send_rc, diagnostics);
      // 安全操作下工作计数也会满。必须 15 台状态都是运行，才算进入运行。
      if (op_requested && slavesAtLeast(EC_STATE_OPERATIONAL) &&
        detail::completeProcessData(wkc, expected_wkc_))
      {
        say("✓ 15 台从站状态已确认进入运行");
        saw_op = true;
        {
          std::lock_guard<std::mutex> lock(wait_mu_);
          op_seen_ = true;
        }
        wait_cv_.notify_all();
        break;
      }
    }
    if (stopped() || !saw_op) {
      if (stopped()) {
        return;
      }
      // 快照可能较旧或尚未取得，明确标记，不能当作超时瞬间的实时状态。
      const bool has_snapshot = startup_al_time != std::chrono::steady_clock::time_point{};
      std::ostringstream stream;
      stream << "bus state=0x" << std::hex << context_.slavelist[0].state
             << std::dec << " late_us=" << (timer.maxLateNs() / 1000)
             << " phase_us=" << (detail::dcPhaseNs(last_dc, DcCycleTimer::kCycleNs) / 1000)
             << " sync0=" << (sync0_armed ? 1 : 0)
             << " al_snapshot_age_ms=" << (has_snapshot ?
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - startup_al_time).count() : -1);
      for (int slave = 1; slave <= context_.slavecount; ++slave) {
        stream << " slave" << std::dec << slave << "=0x" << std::hex
               << etohs(startup_al[slave - 1].alstatus) << "/0x"
               << etohs(startup_al[slave - 1].alstatuscode);
      }
      {
        std::lock_guard<std::mutex> lock(wait_mu_);
        failed_ = true;
        // 这句必须留在返回原因里。上层靠它决定断开重试；只打印在终端的话，
        // ros2_control 会把激活失败当成致命错误，进程以 SIGABRT 退出。
        failure_detail_ = "从站未全部进入运行：" + stream.str();
        say("✗ %s", failure_detail_.c_str());
      }
      wait_cv_.notify_all();
      return;
    }
    // 先武装 5 秒准备，再放开使能。顺序反了的话，稳态第一拍就会发出接通字。
    if (operational_hook_) {
      operational_hook_();
    }
    if (bus_operational_flag_ != nullptr) {
      bus_operational_flag_->store(true, std::memory_order_release);
    }
    // OP 之后继续按同一节拍收发，直到关闭。
    say("● 周期通信已保持");
    int steady_cycle = 0;
    long told_overruns = 0;
    int last_overrun_log = 0;
    // 工作计数不足说明有从站没有处理本帧输出。每 2 秒汇总一次，不开调试也打印。
    int wkc_window_cycles = 0;
    int wkc_bad_frames = 0;
    int wkc_window_min = expected_wkc_;
    int mixed_polls = 0;
    timer.resetStats();
    while (!stopped()) {
      const int wkc = ecx_receive_processdata(&context_, kCycleFrameTimeoutUs);
      // 连续两次广播都不是 15 台同时运行，才锁住使能。一次读失败不算掉线。
      if (steady_cycle > 0 && (steady_cycle % 1000) == 0) {
        const AlBroadcast poll = pollUniformAlState(kStatePollTimeoutUs);
        if (poll == AlBroadcast::kMixed) {
          if (++mixed_polls >= 2 && bus_operational_flag_ != nullptr &&
            bus_operational_flag_->exchange(false, std::memory_order_acq_rel))
          {
            say("! 15 台不再同时处于运行，已锁住使能");
          }
        } else {
          mixed_polls = 0;
        }
      }
      if (wkc != expected_wkc_) {
        ++wkc_bad_frames;
      }
      if (wkc < wkc_window_min) {
        wkc_window_min = wkc;
      }
      if (++wkc_window_cycles >= 2000) {
        if (wkc_bad_frames > 0) {
          say(
            "! 2 秒内 %d 帧工作计数不符，期望 %d，最低 %d",
            wkc_bad_frames, expected_wkc_, wkc_window_min);
        }
        wkc_window_cycles = 0;
        wkc_bad_frames = 0;
        wkc_window_min = expected_wkc_;
      }
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
          item.actual_current = position_in_[slave]->actual_current;
          item.mode_of_operation = position_in_[slave]->mode_of_operation;
        }
        item.valid = detail::completeProcessData(wkc, expected_wkc_) && item.status_word != 0;
      }
      if (timer.overruns() != told_overruns && steady_cycle - last_overrun_log >= 1000) {
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
          position_out_[slave]->target_position = item.target_position;
          position_out_[slave]->mode_of_operation = item.mode_of_operation;
          position_out_[slave]->target_torque = item.target_torque;
          position_out_[slave]->reserved = 0;
        }
      }
      const int send_rc = ecx_send_processdata(&context_);
      traceCycle(feedback, command, true, wkc, send_rc);
      applyPendingDriveMode();
      applyStuckEnableProbe();
      timer.wait(context_.DCtime);
    }
  }

  // 过程数据看门狗：单位是 (分频器+2)*40 ns。短于 100 ms 就拉长到约 100 ms。0 表示关闭，不动。
  // 返回拉长后的超时；读不到时返回 -1。
  std::int64_t widenProcessDataWatchdog(int slave)
  {
    const auto address = context_.slavelist[slave].configadr;
    std::uint16_t divider = 0;
    std::uint16_t counts = 0;
    const int rc_div = ecx_FPRD(
      &context_.port, address, 0x0400, sizeof(divider), &divider, EC_TIMEOUTRET);
    const int rc_cnt = ecx_FPRD(
      &context_.port, address, 0x0420, sizeof(counts), &counts, EC_TIMEOUTRET);
    if (rc_div <= 0 || rc_cnt <= 0 || counts == 0) {
      say(
        "! 轴 %d 读不到过程数据看门狗，分频 %u rc=%d 计数 %u rc=%d",
        slave, divider, rc_div, counts, rc_cnt);
      return -1;
    }
    const std::uint64_t unit_ns = (static_cast<std::uint64_t>(divider) + 2ull) * 40ull;
    const std::uint64_t timeout_ns = unit_ns * counts;
    constexpr std::uint64_t kWantNs = 100000000ull;
    if (timeout_ns >= kWantNs) {
      return static_cast<std::int64_t>(timeout_ns);
    }
    auto widened = static_cast<std::uint16_t>((kWantNs + unit_ns - 1ull) / unit_ns);
    const int rc_write = ecx_FPWR(
      &context_.port, address, 0x0420, sizeof(widened), &widened, EC_TIMEOUTRET);
    say(
      "● 轴 %d 过程数据看门狗由 %llu ms 改为约 100 ms，0x0420=%u rc=%d",
      slave, static_cast<unsigned long long>(timeout_ns / 1000000ull), widened, rc_write);
    return static_cast<std::int64_t>(unit_ns * widened);
  }

  // ecx_dcsync0 不报告写入失败，必须区分“请求配置”和“实际读回”。
  // 仅在启动 SYNC0、尚未请求 SAFE-OP 时调用；不在 OP 周期内读邮箱。
  void logSync0Readback(int slave)
  {
    const auto address = context_.slavelist[slave].configadr;
    std::uint8_t activation = 0;
    std::uint32_t cycle = 0;
    std::uint64_t start = 0;
    // 尚未请求 SAFE-OP，使用 SOEM 常规 2 ms 返回时限；不放宽周期帧的 200 µs 时限。
    const int active_rc = ecx_FPRD(&context_.port, address, ECT_REG_DCSYNCACT,
      sizeof(activation), &activation, EC_TIMEOUTRET);
    const int cycle_rc = ecx_FPRD(&context_.port, address, ECT_REG_DCCYCLE0,
      sizeof(cycle), &cycle, EC_TIMEOUTRET);
    const int start_rc = ecx_FPRD(&context_.port, address, ECT_REG_DCSTART0,
      sizeof(start), &start, EC_TIMEOUTRET);
    // 无有效应答时输出 unknown，避免把本地初始化的零误当成设备配置。
    std::ostringstream report;
    report << "[x1-sync0] slave=" << slave << " activation=";
    if (active_rc == 1) {report << "0x" << std::hex << static_cast<unsigned>(activation) << std::dec;}
    else {report << "unknown";}
    report << " rc=" << active_rc << " cycle_ns=";
    if (cycle_rc == 1) {report << etohl(cycle);} else {report << "unknown";}
    report << " rc=" << cycle_rc << " start_ns=";
    if (start_rc == 1) {report << etohll(start);} else {report << "unknown";}
    report << " rc=" << start_rc;
    say("%s", report.str().c_str());
    if (active_rc != 1 || cycle_rc != 1 || start_rc != 1 ||
      activation != 3 || etohl(cycle) != static_cast<std::uint32_t>(DcCycleTimer::kCycleNs))
    {
      say("! 轴 %d SYNC0 读回未确认预期配置，保留返回码排查", slave);
    }
  }

  // 紧凑发送过程数据，帧尾的参考时钟读会刷新各从站 0x092C。
  // 差已经小于 1 µs 时顺手启动 SYNC0，并返回是否已经启动。
  bool distributeReferenceClock()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    int frames = 0;
    while (frames < 10000 && std::chrono::steady_clock::now() < deadline && !stopped()) {
      ecx_send_processdata(&context_);
      ecx_receive_processdata(&context_, kCycleFrameTimeoutUs);
      ++frames;
    }
    const int n = std::min(context_.slavecount, static_cast<int>(kCoffeeAxisCount));
    int worst = 0;
    std::ostringstream diffs;
    for (int slave = 1; slave <= n; ++slave) {
      std::int32_t raw = 0;
      const int rc = ecx_FPRD(
        &context_.port, context_.slavelist[slave].configadr, ECT_REG_DCSYSDIFF,
        sizeof(raw), &raw, kStatePollTimeoutUs);
      const std::int32_t diff = rc > 0 ?
        detail::systemTimeDifferenceNs(static_cast<std::uint32_t>(etohl(static_cast<std::uint32_t>(raw)))) :
        0;
      if (slave > 1) {
        diffs << ',';
      }
      diffs << diff;
      const int magnitude = diff < 0 ? -diff : diff;
      if (magnitude > worst) {
        worst = magnitude;
      }
    }
    say(
      "● 参考时钟已分发 %d 帧，时钟差最大 %d ns [%s]",
      frames, worst, diffs.str().c_str());
    if (worst > 1000 || n <= 0) {
      return false;
    }
    for (int index = 1; index <= n; ++index) {
      if (!context_.slavelist[index].hasdc) {
        continue;
      }
      ecx_dcsync0(
        &context_, static_cast<std::uint16_t>(index), TRUE,
        static_cast<std::uint32_t>(DcCycleTimer::kCycleNs), DcCycleTimer::kSync0ShiftNs);
      logSync0Readback(index);
    }
    say("● 从站时钟差最大 %d ns，启动 SYNC0", worst);
    return true;
  }

  // 周期帧里的参考时钟读会让各从站更新 0x092C。差小于 1 µs 再开 SYNC0。
  bool armSync0WhenClocksLocked(
    int cycle, int & cursor,
    std::array<std::int32_t, kCoffeeAxisCount + 1> & diff,
    std::array<bool, kCoffeeAxisCount + 1> & seen,
    int & stable)
  {
    if ((cycle % 20) != 0) {
      return false;
    }
    const int n = std::min(context_.slavecount, static_cast<int>(kCoffeeAxisCount));
    if (n <= 0) {
      return false;
    }
    int slave = cursor;
    if (slave < 1 || slave > n) {
      slave = 1;
    }
    for (int guard = 0; guard < n && !context_.slavelist[slave].hasdc; ++guard) {
      slave = slave % n + 1;
    }
    if (!context_.slavelist[slave].hasdc) {
      return false;
    }
    std::int32_t raw = 0;
    const int rc = ecx_FPRD(
      &context_.port, context_.slavelist[slave].configadr, ECT_REG_DCSYSDIFF,
      sizeof(raw), &raw, kStatePollTimeoutUs);
    if (rc > 0) {
      diff[slave] = detail::systemTimeDifferenceNs(
        static_cast<std::uint32_t>(etohl(static_cast<std::uint32_t>(raw))));
      seen[slave] = true;
    }
    cursor = slave % n + 1;
    int worst = 0;
    int missing = 0;
    for (int index = 1; index <= n; ++index) {
      if (!context_.slavelist[index].hasdc) {
        continue;
      }
      if (!seen[index]) {
        ++missing;
        continue;
      }
      const int magnitude = diff[index] < 0 ? -diff[index] : diff[index];
      if (magnitude > worst) {
        worst = magnitude;
      }
    }
    // 某一台读失败就继续采。2.5 s 仍不齐也启动，避免卡在预操作。
    if (missing > 0 && cycle < 2500) {
      return false;
    }
    if (missing == 0) {
      for (int index = 1; index <= n; ++index) {
        seen[index] = false;
      }
      if (worst <= 1000 && cycle >= 800) {
        ++stable;
      } else {
        stable = 0;
      }
      if (stable < 2 && cycle < 2500) {
        return false;
      }
    }
    for (int index = 1; index <= n; ++index) {
      if (!context_.slavelist[index].hasdc) {
        continue;
      }
      ecx_dcsync0(
        &context_, static_cast<std::uint16_t>(index), TRUE,
        static_cast<std::uint32_t>(DcCycleTimer::kCycleNs), DcCycleTimer::kSync0ShiftNs);
      logSync0Readback(index);
    }
    say("● 从站时钟差最大 %d ns，启动 SYNC0", worst);
    return true;
  }

  // 一帧读回各台状态。0x001B 只应答该台并再次请求运行，不写控制字 0x6040，也不改已经在运行的轴。
  bool recoverWatchdogSlaves(
    std::array<int, kCoffeeAxisCount + 1> & ack_count,
    std::array<ec_alstatust, kCoffeeAxisCount> & snapshot,
    std::chrono::steady_clock::time_point & snapshot_time)
  {
    constexpr int kMaxAck = 3;
    bool still_safe = false;
    const int n = std::min(context_.slavecount, static_cast<int>(kCoffeeAxisCount));
    if (n <= 0) {
      return false;
    }
    std::array<std::uint16_t, kCoffeeAxisCount> address{};
    std::array<ec_alstatust, kCoffeeAxisCount> status{};
    for (int index = 0; index < n; ++index) {
      address[static_cast<std::size_t>(index)] = context_.slavelist[index + 1].configadr;
    }
    if (ecx_FPRD_multi(
          &context_, n, address.data(), status.data(), kStatePollTimeoutUs) <= 0)
    {
      return false;
    }
    // 在任何错误应答之前保存原始返回值；多报文返回值不保证每台都应答。
    snapshot = status;
    snapshot_time = std::chrono::steady_clock::now();
    for (int index = 0; index < n; ++index) {
      const int slave = index + 1;
      const auto state = etohs(status[static_cast<std::size_t>(index)].alstatus);
      const auto code = etohs(status[static_cast<std::size_t>(index)].alstatuscode);
      last_al_state_[slave] = state;
      context_.slavelist[slave].state = state;
      context_.slavelist[slave].ALstatuscode = code;
      if ((state & EC_STATE_ERROR) != 0 && code == 0x001B && ack_count[slave] < kMaxAck) {
        // 只清 EtherCAT 状态错误，不写驱动器控制字 0x6040。一次只写一台。
        const auto command = htoes(static_cast<std::uint16_t>(EC_STATE_SAFE_OP | EC_STATE_ACK));
        const int rc = ecx_FPWRw(
          &context_.port, address[static_cast<std::size_t>(index)], ECT_REG_ALCTL, command,
          EC_TIMEOUTRET);
        ++ack_count[slave];
        // 应答之后才允许再广播运行。之前连续广播 rc=15 也留在 0x04。
        op_after_ack_[slave] = true;
        say("! 轴 %d 同步管理器看门狗，应答 rc=%d", slave, rc);
        return false;
      }
      if (state == EC_STATE_SAFE_OP && op_after_ack_[slave]) {
        op_after_ack_[slave] = false;
        still_safe = true;
      }
    }
    return still_safe;
  }

  // 一帧广播读 15 台 AL 状态。状态一致才写入缓存。逐台读会把过程数据帧间隔拉过看门狗。
  enum class AlBroadcast { kNoFrame, kUniform, kMixed };

  AlBroadcast pollUniformAlState(int timeout_us)
  {
    std::uint16_t raw = 0;
    const int wkc = ecx_BRD(
      &context_.port, 0, ECT_REG_ALSTAT, sizeof(raw), &raw, timeout_us);
    if (wkc < context_.slavecount) {
      return AlBroadcast::kNoFrame;
    }
    const std::uint16_t state = etohs(raw);
    if ((state & EC_STATE_ERROR) != 0) {
      return AlBroadcast::kMixed;
    }
    const auto bare = static_cast<std::uint16_t>(state & 0x000F);
    if (bare != EC_STATE_INIT && bare != EC_STATE_PRE_OP &&
      bare != EC_STATE_SAFE_OP && bare != EC_STATE_OPERATIONAL)
    {
      return AlBroadcast::kMixed;
    }
    for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
      last_al_state_[slave] = bare;
      if (slave <= context_.slavecount) {
        context_.slavelist[slave].state = bare;
        context_.slavelist[slave].ALstatuscode = 0;
      }
    }
    context_.slavelist[0].state = bare;
    return AlBroadcast::kUniform;
  }

  // 只读一台的 AL 状态和 AL 码，不自动应答错误或改写状态。
  // 诊断里只广播一次、之后只发过程数据，15 台才会陆续进入运行。
  void refreshOneSlaveState(int slave)
  {
    if (slave < 1 || slave > context_.slavecount) {
      return;
    }
    std::array<std::uint8_t, 6> registers{};
    const auto address = context_.slavelist[slave].configadr;
    const int rc = ecx_FPRD(
      &context_.port, address, 0x0130, registers.size(), registers.data(), EC_TIMEOUTRET);
    if (rc <= 0) {
      // 读取失败不能继续使用上一轮 OP 状态。
      last_al_state_[slave] = 0;
      context_.slavelist[slave].state = 0;
      say("! 轴 %d AL 状态读取失败 rc=%d", slave, rc);
      return;
    }
    const auto al = detail::decodeAlStatus(registers);
    last_al_state_[slave] = al.state;
    context_.slavelist[slave].state = al.state;
    context_.slavelist[slave].ALstatuscode = al.code;
    if (al.state == EC_STATE_OPERATIONAL) {
      al_error_logged_[slave] = false;
      if (op_waiting_[slave]) {
        op_waiting_[slave] = false;
        say("● 轴 %d 已进入运行", slave);
      }
      return;
    }
    if (!op_requested_) {
      return;
    }
    op_waiting_[slave] = true;
    if ((al.state & EC_STATE_ERROR) != 0) {
      // 只记录真实 AL 码，不自动应答错误或重新请求运行。
      if (last_error_logged_[slave] != al.code || !al_error_logged_[slave]) {
        last_error_logged_[slave] = al.code;
        al_error_logged_[slave] = true;
        say("! 轴 %d 总线错误，状态 0x%02x AL码 0x%04x", slave, al.state, al.code);
      }
      return;
    }
    al_error_logged_[slave] = false;
    if (op_quiet_cycles_[slave] > 0) {
      --op_quiet_cycles_[slave];
      return;
    }
    op_quiet_cycles_[slave] = 10;
    say(
      "! 轴 %d 仍在安全操作，状态 0x%02x AL码 0x%04x",
      slave, al.state, al.code);
  }

  bool slavesAtLeast(std::uint16_t state) const
  {
    if (context_.slavecount < static_cast<int>(kCoffeeAxisCount)) {
      return false;
    }
    for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
      if (!detail::alStateReady(last_al_state_[slave], state)) {
        return false;
      }
    }
    return true;
  }

  void applyPendingDriveMode()
  {
    const std::uint32_t packed = pending_drive_mode_.exchange(0);
    if (packed == 0) {return;}
    // 实际模式由onCycle输出0x6060并通过0x6061观察；请求不代表切换完成。
    say("● 模式切换已请求，mask=0x%04x mode=0x%02x，等待PDO模式反馈",
      static_cast<unsigned>(packed >> 8), static_cast<unsigned>(packed & 0xFF));
  }

  void applyStuckEnableProbe()
  {
    const std::uint16_t mask = pending_enable_probe_.exchange(0);
    if (mask == 0) {
      return;
    }
    // 使用已采集的 AL 状态；禁止在 1 ms 周期中批量访问邮箱或改写驱动器控制字。
    for (int slave = 1; slave <= static_cast<int>(kCoffeeAxisCount); ++slave) {
      if ((mask & (1u << (slave - 1))) == 0) {
        continue;
      }
      const ec_slavet & info = context_.slavelist[slave];
      say("! 轴 %d 使能停滞，缓存总线状态 0x%02x AL码 0x%04x；保留故障供排查",
        slave, info.state, info.ALstatuscode);
    }
  }

  ecx_contextt context_{};
  std::atomic<std::uint32_t> pending_drive_mode_{0};
  std::atomic<std::uint16_t> pending_enable_probe_{0};
  std::array<double, kCoffeeAxisCount> rated_nm_{};
  std::uint8_t iomap_[8192]{};
  bool opened_{false};
  std::array<bool, kCoffeeAxisCount> torque_axis_{};
  Cycle on_cycle_;
  std::atomic<bool> * bus_operational_flag_{nullptr};
  std::atomic<bool> * cyclic_enable_flag_{nullptr};
  std::array<bool, kCoffeeAxisCount + 1> op_after_ack_{};
  std::function<void()> operational_hook_{};
  std::thread thread_;
  std::unique_ptr<detail::EnableTrace> enable_trace_;
  std::thread trace_thread_;
  std::atomic<bool> trace_stop_{false};
  int expected_wkc_{0};
  std::array<std::uint16_t, kCoffeeAxisCount + 1> last_al_state_{};
  std::array<std::uint16_t, kCoffeeAxisCount + 1> last_error_logged_{};
  std::array<bool, kCoffeeAxisCount + 1> al_error_logged_{};
  std::array<int, kCoffeeAxisCount + 1> op_quiet_cycles_{};
  std::array<bool, kCoffeeAxisCount + 1> op_waiting_{};
  bool op_requested_{false};
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

// 与 DcCycleTimer 的 1 ms 节拍一致。CSP 每个通信周期写一次 0x607A。
constexpr double kEthercatCycleSec = 1.0e-3;
// 手册：同一台关节下使能到再使能的间隔应大于 300 ms。1 ms 一个周期。
constexpr int kEnableGapCycles = 300;
// eRob_CSP.cpp 周期线程：1 ms 一拍。400 拍故障复位，随后关机、接通、使能各 200 拍。
constexpr int kOfficialResetCycles = 400;
constexpr int kOfficialShutdownCycles = 200;
constexpr int kOfficialSwitchOnCycles = 200;
constexpr int kOfficialEnableCycles = 200;
constexpr int kOfficialEnableEnd =
  kOfficialResetCycles + kOfficialShutdownCycles + kOfficialSwitchOnCycles + kOfficialEnableCycles;

std::uint16_t officialCspEnableWord(int step)
{
  if (step <= kOfficialResetCycles) {
    return 0x0080;
  }
  if (step <= kOfficialResetCycles + kOfficialShutdownCycles) {
    return 0x0006;
  }
  if (step <= kOfficialResetCycles + kOfficialShutdownCycles + kOfficialSwitchOnCycles) {
    return 0x0007;
  }
  return 0x000F;
}

// 准备接通先保持接通字 0x0007 约 1 秒，等掩码离开 0x0021。
constexpr int kSwitchOnHoldCycles = 1000;
// 1 秒后仍是准备接通，再给使能字 0x000F 约 0.5 秒。
constexpr int kEnableStuckCycles = 500;
// 关电压后最多再等 2 秒。看不到接通禁止就不算新一轮。
constexpr int kVoltageOffWaitCycles = 2000;
// 1 ms 一个周期，2000 拍约 2 秒。只给未使能的轴打摘要。
constexpr int kEnableDebugCycles = 2000;

// 日志用的掩码名称，和本文件里比较的 0x006F 一致。不消解状态字 bit5/bit6 的手册冲突。
const char * maskedEnableState(std::uint16_t status_word)
{
  switch (status_word & 0x006F) {
    case 0x0000: return "未就绪";
    case 0x0040: return "接通禁止";
    case 0x0021: return "准备接通";
    case 0x0023: return "已接通";
    case 0x0027: return "已使能";
    case 0x0007: return "快速停机";
    case 0x0008: return "故障";
    case 0x000F: return "故障反应";
    default: return "未知掩码";
  }
}

const char * enablePhaseLabel(int phase)
{
  switch (phase) {
    case 1: return "关电压";
    case 2: return "等待300ms";
    default: return "跟随状态字";
  }
}

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
  right_joint_scale_.fill(1.0);
}

SoemEthercatTransport::~SoemEthercatTransport()
{
  gravity_calib_stop_.store(true);
  if (gravity_calib_thread_.joinable()) {
    gravity_calib_thread_.join();
  }
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
  current_loaded_ = false;
  current_active_ = false;
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
      right_arm_gravity_yaml_ = *yaml;
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
          } else if (key == "joint_scale" || key == "joint_bias") {
            auto & dest = key == "joint_scale" ? right_joint_scale_ : right_joint_bias_;
            for (int slot = 0; slot < 7; ++slot) {
              dest[static_cast<std::size_t>(slot)] = values[slot];
            }
          } else if (key == "sin_amplitude") {
            for (int slot = 0; slot < 7; ++slot) {
              gravity_amplitude_nm_[static_cast<std::size_t>(8 + slot)] = values[slot];
            }
          }
        }
        if (gravity_.setRightArmCorrection(mass_scale, com_delta)) {
          rememberRightArmCorrection(mass_scale, com_delta);
          say("✓ 已读右臂质量倍率和质心偏移");
        }
      }
      if (!right_correction_known_) {
        rememberRightArmCorrection(mass_scale, com_delta);
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

// 随机有轴停在安全操作时，整段回到 INIT 再开。映射错误和中断不重试。
bool bringupShouldRetry(const Result & opened)
{
  if (g_force_stop.load()) {
    return false;
  }
  const std::string & detail = opened.detail;
  // 原始 EtherCAT 网口没有 root 权限时，重试不会改变权限，必须立即停止并提示启动方式。
  if (detail.find("原始网口需要 root") != std::string::npos ||
    detail.find("需要 root") != std::string::npos)
  {
    return false;
  }
  if (detail.find("收到中断") != std::string::npos ||
    detail.find("过程数据长度不符") != std::string::npos ||
    detail.find("周期线程未退出") != std::string::npos)
  {
    return false;
  }
  return detail.find("从站未全部进入运行") != std::string::npos ||
    detail.find("bus state=") != std::string::npos ||
    detail.find("没有进入 OP") != std::string::npos ||
    detail.find("从站数不是 15") != std::string::npos ||
    detail.find("ecx_init 失败") != std::string::npos;
}

Result SoemEthercatTransport::activate()
{
  if (!configured_ || link_ == nullptr) {
    return Result::fail(CoreError::kInvalidConfiguration, "EtherCAT 传输层尚未完成配置");
  }
  debugf("activate begin");
  link_->bindBusOperationalFlag(&bus_operational_);
  link_->bindCyclicEnableFlag(&cyclic_enable_);
  link_->bindOperationalHook([this]() {
      // 官方 eRob_CSP 在进入运行后立刻按节拍接通，没有另外的 5 秒等待。
      servo_ready_delay_ms_ = 0;
      servo_wait_logged_sec_ = -1;
      servo_wait_done_logged_ = true;
    });
  for (int attempt = 1;; ++attempt) {
    if (g_force_stop.load()) {
      return Result::fail(CoreError::kEthercatNotReady, "收到中断，停止重试上电");
    }
    // 用墙钟记录整轮初始化耗时，避免将有效周期数误当成实际等待时间。
    const auto attempt_started = std::chrono::steady_clock::now();
    say("● 第 %d 次总线初始化开始", attempt);
    // 上一轮可能已经锁过位置、走过接通节拍。重开前清掉，避免沿用失败那次的状态。
    cyclic_enable_.store(false, std::memory_order_release);
    shutdown_outputs_ = false;
    current_active_ = false;
    mit_active_ = false;
    motion_armed_ = false;
    path_active_ = false;
    path_.clear();
    path_cycle_ = 0;
    official_enable_step_ = 0;
    enable_debug_cycles_ = 0;
    enabled_mask_seen_ = 0;
    all_enabled_logged_ = false;
    servo_ready_delay_ms_ = 0;
    servo_wait_logged_sec_ = -1;
    servo_wait_done_logged_ = false;
    enable_ready_cycles_.fill(0);
    enable_backoff_cycles_.fill(0);
    enable_retries_.fill(0);
    enable_phase_.fill(0);
    enable_probe_sent_.fill(false);
    held_.fill(false);
    held_pulse_.fill(0);
    goal_pulse_.fill(0);
    has_previous_position_ = false;
    const Result opened = link_->openAndReachOp(
      ifname_, torque_axis_,
      [this](const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
        std::array<SlaveCommand, kCoffeeAxisCount> * command) {
        onCycle(feedback, command);
      });
    const auto attempt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - attempt_started).count();
    if (opened) {
      say("✓ 第 %d 次总线初始化进入 OP，耗时 %lld ms", attempt,
        static_cast<long long>(attempt_ms));
      break;
    }
    debugf("activate failed: %s", opened.detail.c_str());
    // 重试成功也不能丢掉前面失败的原因，不经过 512 字节调试缓冲区截断。
    std::fprintf(stderr, "! 第 %d 次上电失败，耗时 %lld ms：%s\n", attempt,
      static_cast<long long>(attempt_ms), opened.detail.c_str());
    if (!bringupShouldRetry(opened)) {
      return opened;
    }
    const bool no_slave = opened.detail.find("读到 -1") != std::string::npos ||
      opened.detail.find("读到 0") != std::string::npos;
    say(
      no_slave ? "! 第 %d 次上电没有从站应答，断开后重试" :
      "! 第 %d 次上电没有全部进入运行，断开后重试",
      attempt);
    // 没有载波时马上重开会一直读到 -1。等链路起来再扫。
    std::this_thread::sleep_for(std::chrono::milliseconds(no_slave ? 2000 : 400));
  }
  debugf("activate reached OP");
  link_->copyRatedTorqueNm(&rated_axis_nm_);
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
  if (!bus_operational_.load(std::memory_order_acquire)) {
    return Result::fail(CoreError::kReadFailed, "EtherCAT 总线未全部处于 OP，需检查后重新启动");
  }
  std::array<SlaveFeedback, kCoffeeAxisCount> feedback{};
  std::array<bool, kCoffeeAxisCount> held{};
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    feedback = latest_;
    held = held_;
  }
  snapshot->sampled_at = std::chrono::steady_clock::now();
  const double elapsed = has_previous_position_ ?
    std::chrono::duration<double>(snapshot->sampled_at - previous_read_time_).count() : 0.0;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    AxisState & axis = snapshot->axes[index];
    const SlaveFeedback & item = feedback[index];
    if (!item.valid || !held[index]) {
      return Result::fail(CoreError::kReadFailed, "从站反馈无效或位置尚未锁存");
    }
    axis.encoder_pulse_raw = item.actual_position;
    axis.encoder_valid = true;
    axis.position = pulseToPosition(index, item.actual_position);
    axis.position_valid = std::isfinite(axis.position);
    if (has_previous_position_) {
      // 反馈为 rad/s；跨编码器整圈时仍沿用每轴脉冲归一化规则。
      const double pulse_delta = normalizePulseDelta(
        (axis.position - previous_position_[index]) * calibration_[index].pulses_per_unit,
        calibration_[index].wrap_period_pulse);
      axis.velocity = elapsed > 0.0 ?
        pulse_delta / calibration_[index].pulses_per_unit / elapsed : 0.0;
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
    axis.current_permille = item.actual_current;
    axis.current_valid = true;
  }
  previous_read_time_ = snapshot->sampled_at;
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
    if (!latest_[index].valid || latest_[index].mode_of_operation != kZeroErrCyclicPositionMode) {
      return Result::fail(CoreError::kReadFailed, "实际模式尚未确认CSP，拒绝位置目标");
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

Result SoemEthercatTransport::cancelPvtHold()
{
  std::lock_guard<std::mutex> lock(sample_mu_);
  path_active_ = false;
  path_.clear();
  motion_armed_ = false;
  if (!active_ || mit_active_) {
    return Result::fail(CoreError::kWriteFailed, "未激活或处于 MIT，不能位置保持");
  }
  for (const auto & feedback : latest_) {
    if (!feedback.valid || (feedback.status_word & 0x006F) != 0x0027) {
      // 无有效使能反馈时不可猜测当前位置，沿用故障关断策略。
      shutdown_outputs_ = true;
      return Result::fail(CoreError::kWriteFailed, "反馈无效，已撤销轨迹并请求关断");
    }
  }
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    held_pulse_[i] = latest_[i].actual_position;
    goal_pulse_[i] = held_pulse_[i];
  }
  return Result{CoreError::kOk, "PVT 已取消，锁存当前反馈位置"};
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
  if (shutdown_outputs_) {
    return Result::fail(CoreError::kWriteFailed, "关断已锁存，禁止重新允许运动；需检查原因后重新初始化");
  }
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
    if (!latest_[index].valid || latest_[index].mode_of_operation != kZeroErrCyclicPositionMode) {
      return Result::fail(CoreError::kReadFailed, "实际模式尚未确认CSP，拒绝位置目标");
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
  last_cycle_at_ = std::chrono::steady_clock::now();
  if (current_active_) {updateCurrentGravity(feedback);}
  // 15 轴共用同一个时刻。本周期的目标是插补点，速度上限只防止单周期跳变。
  if (mit_active_ && !current_active_ && gravity_ready_) {
    std::array<double, kCoffeeAxisCount> q{};
    for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
      if (feedback[index].valid) {
        q[index] = pulseToPosition(index, feedback[index].actual_position);
      }
    }
    std::array<double, kCoffeeAxisCount> tau{};
    if (gravity_.torques(q, &tau)) {
      for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
        mit_gravity_nm_[index] = gravity_scale_ * tau[index];
        if (!(rated_axis_nm_[index] > 0.0)) {
          mit_gravity_permille_[index] = 0.0;
          if ((gravity_unscaled_mask_ & (1u << index)) == 0) {
            gravity_unscaled_mask_ |= static_cast<std::uint16_t>(1u << index);
            say("! 轴 %zu 没有 0x6076，重力前馈记为 0", index + 1);
          }
          continue;
        }
        mit_gravity_permille_[index] = torquePermilleFromNm(
          gravity_scale_ * tau[index], rated_axis_nm_[index]);
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
  // eRob_CSP.cpp 的 ecatthread：400 拍 0x0080，200 拍 0x0006，200 拍 0x0007，200 拍 0x000F。
  // 1 ms 一拍。目标位置不跟示例写成 0，避免多圈关节被拉向脉冲 0。
  // 不完整帧不计数。走完 1000 拍后再加一拍，节拍才能结束，后面的运动才进得去。
  bool frame_complete = true;
  for (const SlaveFeedback & item : feedback) {
    if (!item.valid) {
      frame_complete = false;
      break;
    }
  }
  const bool cyclic_enable = bus_operational_.load(std::memory_order_acquire) ||
    cyclic_enable_.load(std::memory_order_acquire);
  if (frame_complete && drives_enabled_ && !servo_waiting &&
    cyclic_enable &&
    official_enable_step_ <= kOfficialEnableEnd)
  {
    ++official_enable_step_;
    if (official_enable_step_ == 1) {
      say("● 开始官方接通序列：400/200/200/200 个有效周期（标称 1 ms/周期）；尚不代表全部进入 OP 或使能完成");
      std::fflush(stderr);
    }
  }
  if (!drives_enabled_ || official_enable_step_ > kOfficialEnableEnd) {
    samplePath();
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
    const bool cst_zero = (cst_zero_mask_ & (1u << index)) != 0;
    out.mode_of_operation = (mit_active_ && (mit_mask_ & (1u << index)) != 0) || cst_zero ?
      kZeroErrCyclicTorqueMode : kZeroErrCyclicPositionMode;
    // 不完整帧不推进使能。故障位在官方 1 秒节拍结束前仍按示例发 0x0080，之后才停在 0。
    if (!item.valid) {
      out.control_word = 0x0000;
      continue;
    }
    if (shutdown_outputs_ || !held_[index]) {
      out.control_word = shutdown_outputs_ ? 0x0006 : 0x0000;
      continue;
    }
    if (!drives_enabled_) {
      out.control_word = 0x0000;
      continue;
    }
    // 还没请求运行时只发关机字。请求之后按官方节拍发 0x0080，不等 15 台都进去。
    if (!bus_operational_.load(std::memory_order_acquire) &&
      !cyclic_enable_.load(std::memory_order_acquire))
    {
      out.control_word = 0x0006;
      continue;
    }
    // 手册 5.1.1：上电后伺服还没准备好时不写接通，只保持关机字。
    if (!torque_axis_[index] && servo_waiting) {
      out.control_word = 0x0006;
      continue;
    }
    // 官方接通节拍走完之前，位置轴不按状态字分支，也不关电压重试。
    if (!torque_axis_[index] && official_enable_step_ > 0 &&
      official_enable_step_ <= kOfficialEnableEnd)
    {
      out.control_word = officialCspEnableWord(official_enable_step_);
      continue;
    }
    if ((item.status_word & 0x0008) != 0) {
      out.control_word = 0x0000;
      continue;
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
    // MIT 和零力矩 CST 都把 0x607A 贴着编码器：切回 CSP 不会猛拉，也不会让驱动器
    // 按进入时的旧目标累积位置误差。0x1288 就是这么被带出来的。
    const bool mit_axis = mit_active_ && (mit_mask_ & (1u << index)) != 0;
    if ((mit_axis || cst_zero) && !torque_axis_[index] && item.valid && operation_enabled) {
      held_pulse_[index] = item.actual_position;
      goal_pulse_[index] = item.actual_position;
      out.target_position = item.actual_position;
      if (!mit_axis) {
        // 零力矩验证：目标力矩是常数偏置，第 4 步的小力矩也走这条路。
        out.target_torque = static_cast<std::int16_t>(cst_zero_torque_permille_);
      } else if (current_active_) {
        const int side = index >= 8 ? 1 : 0;
        const int slot = static_cast<int>(index) - (side == 1 ? 8 : 1);
        out.target_torque = static_cast<std::int16_t>(std::lround(current_previous_[side][slot]));
      } else {
        out.target_torque = mitTorque(index, item.actual_position);
      }
    }
  }
  if (bus_operational_.load(std::memory_order_acquire)) {
    logEnableProgress(feedback, *command);
  }
}

void SoemEthercatTransport::logEnableProgress(
  const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback,
  const std::array<SlaveCommand, kCoffeeAxisCount> & command)
{
  if (!drives_enabled_ || shutdown_outputs_) {
    return;
  }
  std::uint16_t enabled_now = 0;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    const SlaveFeedback & item = feedback[index];
    if (item.valid && (item.status_word & 0x006F) == 0x0027) {
      enabled_now = static_cast<std::uint16_t>(enabled_now | (1u << index));
    }
  }
  if (enabled_now == 0x7FFF) {
    if (!all_enabled_logged_) {
      all_enabled_logged_ = true;
      say("✓ 15 轴状态字掩码都是已使能 0x0027");
    }
    enable_debug_cycles_ = 0;
    enabled_mask_seen_ = enabled_now;
    return;
  }
  all_enabled_logged_ = false;
  if (++enable_debug_cycles_ < kEnableDebugCycles) {
    return;
  }
  enable_debug_cycles_ = 0;
  const std::uint16_t dropped = static_cast<std::uint16_t>(enabled_mask_seen_ & ~enabled_now);
  enabled_mask_seen_ = enabled_now;
  int enabled_count = 0;
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    if ((enabled_now & (1u << index)) != 0) {
      ++enabled_count;
    }
  }
  say(
    "%s使能 %d/15，未使能 %d 轴",
    servo_ready_delay_ms_ > 0 && !servo_wait_done_logged_ ? "伺服准备中，" : "",
    enabled_count, static_cast<int>(kCoffeeAxisCount) - enabled_count);
  const auto & names = coffeeAxisDescriptors();
  for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
    if ((enabled_now & (1u << index)) != 0) {
      continue;
    }
    const SlaveFeedback & item = feedback[index];
    const char * dropped_note = (dropped & (1u << index)) != 0 ? " 本段曾离开已使能" : "";
    if (!item.valid) {
      say(
        "! %s 无过程数据 控制字 0x%04x 阶段 %s 重试 %d%s",
        names[index].joint_name.data(), command[index].control_word,
        enablePhaseLabel(enable_phase_[index]), enable_retries_[index], dropped_note);
      continue;
    }
    say(
      "! %s 状态 0x%04x 掩码 0x%04x %s bit3=%d bit4=%d bit7=%d "
      "控制字 0x%04x 阶段 %s 重试 %d 准备接通已保持 %d 周期 位置 %d%s",
      names[index].joint_name.data(), item.status_word, item.status_word & 0x006F,
      maskedEnableState(item.status_word),
      (item.status_word & 0x0008) != 0 ? 1 : 0,
      (item.status_word & 0x0010) != 0 ? 1 : 0,
      (item.status_word & 0x0080) != 0 ? 1 : 0,
      command[index].control_word, enablePhaseLabel(enable_phase_[index]),
      enable_retries_[index], enable_ready_cycles_[index], item.actual_position, dropped_note);
  }
}

Result SoemEthercatTransport::loadCurrentGravity(const std::string & path)
{
  std::uint64_t fingerprint = 0;
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    if (mit_active_ || path_active_ || gravity_calib_running_.load()) {
      return Result::fail(CoreError::kWriteFailed, "运动或扭矩模式中不能加载参数");
    }
    current_loaded_ = false;
    if (!gravity_ready_) {return Result::fail(CoreError::kInvalidConfiguration, "没有URDF模型");}
    fingerprint = gravity_.sourceFingerprint();
  }
  // 文件读写和内存分配不持有周期线程锁。
  CurrentGravityBundle parsed;
  std::string error;
  if (!loadCurrentGravityFile(path, fingerprint, &parsed, &error)) {
    return Result::fail(CoreError::kInvalidConfiguration, error);
  }
  std::array<CurrentGravity, 2> models;
  for (int side = 0; side < 2; ++side) {
    if (parsed.present[side] && !models[side].configure(parsed.arms[side])) {
      return Result::fail(CoreError::kInvalidConfiguration, "电流模型参数无效");
    }
  }
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (mit_active_ || path_active_ || gravity_calib_running_.load() ||
    fingerprint != gravity_.sourceFingerprint()) {
    return Result::fail(CoreError::kWriteFailed, "加载期间运行状态改变，未应用参数");
  }
  current_bundle_ = std::move(parsed);
  current_models_ = std::move(models);
  current_loaded_ = true;
  current_fault_ = 0;
  std::string detail = "电流参数已加载，未切换模式；仅供现场受控验证";
  // 回传原候选摘要，自动验证工具据此避免测试到旧模型。
  for (int side = 0; side < 2; ++side) {
    if (current_bundle_.present[side]) {
      detail += std::string(side == 0 ? " candidate_l=" : " candidate_r=") +
        current_bundle_.candidate_sha256[side];
    }
  }
  return Result{CoreError::kOk, detail};
}

Result SoemEthercatTransport::updateMitHandfeel(int joint, double q_des, double dq_des, double kp, double kd, double tau_ff)
{
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (!current_handfeel_ || !mit_active_) return Result::fail(CoreError::kWriteFailed, "MIT手感尚未进入");
  if (!handfeel_.update(joint, q_des, dq_des, kp, kd, tau_ff)) return Result::fail(CoreError::kInvalidConfiguration, "MIT参数超出逐轴安全范围");
  return Result{CoreError::kOk, "MIT参数已提交到实时线程"};
}

Result SoemEthercatTransport::enterCstZeroTorque(
  std::uint16_t axis_mask, std::int32_t torque_permille)
{
  // 校验在锁内做完就放锁：IgH 的 requestDriveMode 是阻塞邮箱调用，
  // 持锁调用会把周期线程卡住，服务也会永远不返回。
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    if (!active_ || !drives_enabled_ || !bus_operational_.load() || shutdown_outputs_ ||
      mit_active_ || path_active_ || motion_armed_ ||
      std::chrono::steady_clock::now() - last_cycle_at_ > std::chrono::milliseconds(100))
    {return Result::fail(CoreError::kWriteFailed, "零力矩 CST 验证需要新鲜 CSP 保持且无其他运动占用");}
    if (axis_mask == 0) {
      // 退出前把 0x607A 重新对齐退出瞬间的实测值。CST 期间轴若被重力带偏，
      // 直接回 CSP 会按进入前的旧目标猛拉，判据 4 也就不成立。
      for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
        if ((cst_zero_mask_ & (1u << i)) == 0 || !latest_[i].valid) {continue;}
        held_pulse_[i] = latest_[i].actual_position;
        goal_pulse_[i] = latest_[i].actual_position;
      }
    } else {
      // 第 4 步的小力矩只允许单轴，且不超过额定 5%（50‰）。多轴同时加力矩
      // 在没人扶的条件下没有意义，也放大了力矩符号搞反的后果。
      if (std::abs(torque_permille) > kCstZeroMaxPermille) {
        return Result::fail(CoreError::kInvalidConfiguration, "小力矩超出额定 5%（±50‰）");
      }
      int axis_count = 0;
      for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
        if ((axis_mask & (1u << i)) == 0) {continue;}
        ++axis_count;
        if (!latest_[i].valid || (latest_[i].status_word & 0x006F) != 0x0027) {
          return Result::fail(CoreError::kReadFailed, "目标轴未处于已使能状态，拒绝切 CST");
        }
      }
      if (torque_permille != 0 && axis_count != 1) {
        return Result::fail(CoreError::kInvalidConfiguration, "施加小力矩时掩码只能有一个轴");
      }
    }
    cst_zero_mask_ = axis_mask;
    cst_zero_torque_permille_ = axis_mask == 0 ? 0 : torque_permille;
  }
  if (axis_mask == 0) {
    link_->requestDriveMode(kZeroErrCyclicPositionMode, 0x7FFF);
    return Result{CoreError::kOk, "已请求回到周期同步位置"};
  }
  link_->requestDriveMode(kZeroErrCyclicTorqueMode, axis_mask);
  if (torque_permille != 0) {
    return Result{
      CoreError::kOk, "已请求切 CST 并施加 " + std::to_string(torque_permille) +
      "‰ 力矩，模式以 0x6061 读回为准"};
  }
  return Result{CoreError::kOk, "已请求切 CST 且目标力矩恒 0，模式以 0x6061 读回为准"};
}

Result SoemEthercatTransport::cstZeroStatus(std::uint16_t axis_mask)
{
  // 只读周期线程的锁存值，不发 SDO、不改任何输出，服务可以反复轮询。
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (!active_ || !drives_enabled_) {
    return Result::fail(CoreError::kWriteFailed, "总线未激活，读不到 0x6061");
  }
  if (axis_mask == 0) {
    return Result::fail(CoreError::kInvalidConfiguration, "读回掩码为空");
  }
  const auto & descriptors = coffeeAxisDescriptors();
  std::string detail = "读回";
  bool confirmed = true;
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    if ((axis_mask & (1u << i)) == 0) {continue;}
    const SlaveFeedback & item = latest_[i];
    // 期望模式跟着当前请求走：mask 内的轴应当是 CST，退出后应当已回到 CSP。
    const std::uint8_t expected = (cst_zero_mask_ & (1u << i)) != 0 ?
      kZeroErrCyclicTorqueMode : kZeroErrCyclicPositionMode;
    const bool enabled = item.valid && (item.status_word & 0x006F) == 0x0027;
    const bool mode_ok = item.valid && item.mode_of_operation == expected;
    if (!enabled || !mode_ok) {confirmed = false;}
    // Δ脉冲是 0x607A 目标与 0x6064 实际之差，退出后应接近 0。
    const std::int32_t delta = item.valid ? held_pulse_[i] - item.actual_position : 0;
    char buffer[192];
    std::snprintf(
      buffer, sizeof(buffer), " 轴%zu(%.*s)=0x%02x 期望0x%02x 状态0x%04x 力矩%d‰ Δ脉冲%d%s",
      i + 1, static_cast<int>(descriptors[i].joint_name.size()),
      descriptors[i].joint_name.data(),
      static_cast<unsigned>(item.mode_of_operation),
      static_cast<unsigned>(expected), static_cast<unsigned>(item.status_word),
      static_cast<int>(item.actual_torque), static_cast<int>(delta),
      (!enabled || !mode_ok) ? " 未确认" : "");
    detail += buffer;
  }
  if (!confirmed) {
    return Result::fail(CoreError::kWriteFailed, detail);
  }
  return Result{CoreError::kOk, detail};
}

Result SoemEthercatTransport::enterMitHandfeel()
{
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    if (!active_ || !gravity_ready_ || !drives_enabled_ || shutdown_outputs_ ||
      !bus_operational_.load() || official_enable_step_ <= kOfficialEnableEnd ||
      mit_active_ || path_active_ || motion_armed_ || gravity_calib_running_.load() ||
      std::chrono::steady_clock::now()-last_cycle_at_ > std::chrono::milliseconds(100))
    {return Result::fail(CoreError::kWriteFailed, "MIT手感试验需要新鲜CSP保持且无其他运动占用");}
    CurrentGravity::Pose q{};CurrentGravity::Arm bias{};
    for (std::size_t i=0;i<kCoffeeAxisCount;++i) {
      if (torque_axis_[i] || !latest_[i].valid || (latest_[i].status_word & 0x006F)!=0x0027 ||
        latest_[i].mode_of_operation!=kZeroErrCyclicPositionMode)
      {return Result::fail(CoreError::kReadFailed, "进入前需要15轴有效CSP使能反馈");}
      q[i]=pulseToPosition(i,latest_[i].actual_position);
      if (i>=8) {bias[i-8]=latest_[i].actual_current;}
    }
    if (!handfeel_.configure(q,bias)) {
      return Result::fail(CoreError::kWriteFailed, "保持电流超过手感试验预算，拒绝进入");
    }
    current_previous_[1]=bias;current_previous_q_=q;current_velocity_.fill(0.0);
    current_mode_wait_=0;current_modes_confirmed_=false;current_fault_=0;
    current_fault_joint_ = -1;
    current_over_limit_cycles_.fill(0);
    current_handfeel_=true;current_zero_gravity_=false;current_active_=true;mit_active_=true;mit_zero_g_=false;
    mit_mask_=0x7F00;motion_armed_=false;
    for (std::size_t i=0;i<kCoffeeAxisCount;++i) {
      held_pulse_[i]=goal_pulse_[i]=latest_[i].actual_position;
    }
  }
  // requestDriveMode 在 IgH 上是阻塞邮箱调用，放锁后再发，避免和周期线程互锁。
  link_->requestDriveMode(kZeroErrCyclicTorqueMode,mit_mask_);
  return Result{CoreError::kOk, "已请求右臂MIT阻抗手感，等待实际模式；全工作空间弹簧回正，腰和左臂CSP"};
}

Result SoemEthercatTransport::enterCurrentGravity()
{
  std::unique_lock<std::mutex> lock(sample_mu_);
  if (!active_ || !current_loaded_ || !drives_enabled_ || shutdown_outputs_ ||
    !bus_operational_.load() || official_enable_step_ <= kOfficialEnableEnd ||
    mit_active_ || path_active_ || gravity_calib_running_.load())
  {
    return Result::fail(CoreError::kWriteFailed, "未就绪、参数未加载或其他运动模式占用");
  }
  if (std::chrono::steady_clock::now() - last_cycle_at_ > std::chrono::milliseconds(100)) {
    return Result::fail(CoreError::kReadFailed, "进入时反馈过期");
  }
  CurrentGravity::Pose q{}, dq{};
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    if (torque_axis_[i] || !latest_[i].valid || (latest_[i].status_word & 0x006F) != 0x0027 ||
      latest_[i].mode_of_operation != kZeroErrCyclicPositionMode)
    {
      return Result::fail(CoreError::kReadFailed, "进入前需要15轴有效使能且实际模式为CSP");
    }
    q[i] = pulseToPosition(i, latest_[i].actual_position);
  }
  std::uint16_t mask = 0;
  for (int side = 0; side < 2; ++side) {
    if (!current_bundle_.present[side]) {continue;}
    const int start = side == 1 ? 8 : 1;
    for (int j = 0; j < 7; ++j) {
      // 从位置保持时实际电流起步，避免进入时先把支撑电流降为零。
      current_previous_[side][j] = latest_[start+j].actual_current;
    }
    CurrentGravity::Arm check{};
    if (!current_models_[side].command(gravity_, q, dq, current_previous_[side], kEthercatCycleSec, &check)) {
      return Result::fail(CoreError::kWriteFailed, "起点超出辨识邻域或电流预算");
    }
    mask |= side == 1 ? 0x7F00 : 0x00FE;
  }
  current_handfeel_ = false;
  current_zero_gravity_ = true;
  for (int side = 0; side < 2; ++side) {
    if (current_bundle_.present[side]) {current_models_[side].setPoseDomainEnforced(false);}
  }
  current_fault_joint_ = -1;
  current_over_limit_cycles_.fill(0);
  current_previous_q_ = q;
  current_velocity_.fill(0.0);
  current_mode_wait_ = 0;
  current_modes_confirmed_ = false;
  current_fault_ = 0;
  current_fault_joint_ = -1;
  current_over_limit_cycles_.fill(0);
  current_active_ = true;
  mit_active_ = true;
  mit_zero_g_ = false;
  mit_mask_ = mask;
  motion_armed_ = false;
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    held_pulse_[i] = goal_pulse_[i] = latest_[i].actual_position;
  }
  // 阻塞 SDO 放到锁外：周期线程也在等这把锁，持锁调用会把整个控制环卡死。
  lock.unlock();
  link_->requestDriveMode(kZeroErrCyclicTorqueMode, mask);
  return Result{CoreError::kOk, "已请求电流重力补偿，等待0x6061确认；腰和未选手臂保持位置"};
}

Result SoemEthercatTransport::currentGravityStatus()
{
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (current_fault_ != 0) {
    // 状态服务提供退出类别；不能把模型保护退出统称为模式丢失。
    const char * reason = current_fault_ == 1 ? "反馈/使能失效或收到关断请求" :
      current_fault_ == 2 ? (current_handfeel_ ? "MIT控制器输入或保持电流预算" : "重力模型计算或校准模型故障") :
      current_fault_ == 3 ? "实际模式丢失或切换确认超时" :
      current_fault_ == 4 ? "持续电流预算超限" :
      current_fault_ == 5 ? "关节软限位超限" :
      current_fault_ == 6 ? "关节速度超限" :
      current_fault_ == 7 ? "重力补偿输出预算或模型计算无效" : "未知故障";
    return Result::fail(CoreError::kWriteFailed,
      std::string("电流补偿保护退出：") + reason + "，故障码=" + std::to_string(current_fault_) +
      "，关节=" + std::to_string(current_fault_joint_ + 1));
  }
  if (!current_active_ || shutdown_outputs_ ||
    std::chrono::steady_clock::now() - last_cycle_at_ > std::chrono::milliseconds(100))
  {return Result::fail(CoreError::kWriteFailed, "电流补偿未运行或反馈过期");}
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    const auto mode = (mit_mask_ & (1u << i)) != 0 ? kZeroErrCyclicTorqueMode : kZeroErrCyclicPositionMode;
    if (!latest_[i].valid || (latest_[i].status_word & 0x006F) != 0x0027 || latest_[i].mode_of_operation != mode) {
      return Result::fail(CoreError::kReadFailed, "模式切换尚未得到全轴反馈确认");
    }
  }
  return Result{CoreError::kOk, "所选手臂CST已确认，腰和未选手臂CSP已确认"};
}

Result SoemEthercatTransport::currentGravityHoldStatus()
{
  std::lock_guard<std::mutex> lock(sample_mu_);
  if (!active_ || mit_active_ || shutdown_outputs_ || path_active_ || motion_armed_ ||
    std::chrono::steady_clock::now() - last_cycle_at_ > std::chrono::milliseconds(100))
  {return Result::fail(CoreError::kWriteFailed, "尚未确认位置保持");}
  for (const auto & item : latest_) {
    if (!item.valid || (item.status_word & 0x006F) != 0x0027 ||
      item.mode_of_operation != kZeroErrCyclicPositionMode)
    {return Result::fail(CoreError::kReadFailed, "CSP实际模式或使能反馈未确认");}
  }
  return Result{CoreError::kOk, "15轴CSP使能反馈已确认，仍须观察实际停稳"};
}

void SoemEthercatTransport::updateCurrentGravity(
  const std::array<SlaveFeedback, kCoffeeAxisCount> & feedback)
{
  CurrentGravity::Pose q{};
  int fault = 0;
  bool modes_ready = true;
  if (shutdown_outputs_) {fault = 1;}
  for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
    if (!feedback[i].valid || (feedback[i].status_word & 0x006F) != 0x0027) {fault = 1;}
    q[i] = pulseToPosition(i, feedback[i].actual_position);
    const double delta = normalizePulseDelta((q[i] - current_previous_q_[i]) * calibration_[i].pulses_per_unit,
      calibration_[i].wrap_period_pulse) / calibration_[i].pulses_per_unit;
    current_velocity_[i] += 0.05 * (delta / kEthercatCycleSec - current_velocity_[i]);
    const auto mode = (mit_mask_ & (1u << i)) != 0 ? kZeroErrCyclicTorqueMode : kZeroErrCyclicPositionMode;
    modes_ready = modes_ready && feedback[i].mode_of_operation == mode;
  }
  current_previous_q_ = q;
  if (!modes_ready && (current_modes_confirmed_ || ++current_mode_wait_ > 1000)) {fault = 3;}
  if (modes_ready) {current_mode_wait_ = 0; current_modes_confirmed_ = true;}
  // 两种拖动模式都使用真实关节软限位和速度上限，不使用进入时姿态窗口。
  static constexpr double kJointMin[kCoffeeAxisCount] = {
    -6.2831852,-3.1415926,-0.15,-3.1415926,-1.5707963,-3.1415926,-1.7453292,-6.2831853,
    -3.1415926,-3.1415926,-3.1415926,-2.0943951,-3.1415926,-1.7453292,-6.2831853};
  static constexpr double kJointMax[kCoffeeAxisCount] = {
    6.2831852,3.1415926,3.1415926,3.1415926,2.0943951,3.1415926,1.7453292,6.2831853,
    3.1415926,0.15,3.1415926,1.5707963,3.1415926,1.7453292,6.2831853};
  if (fault == 0) {
    for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
      const bool selected = (mit_mask_ & (1u << i)) != 0;
      if (!selected) {continue;}
      if (q[i] < kJointMin[i] || q[i] > kJointMax[i]) {
        fault = 5; current_fault_joint_ = static_cast<int>(i); break;
      }
      const int side = i >= 8 ? 1 : 0;
      const int slot = static_cast<int>(i) - (side == 1 ? 8 : 1);
      if (slot >= 0 && slot < 7 && current_bundle_.present[side]) {
        const double limit = current_models_[side].limitForJoint(static_cast<std::size_t>(slot));
        const double measured = std::abs(static_cast<double>(feedback[i].actual_current));
        if (measured > limit * 1.5 ||
          (measured > limit && ++current_over_limit_cycles_[i] >= 20)) {
          fault = 4; current_fault_joint_ = static_cast<int>(i); break;
        }
        if (measured <= limit) {current_over_limit_cycles_[i] = 0;}
      }
    }
  }
  auto next = current_previous_;
  if (current_handfeel_ && fault == 0 &&
    !handfeel_.command(q,current_velocity_,current_previous_[1],kEthercatCycleSec,&next[1])) {fault=2; current_fault_joint_ = 8;}
  for (int side = 0; side < 2 && fault == 0 && !current_handfeel_; ++side) {
    if (current_bundle_.present[side] && !current_models_[side].command(gravity_, q,
      current_velocity_, current_previous_[side], kEthercatCycleSec, &next[side])) {fault = 7; current_fault_joint_ = side == 1 ? 8 : 1;}
  }
  if (fault != 0) {
    current_fault_ = fault;
    current_active_ = false;
    mit_active_ = false;
    motion_armed_ = false;
    // 有效反馈下超域则回CSP保持；反馈/模式故障无法确认保持，执行关断。
    if (fault != 2) {shutdown_outputs_ = true;}
    for (std::size_t i = 0; i < kCoffeeAxisCount; ++i) {
      if (feedback[i].valid) {held_pulse_[i] = goal_pulse_[i] = feedback[i].actual_position;}
    }
    link_->requestDriveMode(kZeroErrCyclicPositionMode, mit_mask_);
    return;
  }
  if (modes_ready) {current_previous_ = next;}
}

std::int16_t SoemEthercatTransport::mitTorque(std::size_t index, std::int32_t actual_pulse)
{
  const double measured = pulseToPosition(index, actual_pulse);
  // 标定软持跟踪主站目标角。平时 MIT 的平衡点仍是进入时锁存的编码器。
  const double desired = mit_track_qdes_ ? mit_qdes_rad_[index] :
    pulseToPosition(index, mit_hold_pulse_[index]);
  double velocity = 0.0;
  if (mit_have_sample_[index]) {
    velocity = (measured - mit_prev_rad_[index]) / kEthercatCycleSec;
  }
  mit_prev_rad_[index] = measured;
  mit_have_sample_[index] = true;
  mit_velocity_[index] += 0.05 * (velocity - mit_velocity_[index]);
  const double error = desired - measured;
  const bool right = index >= 8 && index < 15;
  const int slot = right ? static_cast<int>(index - 8) : -1;
  const double rated = rated_axis_nm_[index];
  const double joint_scale = slot >= 0 &&
    right_joint_scale_[static_cast<std::size_t>(slot)] > 0.0 ?
    right_joint_scale_[static_cast<std::size_t>(slot)] : 1.0;
  const double bias = slot >= 0 ? right_joint_bias_[static_cast<std::size_t>(slot)] : 0.0;
  // URDF 主项乘倍率，再叠加零差的 A·sin(θ)。幅值为 0 时和第二项无关。
  double gravity_nm = fusedGravityNm(
    mit_gravity_nm_[index], joint_scale, gravity_amplitude_nm_[index], measured) + bias;
  // 手册 8.2.123：周期同步扭矩里，扭矩偏移加到 0x6071。偏置直接写进目标力矩。
  // 硬跟踪要落到接近点附近。22 N·m/rad 时，2 N·m 的模型残差就会停在 0.08 rad 以外。
  static constexpr double kHardKpNm[7] = {320.0, 320.0, 160.0, 160.0, 48.0, 40.0, 36.0};
  static constexpr double kHardKdNm[7] = {24.0, 24.0, 12.0, 12.0, 3.0, 2.4, 2.0};
  static constexpr double kSoftKpNm[7] = {2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0};
  static constexpr double kSoftKdNm[7] = {0.35, 0.35, 0.3, 0.3, 0.15, 0.12, 0.15};
  double command = 0.0;
  if (mit_track_qdes_ && slot >= 0) {
    const double kp_nm = mit_soft_hold_ ? kSoftKpNm[slot] : kHardKpNm[slot];
    const double kd_nm = mit_soft_hold_ ? kSoftKdNm[slot] : kHardKdNm[slot];
    if (mit_soft_hold_) {
      mit_balance_nm_[static_cast<std::size_t>(slot)] = gravity_nm + kp_nm * error;
    }
    command = torquePermilleFromNm(
      gravity_nm + kp_nm * error - kd_nm * mit_velocity_[index], rated);
  } else if (mit_zero_g_) {
    const double kd_nm = slot >= 4 ? kSoftKdNm[slot] : 0.0;
    command = torquePermilleFromNm(gravity_nm - kd_nm * mit_velocity_[index], rated);
  } else {
    // mit_ff_scale_ = 0 时就是纯 PD：重力模型完全不参与，手感只由 kp/kd 决定。
    command = mit_kp_ * error - mit_kd_ * mit_velocity_[index] +
      torquePermilleFromNm(gravity_nm * mit_ff_scale_, rated);
  }
  if (slot >= 0) {
    mit_balance_ok_[static_cast<std::size_t>(slot)] = std::abs(command) < mit_limit_ - 0.5;
  }
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
  return beginRightArmTorque(0x7F00, mit_kp_, mit_kd_, 1.0, false);
}

Result SoemEthercatTransport::enterMitHoldEx(
  std::uint16_t mask, double kp_permille, double kd_permille, double ff_scale)
{
  // 纯 PD 试验不需要重力模型；要加前馈就必须先有模型，否则会静默变成纯 PD。
  if (ff_scale > 0.0 && !gravity_ready_) {
    return Result::fail(
      CoreError::kInvalidConfiguration, "没有 URDF 重力模型，只能做 ff_scale=0 的纯 PD 试验");
  }
  return beginRightArmTorque(mask, kp_permille, kd_permille, ff_scale, false);
}

bool SoemEthercatTransport::rightArmRatedTorqueReady(std::string * missing) const
{
  const auto & descriptors = coffeeAxisDescriptors();
  std::string names;
  for (int joint = 0; joint < 7; ++joint) {
    const std::size_t index = static_cast<std::size_t>(8 + joint);
    if (rated_axis_nm_[index] > 0.0) {
      continue;
    }
    if (!names.empty()) {
      names += "、";
    }
    names += descriptors[index].joint_name;
  }
  if (names.empty()) {
    return true;
  }
  if (missing != nullptr) {
    *missing = names;
  }
  return false;
}

Result SoemEthercatTransport::enterZeroGravity()
{
  if (!gravity_ready_) {
    return Result::fail(CoreError::kInvalidConfiguration, "没有 URDF 重力模型，不能做零重力");
  }
  std::string missing;
  if (!rightArmRatedTorqueReady(&missing)) {
    return Result::fail(
      CoreError::kInvalidConfiguration,
      "右臂没有读到额定力矩 0x6076，不能把牛·米换成驱动器千分比：" + missing);
  }
  return beginRightArmTorque(0x7F00, 0.0, mit_kd_, 1.0, true);
}

Result SoemEthercatTransport::beginRightArmTorque(
  std::uint16_t mask, double kp_permille, double kd_permille, double ff_scale,
  bool zero_gravity)
{
  if (!active_ || link_ == nullptr) {
    return Result::fail(CoreError::kTransportUnavailable, "EtherCAT 传输层尚未激活");
  }
  for (bool torque : torque_axis_) {
    if (torque) {
      return Result::fail(CoreError::kInvalidConfiguration, "已有轴用纯扭矩映射启动，不能再切 MIT");
    }
  }
  // 只允许右臂 r1–r7。左臂和腰没有 MIT 力矩通路，误配掩码会变成"以为在调右臂"。
  if (mask == 0 || (mask & ~0x7F00u) != 0) {
    return Result::fail(CoreError::kInvalidConfiguration, "MIT 掩码必须非空且只包含右臂 bit8..14");
  }
  if (!std::isfinite(kp_permille) || !std::isfinite(kd_permille) || !std::isfinite(ff_scale) ||
    kp_permille < 0.0 || kp_permille > 2000.0 || kd_permille < 0.0 || kd_permille > 200.0 ||
    ff_scale < 0.0 || ff_scale > 1.0)
  {
    return Result::fail(
      CoreError::kInvalidConfiguration,
      "MIT 参数越界：kp 0..2000‰/rad，kd 0..200‰/(rad·s)，ff_scale 0..1");
  }
  // 校验和状态准备都在锁内完成；requestDriveMode 在 IgH 上是阻塞邮箱调用，
  // 必须放锁后再发，否则会和等锁的周期线程互锁（服务无响应）。
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    if (!drives_enabled_) {
      return Result::fail(CoreError::kWriteFailed, "驱动器未使能，不能进入 MIT");
    }
    if (mit_active_) {return Result::fail(CoreError::kWriteFailed, "已有扭矩模式占用，请先退出");}
    mit_mask_ = mask;
    mit_kp_ = kp_permille;
    mit_kd_ = kd_permille;
    mit_ff_scale_ = ff_scale;
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
    mit_zero_g_ = zero_gravity;
    mit_active_ = true;
  }
  link_->requestDriveMode(kZeroErrCyclicTorqueMode, mask);
  if (zero_gravity) {
    return Result{
      CoreError::kOk,
      "右臂零重力：重力前馈和关节偏置直接写进周期同步扭矩。请扶着右臂；松手后下沉或上抬，说明补偿不对"};
  }
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
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    if (shutdown_outputs_) {
      return Result::fail(CoreError::kWriteFailed, "已请求关断，不能把退出请求当成位置保持成功");
    }
    current_active_ = false;
    current_handfeel_ = false;
    current_zero_gravity_ = false;
    for (int side = 0; side < 2; ++side) {
      if (current_bundle_.present[side]) {current_models_[side].setPoseDomainEnforced(true);}
    }
    mit_active_ = false;
    mit_zero_g_ = false;
    mit_track_qdes_ = false;
    mit_soft_hold_ = false;
    motion_armed_ = false;
    gravity_calib_stop_.store(true);
    for (const auto & item : latest_) {
      if (!item.valid || (item.status_word & 0x006F) != 0x0027 ||
        std::chrono::steady_clock::now() - last_cycle_at_ > std::chrono::milliseconds(100))
      {
        shutdown_outputs_ = true;
        return Result::fail(CoreError::kReadFailed, "退出时反馈无效，已请求关断，不能确认位置保持");
      }
    }
    for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
      held_pulse_[index] = latest_[index].actual_position;
      goal_pulse_[index] = latest_[index].actual_position;
    }
  }
  // 同上：阻塞 SDO 放到锁外，周期线程才能继续跑。
  link_->requestDriveMode(kZeroErrCyclicPositionMode, mit_mask_);
  return Result{CoreError::kOk, "已请求退出扭矩模式并锁存当前位置；需确认CSP反馈和停稳"};
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

bool SoemEthercatTransport::writeRightArmGravityFile(
  const std::array<double, 7> & mass_scale,
  const std::array<std::array<double, 3>, 7> & com_delta) const
{
  if (right_arm_gravity_yaml_.empty()) {
    return false;
  }
  std::ofstream output(right_arm_gravity_yaml_);
  if (!output) {
    return false;
  }
  output << "# 右臂 r1 到 r7。质量倍率相对 URDF，质心偏移单位米。\n";
  output << "mass_scale:";
  for (double value : mass_scale) {
    output << " " << value;
  }
  output << "\n";
  const char * keys[3] = {"com_dx:", "com_dy:", "com_dz:"};
  for (int axis = 0; axis < 3; ++axis) {
    output << keys[axis];
    for (const auto & delta : com_delta) {
      output << " " << delta[axis];
    }
    output << "\n";
  }
  output << "joint_scale:";
  for (double value : right_joint_scale_) {
    output << " " << (value > 0.0 ? value : 1.0);
  }
  output << "\njoint_bias:";
  for (double value : right_joint_bias_) {
    output << " " << value;
  }
  output << "\n# 零差单关节残余，单位 N·m。0 表示只用 URDF。\nsin_amplitude:";
  for (int slot = 0; slot < 7; ++slot) {
    output << " " << gravity_amplitude_nm_[static_cast<std::size_t>(8 + slot)];
  }
  output << "\n";
  return true;
}

Result SoemEthercatTransport::resetRightArmGravityToUrdf()
{
  if (!gravity_ready_) {
    return Result::fail(CoreError::kInvalidConfiguration, "没有 URDF 重力模型");
  }
  std::array<double, 7> scale{};
  scale.fill(1.0);
  std::array<std::array<double, 3>, 7> delta{};
  if (!gravity_.setRightArmCorrection(scale, delta)) {
    return Result::fail(CoreError::kInvalidConfiguration, "右臂连杆没有对上 URDF");
  }
  rememberRightArmCorrection(scale, delta);
  right_joint_scale_.fill(1.0);
  right_joint_bias_.fill(0.0);
  for (int slot = 0; slot < 7; ++slot) {
    gravity_amplitude_nm_[static_cast<std::size_t>(8 + slot)] = 0.0;
  }
  say("✓ 右臂重力已在内存恢复为 URDF 初值，尚未写入文件");
  return Result{CoreError::kOk, "右臂已恢复 URDF 初值，关节倍率和偏置也已清掉。按 p p 才会写入文件"};
}

void SoemEthercatTransport::rememberRightArmCorrection(
  const std::array<double, 7> & mass_scale,
  const std::array<std::array<double, 3>, 7> & com_delta)
{
  right_mass_scale_ = mass_scale;
  right_com_delta_ = com_delta;
  right_correction_known_ = true;
}

Result SoemEthercatTransport::saveRightArmGravity()
{
  if (!right_correction_known_) {
    return Result::fail(CoreError::kInvalidConfiguration, "还没有可保存的右臂重力修正");
  }
  if (!writeRightArmGravityFile(right_mass_scale_, right_com_delta_)) {
    return Result::fail(CoreError::kWriteFailed, "右臂重力文件没有写成");
  }
  say("✓ 右臂重力已写入 %s", right_arm_gravity_yaml_.c_str());
  return Result{CoreError::kOk, "已写入 " + right_arm_gravity_yaml_};
}

Result SoemEthercatTransport::startRightArmGravityCalibration()
{
  if (!active_ || !gravity_ready_) {
    return Result::fail(CoreError::kTransportUnavailable, "重力模型还没准备好");
  }
  if (mit_active_) {
    return Result::fail(CoreError::kWriteFailed, "请先按 n 退出 MIT，右臂才能自己走到标定姿态");
  }
  std::string missing;
  if (!rightArmRatedTorqueReady(&missing)) {
    return Result::fail(
      CoreError::kInvalidConfiguration,
      "右臂没有读到额定力矩 0x6076，记录到的千分比不能换成牛·米：" + missing);
  }
  if (gravity_calib_running_.exchange(true)) {
    return Result::fail(CoreError::kWriteFailed, "右臂标定已经在进行");
  }
  if (gravity_calib_thread_.joinable()) {
    gravity_calib_thread_.join();
  }
  gravity_calib_stop_.store(false);
  gravity_calib_thread_ = std::thread([this]() {runRightArmGravityCalibration();});
  return Result{CoreError::kOk, "右臂进入周期同步扭矩，从两侧软持采样。请让开右臂"};
}

void SoemEthercatTransport::runRightArmGravityCalibration()
{
  struct ClearRunning
  {
    std::atomic<bool> * flag;
    ~ClearRunning() {flag->store(false);}
  } clear{&gravity_calib_running_};
  struct LeaveTorque
  {
    SoemEthercatTransport * self;
    bool armed{false};
    ~LeaveTorque()
    {
      if (armed) {
        self->exitMitHold();
      }
    }
  } leave{this, false};

  const Result entered = beginRightArmTorque(0x7F00, mit_kp_, mit_kd_, 1.0, false);
  if (!entered) {
    say("✗ 右臂不能进入周期同步扭矩：%s", entered.detail.c_str());
    return;
  }
  leave.armed = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (gravity_calib_stop_.load()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(sample_mu_);
    if (!mit_active_) {
      return;
    }
    mit_track_qdes_ = true;
    mit_soft_hold_ = false;
    for (int joint = 0; joint < 7; ++joint) {
      const std::size_t index = static_cast<std::size_t>(8 + joint);
      mit_qdes_rad_[index] = latest_[index].valid ?
        pulseToPosition(index, latest_[index].actual_position) : 0.0;
    }
  }

  struct Pose { const char * name; double q[7]; };
  // 只到前方和侧方。肩不要转到身后，也不要举过头顶；这台手臂没有机械挡块。
  const Pose goals[] = {
    {"前伸", {0.30, -0.10, 0.00, 0.40, 0.00, 0.10, 0.00}},
    {"前屈肘", {0.55, -0.25, 0.30, 1.15, 0.30, 0.00, 0.20}},
    {"斜前", {0.95, -0.45, -0.30, 0.75, -0.30, 0.20, -0.20}},
    {"侧前平举", {1.10, -0.85, 0.00, 1.05, 0.00, 0.15, 0.00}},
    {"侧前屈肘", {0.85, -0.70, 0.40, 1.25, 0.45, -0.15, 0.30}},
    {"正侧伸", {0.15, -1.15, 0.00, 0.35, 0.00, 0.10, 0.00}},
    {"正侧半屈", {0.25, -1.00, 0.35, 0.80, 0.20, 0.20, 0.25}},
    {"正侧屈肘", {0.10, -1.30, -0.25, 1.20, -0.40, 0.10, -0.20}},
    {"侧平", {0.40, -1.40, 0.00, 0.85, 0.00, 0.00, 0.00}},
    {"侧平翻腕", {0.35, -1.20, 0.20, 0.95, 0.50, -0.25, 0.40}},
    {"前平", {0.70, -0.55, 0.00, 0.55, 0.00, 0.25, 0.00}},
    {"前伸翻腕", {0.40, -0.20, -0.20, 0.50, -0.45, 0.30, -0.35}},
    {"侧伸", {0.05, -1.40, 0.10, 0.45, 0.10, -0.10, 0.10}},
    {"斜侧", {0.75, -1.10, -0.20, 1.10, 0.20, 0.10, 0.20}},
    {"侧前略收", {0.60, -0.60, 0.15, 1.20, 0.15, 0.15, 0.15}},
    {"侧下", {-0.90, -0.35, 0.10, 0.55, 0.10, 0.10, 0.00}},
    {"侧向外", {-0.60, -1.05, 0.00, 0.90, 0.00, 0.15, 0.10}},
    {"侧前", {-0.25, -0.70, 0.20, 1.05, 0.20, 0.00, 0.00}},
  };
  const double approach[7] = {0.12, 0.08, 0.12, 0.12, 0.12, 0.10, 0.12};
  // r1=-1 rad 仍是侧下方，不是翻到身后。再往负超过约 -1.2 rad 才会绕到背后。
  const double side_lower[7] = {-1.20, -1.55, -0.65, 0.05, -0.75, -0.55, -0.65};
  const double side_upper[7] = {1.30, 0.08, 0.65, 1.42, 0.75, 0.55, 0.65};
  // 目标点仍只在身前。启动检查对肘和腕用 URDF 限位内收 0.05 rad。
  // 直肘 0.017 rad 或腕转到 1.6 rad 并不等于肩已经绕到身后。
  const double urdf_lower[7] = {-3.1416, -3.1416, -3.1416, -2.0944, -3.1416, -1.7453, -6.2832};
  const double urdf_upper[7] = {3.1416, 0.15, 3.1416, 1.5708, 3.1416, 1.7453, 6.2832};
  double start_lower[7];
  double start_upper[7];
  for (int joint = 0; joint < 7; ++joint) {
    start_lower[joint] = urdf_lower[joint] + 0.05;
    start_upper[joint] = urdf_upper[joint] - 0.05;
  }
  start_lower[0] = side_lower[0];
  start_upper[0] = side_upper[0];
  start_lower[1] = side_lower[1];
  start_upper[1] = side_upper[1];
  double lower[7];
  double upper[7];
  for (int joint = 0; joint < 7; ++joint) {
    lower[joint] = side_lower[joint] + approach[joint];
    upper[joint] = side_upper[joint] - approach[joint];
  }
  auto clamp_to = [](Pose pose, const double * lo, const double * hi) {
      for (int joint = 0; joint < 7; ++joint) {
        pose.q[joint] = std::min(hi[joint], std::max(lo[joint], pose.q[joint]));
      }
      return pose;
    };
  auto command_q = [&]() {
      std::array<double, 7> q{};
      std::lock_guard<std::mutex> lock(sample_mu_);
      for (int joint = 0; joint < 7; ++joint) {
        q[static_cast<std::size_t>(joint)] =
          mit_qdes_rad_[static_cast<std::size_t>(8 + joint)];
      }
      return q;
    };
  auto set_qdes = [&](const std::array<double, 7> & q) {
      std::lock_guard<std::mutex> lock(sample_mu_);
      for (int joint = 0; joint < 7; ++joint) {
        mit_qdes_rad_[static_cast<std::size_t>(8 + joint)] = q[static_cast<std::size_t>(joint)];
      }
    };
  auto stiffen = [&]() {
      std::lock_guard<std::mutex> lock(sample_mu_);
      mit_soft_hold_ = false;
      for (int joint = 0; joint < 7; ++joint) {
        const std::size_t index = static_cast<std::size_t>(8 + joint);
        if (latest_[index].valid) {
          mit_qdes_rad_[index] = pulseToPosition(index, latest_[index].actual_position);
        }
      }
    };
  struct Window
  {
    std::deque<std::pair<double, std::array<double, 7>>> samples;
    void push(double time, const std::array<double, 7> & q)
    {
      samples.push_back({time, q});
      while (samples.size() > 2 && time - samples[1].first >= 0.20) {
        samples.pop_front();
      }
    }
    double speed() const
    {
      if (samples.size() < 2) {
        return 1e9;
      }
      const double dt = samples.back().first - samples.front().first;
      if (dt < 0.15) {
        return 1e9;
      }
      double peak = 0.0;
      for (int joint = 0; joint < 7; ++joint) {
        peak = std::max(peak, std::abs(
          samples.back().second[static_cast<std::size_t>(joint)] -
          samples.front().second[static_cast<std::size_t>(joint)]) / dt);
      }
      return peak;
    }
  };
  auto measured_q = [&]() {
      std::array<double, 7> q{};
      std::lock_guard<std::mutex> lock(sample_mu_);
      for (int joint = 0; joint < 7; ++joint) {
        const std::size_t index = static_cast<std::size_t>(8 + joint);
        q[static_cast<std::size_t>(joint)] = latest_[index].valid ?
          pulseToPosition(index, latest_[index].actual_position) : mit_qdes_rad_[index];
      }
      return q;
    };
  auto glide = [&](const Pose & target, bool soft, double duration) -> bool {
      const std::array<double, 7> from = command_q();
      {
        std::lock_guard<std::mutex> lock(sample_mu_);
        mit_soft_hold_ = soft;
      }
      const auto began = std::chrono::steady_clock::now();
      while (!gravity_calib_stop_.load()) {
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - began).count();
        const double alpha = duration > 0.0 ? std::min(1.0, elapsed / duration) : 1.0;
        const double smooth = alpha * alpha * (3.0 - 2.0 * alpha);
        std::array<double, 7> next{};
        for (int joint = 0; joint < 7; ++joint) {
          next[static_cast<std::size_t>(joint)] = from[static_cast<std::size_t>(joint)] +
            (target.q[joint] - from[static_cast<std::size_t>(joint)]) * smooth;
        }
        set_qdes(next);
        if (alpha >= 1.0) {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return false;
    };
  auto wait_quiet = [&](const Pose & target, bool hard) -> bool {
      Window window;
      auto quiet_since = std::chrono::steady_clock::now();
      auto bad_since = quiet_since;
      bool bad = false;
      const auto began = quiet_since;
      const auto deadline = began + std::chrono::seconds(8);
      double error = 0.0;
      double speed = 0.0;
      int worst = 0;
      std::array<double, 7> joint_error{};
      std::array<double, 7> last_q{};
      while (!gravity_calib_stop_.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        last_q = measured_q();
        const double time = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - began).count();
        window.push(time, last_q);
        speed = window.speed();
        error = 0.0;
        bool on_approach_side = true;
        for (int joint = 0; joint < 7; ++joint) {
          joint_error[static_cast<std::size_t>(joint)] = std::abs(
            last_q[static_cast<std::size_t>(joint)] - target.q[joint]);
          if (joint_error[static_cast<std::size_t>(joint)] > error) {
            error = joint_error[static_cast<std::size_t>(joint)];
            worst = joint;
          }
          // 允许误差小于该轴接近量的 85%。0.053 rad 仍落在 0.08 rad 的接近侧。
          if (hard && joint_error[static_cast<std::size_t>(joint)] > approach[joint] * 0.85) {
            on_approach_side = false;
          }
        }
        const auto now = std::chrono::steady_clock::now();
        const double error_abort = hard ? 0.75 : 1.35;
        if (error > error_abort || speed > 1.5) {
          if (!bad) {
            bad = true;
            bad_since = now;
          } else if (std::chrono::duration<double>(now - bad_since).count() > 0.30) {
            say("✗ 右臂跟踪超限，标定停止。最大是 r%d，误差 %.3f rad，速度 %.3f rad/s",
              worst + 1, error, speed > 100.0 ? -1.0 : speed);
            gravity_calib_stop_.store(true);
            return false;
          }
        } else {
          bad = false;
        }
        const bool quiet = speed < 0.05 && (!hard || on_approach_side);
        if (!quiet) {
          quiet_since = now;
        } else if (std::chrono::duration<double>(now - quiet_since).count() >= 0.40) {
          return true;
        }
      }
      if (!gravity_calib_stop_.load()) {
        std::array<bool, 7> saturated{};
        {
          std::lock_guard<std::mutex> lock(sample_mu_);
          saturated = mit_balance_ok_;
        }
        const double allow = approach[worst] * 0.85;
        const double shown_speed = speed > 100.0 ? -1.0 : speed;
        say(hard ?
          "! 硬跟踪没有停稳。最大是 r%d，误差 %.3f rad，允许 %.3f，速度 %.3f rad/s" :
          "! 软持没有停稳。最大是 r%d，误差 %.3f rad，允许 %.3f，速度 %.3f rad/s",
          worst + 1, error, allow, shown_speed);
        say("! 七轴误差 rad: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
          joint_error[0], joint_error[1], joint_error[2], joint_error[3],
          joint_error[4], joint_error[5], joint_error[6]);
        say("! 目标 rad: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
          target.q[0], target.q[1], target.q[2], target.q[3],
          target.q[4], target.q[5], target.q[6]);
        say("! 实测 rad: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
          last_q[0], last_q[1], last_q[2], last_q[3], last_q[4], last_q[5], last_q[6]);
        say("! 力矩顶满: %s %s %s %s %s %s %s",
          saturated[0] ? "否" : "是", saturated[1] ? "否" : "是", saturated[2] ? "否" : "是",
          saturated[3] ? "否" : "是", saturated[4] ? "否" : "是", saturated[5] ? "否" : "是",
          saturated[6] ? "否" : "是");
      }
      return false;
    };
  auto record_side = [&](std::array<double, 7> * q_mean, std::array<double, 7> * y_mean) -> bool {
      Window window;
      std::array<double, 7> q_sum{};
      std::array<double, 7> y_sum{};
      int count = 0;
      const auto began = std::chrono::steady_clock::now();
      const auto until = began + std::chrono::seconds(3);
      while (!gravity_calib_stop_.load() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::array<double, 7> q{};
        std::array<double, 7> y{};
        bool usable = true;
        {
          std::lock_guard<std::mutex> lock(sample_mu_);
          for (int joint = 0; joint < 7; ++joint) {
            const std::size_t index = static_cast<std::size_t>(8 + joint);
            q[static_cast<std::size_t>(joint)] = latest_[index].valid ?
              pulseToPosition(index, latest_[index].actual_position) : 0.0;
            y[static_cast<std::size_t>(joint)] = mit_balance_nm_[static_cast<std::size_t>(joint)];
            usable = usable && mit_soft_hold_ && mit_balance_ok_[static_cast<std::size_t>(joint)];
          }
        }
        const double time = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - began).count();
        window.push(time, q);
        if (!usable || !(window.speed() < 0.05)) {
          continue;
        }
        for (int joint = 0; joint < 7; ++joint) {
          q_sum[static_cast<std::size_t>(joint)] += q[static_cast<std::size_t>(joint)];
          y_sum[static_cast<std::size_t>(joint)] += y[static_cast<std::size_t>(joint)];
        }
        ++count;
        if (count >= 50 && time >= 1.2) {
          break;
        }
      }
      if (count < 15) {
        return false;
      }
      for (int joint = 0; joint < 7; ++joint) {
        (*q_mean)[static_cast<std::size_t>(joint)] =
          q_sum[static_cast<std::size_t>(joint)] / count;
        (*y_mean)[static_cast<std::size_t>(joint)] =
          y_sum[static_cast<std::size_t>(joint)] / count;
      }
      return true;
    };

  const std::array<double, 7> start_q = command_q();
  for (int joint = 0; joint < 7; ++joint) {
    if (start_q[static_cast<std::size_t>(joint)] < start_lower[joint] ||
      start_q[static_cast<std::size_t>(joint)] > start_upper[joint])
    {
      say(joint < 2 ?
        "✗ 右臂 r%d 在 %.3f rad，允许 %.2f 到 %.2f。超出后会绕到身后或翻过头顶，标定不动" :
        "✗ 右臂 r%d 在 %.3f rad，允许 %.2f 到 %.2f。已贴近物理限位，标定不动",
        joint + 1, start_q[static_cast<std::size_t>(joint)],
        start_lower[joint], start_upper[joint]);
      return;
    }
  }
  say("● 右臂只在前方和侧方做软持，不会转到身后。请让开");
  std::vector<std::array<double, kCoffeeAxisCount>> poses;
  std::vector<std::array<double, 7>> torques;
  for (const Pose & goal_in : goals) {
    if (gravity_calib_stop_.load()) {
      break;
    }
    const Pose goal = clamp_to(goal_in, lower, upper);
    say("● 右臂换到%s", goal.name);
    std::array<double, 7> q_sum{};
    std::array<double, 7> y_sum{};
    int sides = 0;
    for (int sign : {1, -1}) {
      if (gravity_calib_stop_.load()) {
        break;
      }
      Pose side = goal;
      for (int joint = 0; joint < 7; ++joint) {
        side.q[joint] = goal.q[joint] + sign * approach[joint];
      }
      side = clamp_to(side, side_lower, side_upper);
      const std::array<double, 7> from = command_q();
      double farthest = 0.0;
      for (int joint = 0; joint < 7; ++joint) {
        farthest = std::max(
          farthest, std::abs(side.q[joint] - from[static_cast<std::size_t>(joint)]));
      }
      say("● 右臂硬跟踪到%s侧", sign > 0 ? "正" : "负");
      if (!glide(side, false, std::max(1.5, 1.5 * farthest / 0.12))) {
        break;
      }
      if (!wait_quiet(side, true)) {
        continue;
      }
      say("● 软持滑向航点");
      if (!glide(goal, true, 1.5)) {
        break;
      }
      if (!wait_quiet(goal, false)) {
        stiffen();
        continue;
      }
      std::array<double, 7> q{};
      std::array<double, 7> y{};
      const bool recorded = record_side(&q, &y);
      stiffen();
      if (!recorded) {
        say("! 软持样本不够，跳过这一侧");
        continue;
      }
      for (int joint = 0; joint < 7; ++joint) {
        q_sum[static_cast<std::size_t>(joint)] += q[static_cast<std::size_t>(joint)];
        y_sum[static_cast<std::size_t>(joint)] += y[static_cast<std::size_t>(joint)];
      }
      ++sides;
    }
    if (sides != 2) {
      say("! 这一姿态两侧没有都记到，跳过");
      continue;
    }
    std::array<double, kCoffeeAxisCount> full{};
    {
      std::lock_guard<std::mutex> lock(sample_mu_);
      for (std::size_t index = 0; index < kCoffeeAxisCount; ++index) {
        full[index] = latest_[index].valid ?
          pulseToPosition(index, latest_[index].actual_position) : 0.0;
      }
    }
    for (int joint = 0; joint < 7; ++joint) {
      full[static_cast<std::size_t>(8 + joint)] = 0.5 * q_sum[static_cast<std::size_t>(joint)];
      y_sum[static_cast<std::size_t>(joint)] *= 0.5;
    }
    poses.push_back(full);
    torques.push_back(y_sum);
    say("✓ 已记录一对软持力矩，累计 %zu 组", poses.size());
  }
  if (gravity_calib_stop_.load()) {
    say("✗ 标定被中止，不拟合");
    return;
  }
  leave.armed = false;
  exitMitHold();

  std::vector<std::array<double, kCoffeeAxisCount>> train_q;
  std::vector<std::array<double, kCoffeeAxisCount>> held_q;
  std::vector<std::array<double, 7>> train_y;
  std::vector<std::array<double, 7>> held_y;
  for (std::size_t index = 0; index < poses.size(); ++index) {
    if (index % 3 == 2) {
      held_q.push_back(poses[index]);
      held_y.push_back(torques[index]);
    } else {
      train_q.push_back(poses[index]);
      train_y.push_back(torques[index]);
    }
  }
  if (train_q.size() < 8 || held_q.size() < 3) {
    say("✗ 训练姿态 %zu 组、留出姿态 %zu 组，不够拟合", train_q.size(), held_q.size());
    return;
  }
  auto model_of = [&](const std::array<double, kCoffeeAxisCount> & q) {
      std::array<double, kCoffeeAxisCount> tau{};
      gravity_.torques(q, &tau);
      return tau;
    };
  auto rms = [&](int joint_only) {
      double square = 0.0;
      int count = 0;
      for (std::size_t index = 0; index < held_q.size(); ++index) {
        const auto tau = model_of(held_q[index]);
        for (int joint = 0; joint < 7; ++joint) {
          if (joint_only >= 0 && joint != joint_only) {
            continue;
          }
          const double error = held_y[index][static_cast<std::size_t>(joint)] -
            tau[static_cast<std::size_t>(8 + joint)];
          square += error * error;
          ++count;
        }
      }
      return count > 0 ? std::sqrt(square / count) : 1e9;
    };
  const double before_held = rms(-1);
  std::array<double, 7> saved_scale = right_mass_scale_;
  std::array<std::array<double, 3>, 7> saved_delta = right_com_delta_;
  std::array<double, 7> scale{};
  std::array<std::array<double, 3>, 7> delta{};
  std::string error;
  if (!gravity_.identifyRightArm(train_q, train_y, &scale, &delta, &error)) {
    say("✗ 右臂拟合失败：%s", error.c_str());
    return;
  }
  if (!gravity_.setRightArmCorrection(scale, delta)) {
    say("✗ 拟合结果没有写进模型");
    return;
  }
  const double after_held = rms(-1);
  if (!(after_held < before_held)) {
    gravity_.setRightArmCorrection(saved_scale, saved_delta);
    say("! 留出姿态误差 %.3f N·m 没有小于原来的 %.3f，连杆参数保持不变",
      after_held, before_held);
  } else {
    rememberRightArmCorrection(scale, delta);
    say("✓ 连杆参数留出误差 %.3f N·m，原来 %.3f", after_held, before_held);
  }
  int accepted = 0;
  for (int joint = 0; joint < 7; ++joint) {
    double ata[2][2]{};
    double atb[2]{};
    for (std::size_t index = 0; index < train_q.size(); ++index) {
      const auto tau = model_of(train_q[index]);
      const double gravity = tau[static_cast<std::size_t>(8 + joint)];
      const double measured = train_y[index][static_cast<std::size_t>(joint)];
      const double a0 = gravity / 0.05;
      const double a1 = 1.0 / 0.05;
      ata[0][0] += a0 * a0;
      ata[0][1] += a0 * a1;
      ata[1][0] += a1 * a0;
      ata[1][1] += a1 * a1;
      atb[0] += a0 * measured / 0.05;
      atb[1] += a1 * measured / 0.05;
    }
    ata[0][0] += 4.0;
    atb[0] += 4.0;
    ata[1][1] += 0.04;
    const double det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
    if (std::abs(det) < 1e-9) {
      continue;
    }
    double joint_scale = (atb[0] * ata[1][1] - ata[0][1] * atb[1]) / det;
    double joint_bias = (ata[0][0] * atb[1] - atb[0] * ata[1][0]) / det;
    joint_scale = std::min(1.2, std::max(0.8, joint_scale));
    joint_bias = std::min(0.08, std::max(-0.08, joint_bias));
    auto joint_rms = [&](double used_scale, double used_bias) {
        double square = 0.0;
        for (std::size_t index = 0; index < held_q.size(); ++index) {
          const auto tau = model_of(held_q[index]);
          const double predicted = used_scale * tau[static_cast<std::size_t>(8 + joint)] + used_bias;
          const double residual = held_y[index][static_cast<std::size_t>(joint)] - predicted;
          square += residual * residual;
        }
        return std::sqrt(square / static_cast<double>(held_q.size()));
      };
    const double previous_scale = right_joint_scale_[static_cast<std::size_t>(joint)] > 0.0 ?
      right_joint_scale_[static_cast<std::size_t>(joint)] : 1.0;
    const double previous = joint_rms(
      previous_scale, right_joint_bias_[static_cast<std::size_t>(joint)]);
    const double candidate = joint_rms(joint_scale, joint_bias);
    if (candidate < previous) {
      right_joint_scale_[static_cast<std::size_t>(joint)] = joint_scale;
      right_joint_bias_[static_cast<std::size_t>(joint)] = joint_bias;
      ++accepted;
    }
  }
  say("✓ 软持拟合结束。%d 个关节的倍率或偏置留出误差变小。按 v v 检验，按 p p 写入", accepted);
}

}  // namespace x1_coffee_hardware_core
