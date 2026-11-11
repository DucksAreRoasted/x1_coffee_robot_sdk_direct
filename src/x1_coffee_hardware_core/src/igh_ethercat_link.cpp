// IgH（EtherLab）总线实现。对应 EthercatLink，被 SoemEthercatTransport 复用。
//
// 与 igh_init_probe 的区别：探测器只做「上电 + 保持」，这里还要按 1 ms 持续交换
// 完整的 ZeroErr PDO（Rx 10 字节 / Tx 16 字节），把反馈和指令交给策略层，
// 让 ROS2 control、/x1/pvt_move 和 MoveIt 走同一条周期。

#include "x1_coffee_hardware_core/igh_ethercat_link.hpp"

#include <ecrt.h>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include "x1_coffee_hardware_core/zeroerr_drive.hpp"

namespace x1_coffee_hardware_core
{
namespace
{

constexpr std::size_t kAxes = kCoffeeAxisCount;
constexpr std::uint32_t kVendorId = 0x5a65726f;
constexpr std::uint32_t kProductCode = 0x29252;
constexpr std::uint32_t kCycleNs = 1000000;
// 厂商 Gecat_15axis.eni 的 slave_dc：assignactive 0x300、SYNC0 1 ms、位移 100 µs。
constexpr std::int32_t kSync0ShiftNs = 100000;
constexpr std::uint16_t kAssignActivate = 0x0300;
// 15 台各一个输出 FMMU 加一个输入 FMMU。
constexpr unsigned int kExpectedWorkingCounter = static_cast<unsigned int>(kAxes) * 3u;
// IgH 逐台配置 SDO 和 DC，实测 15 台约 14 秒，给足余量。
constexpr int kOpenTimeoutSec = 60;
// 退使能时多跑几十拍 0x0006，让驱动器先看到关机字再断主站。
constexpr int kShutdownCycles = 50;
// 与 SOEM 路径和启动脚本一致的 EtherCAT 专用核。
constexpr int kBusCpu = 3;

void say(const char * format, ...)
{
  std::va_list args;
  va_start(args, format);
  std::fprintf(stderr, "● ");
  std::vfprintf(stderr, format, args);
  std::fprintf(stderr, "\n");
  va_end(args);
}

// EtherLab 应用时间：2000-01-01 纪元，必须用实时时钟换算。
std::uint64_t epochNs()
{
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return (static_cast<std::uint64_t>(ts.tv_sec) - 946684800ULL) * 1000000000ULL +
         static_cast<std::uint64_t>(ts.tv_nsec);
}

void sleepCycle(timespec * next)
{
  next->tv_nsec += static_cast<long>(kCycleNs);
  while (next->tv_nsec >= 1000000000L) {
    next->tv_nsec -= 1000000000L;
    ++next->tv_sec;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, nullptr);
}

// 与 zeroerr_drive.hpp 的 ZeroErrOutput / ZeroErrInput 逐项对应。
const ec_pdo_entry_info_t kRxEntries[] = {
  {0x6040, 0x00, 16},  // 控制字
  {0x607A, 0x00, 32},  // 目标位置
  {0x6071, 0x00, 16},  // 目标力矩
  {0x6060, 0x00, 8},   // 运行模式
  {0x0000, 0x00, 8},   // 模式字节后的填充必须放最后
};
const ec_pdo_entry_info_t kTxEntries[] = {
  {0x6041, 0x00, 16},  // 状态字
  {0x6064, 0x00, 32},  // 实际位置
  {0x606C, 0x00, 32},  // 实际速度
  {0x6077, 0x00, 16},  // 实际力矩
  {0x6078, 0x00, 16},  // 实际电流
  {0x6061, 0x00, 8},   // 实际模式
  {0x0000, 0x00, 8},
};
const ec_pdo_info_t kPdos[] = {
  {0x1600, 5, kRxEntries},
  {0x1A00, 7, kTxEntries},
};
const ec_sync_info_t kSyncs[] = {
  {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
  {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 1, &kPdos[0], EC_WD_ENABLE},
  {3, EC_DIR_INPUT, 1, &kPdos[1], EC_WD_DISABLE},
  {0xff},
};

constexpr std::size_t kRxCount = sizeof(kRxEntries) / sizeof(kRxEntries[0]);
constexpr std::size_t kTxCount = sizeof(kTxEntries) / sizeof(kTxEntries[0]);

}  // namespace

struct IghEthercatLink::Impl
{
  ec_master_t * master{nullptr};
  ec_domain_t * domain{nullptr};
  ec_slave_config_t * sc[kAxes]{};
  unsigned int rx_offset[kAxes][kRxCount]{};
  unsigned int tx_offset[kAxes][kTxCount]{};
  std::uint8_t * data{nullptr};

  std::thread thread;
  std::atomic<bool> stop{false};
  std::atomic<bool> thread_done{false};
  std::mutex wait_mu;
  std::condition_variable wait_cv;
  bool op_seen{false};
  bool failed{false};
  std::string failure_detail;

  std::atomic<bool> * bus_flag{nullptr};
  std::atomic<bool> * cyclic_flag{nullptr};
  std::function<void()> hook;
  Cycle on_cycle;

  bool opened{false};
  bool activated{false};

  void clearRuntime()
  {
    stop.store(false);
    thread_done.store(false);
    {
      std::lock_guard<std::mutex> lock(wait_mu);
      op_seen = false;
      failed = false;
      failure_detail.clear();
    }
  }

  void reportFailure(const std::string & detail)
  {
    {
      std::lock_guard<std::mutex> lock(wait_mu);
      failed = true;
      failure_detail = detail;
    }
    wait_cv.notify_all();
  }

  void markOperational()
  {
    // 顺序与 SOEM 一致：先放开使能节拍，再起 5 秒准备计时，最后宣布总线就绪。
    if (cyclic_flag != nullptr) {
      cyclic_flag->store(true, std::memory_order_release);
    }
    if (hook) {
      hook();
    }
    if (bus_flag != nullptr) {
      bus_flag->store(true, std::memory_order_release);
    }
    {
      std::lock_guard<std::mutex> lock(wait_mu);
      op_seen = true;
    }
    wait_cv.notify_all();
  }

  void cycle()
  {
    // 周期线程尽量独占 CPU 3 并提成 SCHED_FIFO。启动脚本可能只放开 CPU 2，
    // 绑不上就留在继承的 CPU 上继续跑，不因此拒绝上电。
    cpu_set_t allowed{};
    if (pthread_getaffinity_np(pthread_self(), sizeof(allowed), &allowed) == 0 &&
      CPU_ISSET(kBusCpu, &allowed) != 0)
    {
      cpu_set_t want{};
      CPU_ZERO(&want);
      CPU_SET(kBusCpu, &want);
      if (pthread_setaffinity_np(pthread_self(), sizeof(want), &want) == 0) {
        say("✓ IgH 周期线程已绑定 CPU %d", kBusCpu);
      }
    } else {
      say("! 进程没有放开 CPU %d，IgH 周期线程留在继承的 CPU 上", kBusCpu);
    }
    sched_param priority{};
    priority.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &priority) == 0) {
      say("✓ IgH 周期线程已设为实时优先级 80");
    } else {
      say("! IgH 周期线程没能设为实时优先级，1 ms 节拍可能有抖动");
    }
    // 锁住已映射内存，避免上电阶段的缺页把 1 ms 节拍打断。
    (void)mlockall(MCL_CURRENT | MCL_FUTURE);

    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    ec_domain_state_t ds{};
    ec_master_state_t ms{};
    bool announced = false;
    int mixed_polls = 0;
    int shutdown_cycles = 0;
    int wait_cycles = 0;
    int link_down_cycles = 0;

    for (;;) {
      const bool stopping = stop.load(std::memory_order_acquire);
      if (stopping && ++shutdown_cycles > kShutdownCycles) {
        break;
      }

      ecrt_master_application_time(master, epochNs());
      ecrt_master_receive(master);
      ecrt_domain_process(domain);
      ecrt_domain_state(domain, &ds);
      ecrt_master_state(master, &ms);

      const bool complete = ms.link_up && ds.wc_state == EC_WC_COMPLETE &&
        ds.working_counter == kExpectedWorkingCounter;
      const bool all_op = ms.al_states == static_cast<unsigned int>(EC_AL_STATE_OP);

      std::array<SlaveFeedback, kAxes> feedback{};
      for (std::size_t i = 0; i < kAxes; ++i) {
        SlaveFeedback & item = feedback[i];
        // 整个域只有一个 LRW 数据报，要么整帧有效，要么整帧作废。
        item.valid = complete;
        if (!complete) {
          continue;
        }
        item.status_word = EC_READ_U16(data + tx_offset[i][0]);
        item.actual_position = EC_READ_S32(data + tx_offset[i][1]);
        item.actual_torque = EC_READ_S16(data + tx_offset[i][3]);
        item.actual_current = EC_READ_S16(data + tx_offset[i][4]);
        item.mode_of_operation = EC_READ_U8(data + tx_offset[i][5]);
      }

      std::array<SlaveCommand, kAxes> command{};
      if (on_cycle) {
        on_cycle(feedback, &command);
      }
      if (stopping) {
        // 退使能：所有轴只发 CiA-402 Shutdown，不再接受任何新目标。
        for (std::size_t i = 0; i < kAxes; ++i) {
          command[i].control_word = 0x0006;
        }
      }

      for (std::size_t i = 0; i < kAxes; ++i) {
        EC_WRITE_U16(data + rx_offset[i][0], command[i].control_word);
        EC_WRITE_S32(data + rx_offset[i][1], command[i].target_position);
        EC_WRITE_S16(data + rx_offset[i][2], command[i].target_torque);
        EC_WRITE_U8(data + rx_offset[i][3], command[i].mode_of_operation);
      }

      ecrt_master_sync_reference_clock(master);
      ecrt_master_sync_slave_clocks(master);
      ecrt_domain_queue(domain);
      ecrt_master_send(master);

      if (!announced && complete && all_op) {
        announced = true;
        say("✓ 15 台从站进入运行，工作计数 %u/%u", ds.working_counter, kExpectedWorkingCounter);
        markOperational();
      } else if (!announced) {
        // 启动阶段每 2 秒报一次进度，直接说清是「没进 OP」还是「链路/从站掉了」。
        if (++wait_cycles >= 2000) {
          wait_cycles = 0;
          int op_count = 0;
          int online_count = 0;
          for (std::size_t i = 0; i < kAxes; ++i) {
            ec_slave_config_state_t state{};
            if (ecrt_slave_config_state(sc[i], &state) != 0) {
              continue;
            }
            if (state.online != 0u) {
              ++online_count;
            }
            if (state.al_state == EC_AL_STATE_OP) {
              ++op_count;
            }
          }
          say(
            "… 等待运行：OP %d/%zu，在线 %d/%zu，工作计数 %u/%u，AL 位图 0x%02x",
            op_count, kAxes, online_count, kAxes,
            ds.working_counter, kExpectedWorkingCounter, ms.al_states);
          if (online_count < static_cast<int>(kAxes)) {
            say("! 有从站离线，先查链路灯和网线，再查 dmesg 里的 AL 状态码");
          }
        }
        // 链路断开超过 2 秒就没必要再等到超时，直接让上层拿到原因。
        if (ms.link_up == 0u) {
          if (++link_down_cycles >= 2000) {
            reportFailure("IgH: 主站链路断开超过 2 秒，检查 EtherCAT 线缆和供电");
            break;
          }
        } else {
          link_down_cycles = 0;
        }
      }

      if (announced) {
        // 连续两秒不同时处于运行才锁住使能，一次读失败不算掉线。
        if (!complete || !all_op) {
          if (++mixed_polls >= 2000) {
            mixed_polls = 0;
            if (bus_flag != nullptr &&
              bus_flag->exchange(false, std::memory_order_acq_rel))
            {
              say(
                "! 15 台不再同时处于运行，已锁住使能（工作计数 %u/%u，AL 0x%02x）",
                ds.working_counter, kExpectedWorkingCounter, ms.al_states);
            }
          }
        } else {
          mixed_polls = 0;
        }
      }

      sleepCycle(&next);
    }
    thread_done.store(true, std::memory_order_release);
  }
};

IghEthercatLink::IghEthercatLink()
: impl_(std::make_unique<Impl>())
{
}

IghEthercatLink::~IghEthercatLink()
{
  shutdownToInit();
}

Result IghEthercatLink::openAndReachOp(
  const std::string & ifname,
  const std::array<bool, kCoffeeAxisCount> & torque_axis,
  const Cycle & on_cycle)
{
  Impl & s = *impl_;
  if (s.opened) {
    return Result::fail(CoreError::kTransportUnavailable, "IgH 主站已经打开");
  }
  // 阶段一先只支持位置轴：力矩轴的 PDO 布局和使能节拍还没搬过来。
  for (std::size_t i = 0; i < kAxes; ++i) {
    if (torque_axis[i]) {
      return Result::fail(
        CoreError::kInvalidConfiguration,
        "IgH 后端暂不支持力矩轴，轴 " + std::to_string(i + 1) + " 需要先改回 CSP");
    }
  }
  s.clearRuntime();
  if (s.bus_flag != nullptr) {
    s.bus_flag->store(false, std::memory_order_release);
  }
  s.on_cycle = on_cycle;
  say("● 请求 IgH 主站 0（网口由 main_devices 绑定，忽略 %s）", ifname.c_str());

  s.master = ecrt_request_master(0);
  if (s.master == nullptr) {
    return Result::fail(
      CoreError::kEthercatNotReady,
      "IgH: 请求主站失败，通常是有别的进程占着主站，或 ec_master 没加载");
  }
  s.domain = ecrt_master_create_domain(s.master);
  if (s.domain == nullptr) {
    ecrt_release_master(s.master);
    s.master = nullptr;
    return Result::fail(CoreError::kEthercatNotReady, "IgH: 创建过程数据域失败");
  }

  for (std::size_t i = 0; i < kAxes; ++i) {
    s.sc[i] = ecrt_master_slave_config(
      s.master, 0, static_cast<std::uint16_t>(i), kVendorId, kProductCode);
    if (s.sc[i] == nullptr) {
      ecrt_release_master(s.master);
      s.master = nullptr;
      return Result::fail(
        CoreError::kEthercatNotReady, "IgH: 从站 " + std::to_string(i + 1) + " 配置失败");
    }
    if (ecrt_slave_config_pdos(s.sc[i], EC_END, kSyncs) != 0) {
      ecrt_release_master(s.master);
      s.master = nullptr;
      return Result::fail(
        CoreError::kEthercatNotReady,
        "IgH: 从站 " + std::to_string(i + 1) + " PDO 配置失败");
    }
    // 先锁存当前位置再切 CSP：目标位置不写 0，避免多圈关节被拉向脉冲 0。
    if (ecrt_slave_config_sdo8(s.sc[i], 0x6060, 0, kZeroErrCyclicPositionMode) != 0 ||
      ecrt_slave_config_sdo16(s.sc[i], 0x1C32, 0x01, 0x0002) != 0 ||
      ecrt_slave_config_sdo16(s.sc[i], 0x1C33, 0x01, 0x0002) != 0)
    {
      ecrt_release_master(s.master);
      s.master = nullptr;
      return Result::fail(
        CoreError::kEthercatNotReady,
        "IgH: 从站 " + std::to_string(i + 1) + " 启动 SDO 配置失败");
    }
    for (std::size_t k = 0; k < kRxCount; ++k) {
      const int offset = ecrt_slave_config_reg_pdo_entry_pos(
        s.sc[i], 2, 0, static_cast<unsigned int>(k), s.domain, nullptr);
      if (offset < 0) {
        ecrt_release_master(s.master);
        s.master = nullptr;
        return Result::fail(
          CoreError::kEthercatNotReady,
          "IgH: 从站 " + std::to_string(i + 1) + " 输出 PDO 第 " + std::to_string(k) +
          " 项注册失败");
      }
      s.rx_offset[i][k] = static_cast<unsigned int>(offset);
    }
    for (std::size_t k = 0; k < kTxCount; ++k) {
      const int offset = ecrt_slave_config_reg_pdo_entry_pos(
        s.sc[i], 3, 0, static_cast<unsigned int>(k), s.domain, nullptr);
      if (offset < 0) {
        ecrt_release_master(s.master);
        s.master = nullptr;
        return Result::fail(
          CoreError::kEthercatNotReady,
          "IgH: 从站 " + std::to_string(i + 1) + " 输入 PDO 第 " + std::to_string(k) +
          " 项注册失败");
      }
      s.tx_offset[i][k] = static_cast<unsigned int>(offset);
    }
    if (ecrt_slave_config_dc(
        s.sc[i], kAssignActivate, kCycleNs, kSync0ShiftNs, 0, 0) != 0)
    {
      ecrt_release_master(s.master);
      s.master = nullptr;
      return Result::fail(
        CoreError::kEthercatNotReady, "IgH: 从站 " + std::to_string(i + 1) + " DC 配置失败");
    }
  }

  // IgH 会在这里阻塞逐台下发 SDO、SM/FMMU 和 DC，实测 15 台约 14 秒。
  say("● 正在配置 15 台从站并请求运行，IgH 逐台配置，约 15 秒，期间不要中断");
  if (ecrt_master_activate(s.master) != 0) {
    ecrt_release_master(s.master);
    s.master = nullptr;
    return Result::fail(CoreError::kEthercatNotReady, "IgH: 激活主站失败");
  }
  s.activated = true;
  s.data = ecrt_domain_data(s.domain);
  if (s.data == nullptr) {
    shutdownToInit();
    return Result::fail(CoreError::kEthercatNotReady, "IgH: 取过程数据指针失败");
  }

  s.opened = true;
  s.thread = std::thread([&s]() {s.cycle();});

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::seconds(kOpenTimeoutSec);
  bool ok = false;
  std::string detail;
  {
    std::unique_lock<std::mutex> lock(s.wait_mu);
    while (std::chrono::steady_clock::now() < deadline) {
      s.wait_cv.wait_for(lock, std::chrono::milliseconds(200), [&s]() {
          return s.op_seen || s.failed;
        });
      if (s.op_seen || s.failed) {
        ok = s.op_seen;
        detail = s.failure_detail;
        break;
      }
    }
    if (!ok && detail.empty()) {
      detail = "IgH: " + std::to_string(kOpenTimeoutSec) + " 秒内没有 15 台同时进入运行";
    }
  }
  if (!ok) {
    shutdownToInit();
    return Result::fail(CoreError::kEthercatNotReady, detail);
  }
  say("✓ IgH 主站已进入运行，1 ms 周期通信已保持");
  return Result::ok();
}

Result IghEthercatLink::shutdownToInit()
{
  Impl & s = *impl_;
  s.stop.store(true, std::memory_order_release);
  if (s.thread.joinable()) {
    s.thread.join();
  }
  if (s.master != nullptr) {
    if (s.activated) {
      // 退到 PREOP，输出断开；不清故障，不写零点。
      ecrt_master_deactivate(s.master);
    }
    ecrt_release_master(s.master);
    s.master = nullptr;
  }
  s.domain = nullptr;
  s.data = nullptr;
  s.activated = false;
  s.opened = false;
  if (s.bus_flag != nullptr) {
    s.bus_flag->store(false, std::memory_order_release);
  }
  return Result::ok();
}

void IghEthercatLink::requestDriveMode(std::uint8_t mode, std::uint16_t axis_mask)
{
  // 运行中切模式需要邮箱访问，阶段一不做；位置保持和 PVT 不依赖它。
  if (axis_mask != 0) {
    say("! IgH 后端暂不支持运行中切 0x6060=0x%02x 到 %u 台轴", mode, axis_mask);
  }
}

void IghEthercatLink::bindBusOperationalFlag(std::atomic<bool> * flag)
{
  impl_->bus_flag = flag;
}

void IghEthercatLink::bindCyclicEnableFlag(std::atomic<bool> * flag)
{
  impl_->cyclic_flag = flag;
  if (flag != nullptr) {
    flag->store(false, std::memory_order_release);
  }
}

void IghEthercatLink::bindOperationalHook(std::function<void()> hook)
{
  impl_->hook = std::move(hook);
}

}  // namespace x1_coffee_hardware_core
