/* 只诊断 EtherCAT 安全操作(0x04)到运行(0x08)。
 * 不使能、不写目标位置、不清报警。输出缓冲区保持 0，即手册要求的安全输出。
 * 分布时钟与零差官方 IgH 示例一致：SYNC0 周期 1 ms，偏移 0。
 * 用法: ecat_safeop_diag [网口]
 * 默认 enp3s0。看到全部进入运行，或 15 秒到时，程序自己退出。
 */
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "soem/soem.h"

#define MOTOR_COUNT 15
#define CYCLE_NS 1000000L
#define DIAG_SEC 15

static ecx_contextt ctx;
static uint8_t iomap[8192];
static int opened;
static volatile sig_atomic_t stop_requested;

static const char *joint_name(int slave)
{
  static const char *names[MOTOR_COUNT] = {
    "Joint_Body",
    "Joint_l1", "Joint_l2", "Joint_l3", "Joint_l4", "Joint_l5", "Joint_l6", "Joint_l7",
    "Joint_r1", "Joint_r2", "Joint_r3", "Joint_r4", "Joint_r5", "Joint_r6", "Joint_r7"};
  if (slave < 1 || slave > MOTOR_COUNT) {
    return "?";
  }
  return names[slave - 1];
}

static const char *al_state_name(uint16_t state)
{
  switch (state & 0x0f) {
    case EC_STATE_INIT: return "初始化";
    case EC_STATE_PRE_OP: return "预操作";
    case EC_STATE_SAFE_OP: return "安全操作";
    case EC_STATE_OPERATIONAL: return "运行";
    default: return "其他";
  }
}

static void on_signal(int signo)
{
  (void)signo;
  stop_requested = 1;
}

static int64_t now_ns(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void sleep_until(int64_t deadline_ns)
{
  struct timespec ts;
  ts.tv_sec = (time_t)(deadline_ns / 1000000000LL);
  ts.tv_nsec = (long)(deadline_ns % 1000000000LL);
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* 只读 ESC。0x030C 处理单元错误计到 255 就停住，单看这个数不能判断还在增加。 */
static void log_link(int slave)
{
  uint16_t address = ctx.slavelist[slave].configadr;
  uint8_t errors[20];
  uint16_t watchdog = 0;
  uint32_t time_diff = 0;
  uint8_t sync_active = 0;
  int rc_errors;
  int rc_watchdog;
  int rc_diff;
  int rc_sync;
  memset(errors, 0, sizeof(errors));
  rc_errors = ecx_FPRD(
    &ctx.port, address, 0x0300, (uint16)sizeof(errors), errors, EC_TIMEOUTRET);
  rc_watchdog = ecx_FPRD(
    &ctx.port, address, 0x0440, sizeof(watchdog), &watchdog, EC_TIMEOUTRET);
  rc_diff = ecx_FPRD(
    &ctx.port, address, 0x092C, sizeof(time_diff), &time_diff, EC_TIMEOUTRET);
  rc_sync = ecx_FPRD(
    &ctx.port, address, 0x0981, sizeof(sync_active), &sync_active, EC_TIMEOUTRET);
  printf(
    "  链路 无效帧 %u/%u 接收错误 %u/%u 转发 %u/%u 处理 %u 掉线 %u "
    "看门狗 0x%04x 时钟偏差 %s%u ns SYNC0 0x%02x rc %d/%d/%d/%d\n",
    errors[0], errors[2], errors[1], errors[3], errors[8], errors[9], errors[12],
    errors[0x10], watchdog, (time_diff & 0x80000000u) ? "-" : "",
    time_diff & 0x7fffffffu, sync_active, rc_errors, rc_watchdog, rc_diff, rc_sync);
}

static void return_to_init(void)
{
  int n;
  if (!opened) {
    return;
  }
  for (n = 0; n < 20; ++n) {
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    usleep(1000);
  }
  ctx.slavelist[0].state = EC_STATE_INIT | EC_STATE_ACK;
  ecx_writestate(&ctx, 0);
  ecx_statecheck(&ctx, 0, EC_STATE_INIT, 200000);
  printf("总线已收回初始化，汇总状态 0x%04x\n", ctx.slavelist[0].state);
  fflush(stdout);
}

int main(int argc, char **argv)
{
  const char *ifname = (argc > 1) ? argv[1] : "enp3s0";
  int count;
  int slave;
  int cycle;
  int safeop_seen = 0;
  int primed = 0;
  int op_requested = 0;
  int expected_wkc = 0;
  int bad_wkc = 0;
  int64_t next_ns;
  int64_t start_ns;
  int64_t max_late_ns = 0;
  int reached[MOTOR_COUNT + 1];
  int logged_wait[MOTOR_COUNT + 1];

  memset(reached, 0, sizeof(reached));
  memset(logged_wait, 0, sizeof(logged_wait));
  printf("诊断安全操作 0x04。不使能，输出保持 0，周期 1 ms，SYNC0 偏移 0。\n");
  printf("网口 %s。最多 %d 秒。Ctrl-C 会先收回初始化再退出。\n", ifname, DIAG_SEC);
  fflush(stdout);

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  if (!ecx_init(&ctx, ifname)) {
    fprintf(stderr, "打不开网口 %s\n", ifname);
    return 1;
  }
  opened = 1;
  count = ecx_config_init(&ctx);
  printf("从站数 %d\n", count);
  fflush(stdout);
  if (count == EC_NOFRAME) {
    fprintf(stderr,
      "没有收到任何从站应答。网口能打开，但总线上没有帧回来。\n"
      "先停掉正在占用 enp3s0 的控制栈，确认网线链路是通的，再运行本程序。\n");
    ecx_close(&ctx);
    opened = 0;
    return 1;
  }
  if (count != MOTOR_COUNT) {
    fprintf(stderr, "从站数不是 15\n");
    return_to_init();
    ecx_close(&ctx);
    opened = 0;
    return 1;
  }
  /* 用从站 EEPROM 里的过程数据，不重写映射，避免诊断本身改配置。 */
  ecx_config_map_group(&ctx, iomap, 0);
  ecx_configdc(&ctx);
  for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
    ecx_dcsync0(&ctx, (uint16)slave, TRUE, (uint32)CYCLE_NS, 0);
    printf(
      "轴 %d %s 输出 %u 字节 输入 %u 字节 分布时钟 %d\n",
      slave, joint_name(slave), ctx.slavelist[slave].Obytes, ctx.slavelist[slave].Ibytes,
      ctx.slavelist[slave].hasdc ? 1 : 0);
  }
  expected_wkc = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;
  printf("期望工作计数 %d\n", expected_wkc);
  fflush(stdout);

  ctx.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&ctx, 0);
  start_ns = now_ns();
  next_ns = start_ns;
  for (cycle = 0; !stop_requested && (now_ns() - start_ns) < (int64_t)DIAG_SEC * 1000000000LL;
       ++cycle)
  {
    int64_t before_ns = now_ns();
    int late_ns;
    int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    int op_count = 0;
    if (wkc != expected_wkc) {
      ++bad_wkc;
    }
    ecx_send_processdata(&ctx);
    late_ns = (int)(now_ns() - before_ns);
    if (late_ns > max_late_ns) {
      max_late_ns = late_ns;
    }

    /* 100 ms 才读一次状态，避免把 1 ms 节拍撑开。 */
    if ((cycle % 100) == 0) {
      ecx_readstate(&ctx);
    }
    if (!safeop_seen && (ctx.slavelist[0].state & 0x0f) >= EC_STATE_SAFE_OP) {
      safeop_seen = 1;
      printf("进入安全操作，第 %d 拍，汇总状态 0x%04x\n", cycle, ctx.slavelist[0].state);
      fflush(stdout);
    }
    if (safeop_seen && !op_requested && ++primed >= 100) {
      ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
      ecx_writestate(&ctx, 0);
      op_requested = 1;
      printf("已连发 100 ms 安全输出，请求进入运行\n");
      fflush(stdout);
    }
    if (op_requested && (cycle % 500) == 0) {
      int waiting = 0;
      ecx_readstate(&ctx);
      for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
        uint16_t state = ctx.slavelist[slave].state;
        if (state == EC_STATE_OPERATIONAL) {
          if (!reached[slave]) {
            reached[slave] = 1;
            printf(
              "轴 %d %s 已进入运行，用时 %.1f 秒\n",
              slave, joint_name(slave), (now_ns() - start_ns) / 1.0e9);
          }
          ++op_count;
          continue;
        }
        ++waiting;
        if (logged_wait[slave] < 3) {
          ++logged_wait[slave];
          printf(
            "轴 %d %s 状态 0x%02x %s%s AL码 0x%04x\n",
            slave, joint_name(slave), state, al_state_name(state),
            (state & EC_STATE_ERROR) ? " 带错误" : "",
            ctx.slavelist[slave].ALstatuscode);
          log_link(slave);
        }
      }
      printf(
        "运行 %d/15，工作计数不符 %d 帧，本段收发最慢 %lld 微秒\n",
        op_count, bad_wkc, (long long)(max_late_ns / 1000));
      fflush(stdout);
      bad_wkc = 0;
      max_late_ns = 0;
      if (waiting == 0) {
        printf("15 台都已进入运行\n");
        break;
      }
    }
    next_ns += CYCLE_NS;
    sleep_until(next_ns);
  }

  ecx_readstate(&ctx);
  printf("--- 结束 ---\n");
  for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
    uint16_t state = ctx.slavelist[slave].state;
    printf(
      "轴 %d %s 状态 0x%02x %s AL码 0x%04x %s\n",
      slave, joint_name(slave), state, al_state_name(state),
      ctx.slavelist[slave].ALstatuscode,
      state == EC_STATE_OPERATIONAL ? "已进入运行" : "仍未进入运行");
  }
  fflush(stdout);
  return_to_init();
  ecx_close(&ctx);
  opened = 0;
  return 0;
}
