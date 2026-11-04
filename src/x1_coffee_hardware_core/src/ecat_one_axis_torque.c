/* 直连 EtherCAT，只对一台电机做周期同步扭矩。
 * 手册 5.9：0x6060=0x0A，使能前 0x6071=0，控制字按状态走 0x0006/0x0007/0x000F，
 * 进入使能后再写目标扭矩。单位是额定电流的千分比。
 * 其余 14 台是轮廓位置，目标锁在当前编码器，控制字保持 0。
 * 用法: ecat_one_axis_torque [网口] [从站 1-15] [扭矩千分比] [秒]
 * 默认 eno1、从站 1（腰）、50、1 秒。绝对值不超过 150，时间不超过 3 秒。
 */
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "soem/soem.h"

#define MOTOR_COUNT 15
#define MAX_ABS_TORQUE 150

typedef struct __attribute__((packed))
{
  int32_t target_position;
  uint32_t digital_output;
  uint16_t control_word;
} PositionOutput;

typedef struct __attribute__((packed))
{
  int32_t actual_position;
  uint32_t digital_inputs;
  uint16_t status_word;
} PositionInput;

typedef struct __attribute__((packed))
{
  uint16_t control_word;
  int16_t target_torque;
} TorqueOutput;

typedef struct __attribute__((packed))
{
  uint16_t status_word;
  int16_t actual_torque;
  int16_t actual_current;
} TorqueInput;

static ecx_contextt ctx;
static uint8_t iomap[8192];
static int opened;
static int torque_slave = 1;
static volatile sig_atomic_t stop_requested;
static PositionOutput *position_out[MOTOR_COUNT + 1];
static PositionInput *position_in[MOTOR_COUNT + 1];
static TorqueOutput *torque_out;
static TorqueInput *torque_in;
static int held[MOTOR_COUNT + 1];

static const char *joint_name(int slave)
{
  static const char *names[MOTOR_COUNT] = {
    "Joint_Body",
    "Joint_l1", "Joint_l2", "Joint_l3", "Joint_l4", "Joint_l5", "Joint_l6", "Joint_l7",
    "Joint_r1", "Joint_r2", "Joint_r3", "Joint_r4", "Joint_r5", "Joint_r6", "Joint_r7"};
  if (slave < 1 || slave > MOTOR_COUNT) {
    return "unknown";
  }
  return names[slave - 1];
}

static uint16_t torque_control_word(uint16_t status)
{
  switch (status) {
    case 0x1208:
      return 0x0080;
    case 0x1250:
      return 0x0006;
    case 0x1231:
      return 0x0007;
    case 0x1233:
    case 0x1237:
    case 0x1637:
      return 0x000F;
    default:
      return 0x0006;
  }
}

static int operation_enabled(uint16_t status)
{
  return status == 0x1237 || status == 0x1637;
}

static int pdo_config(ecx_contextt *context, uint16 slave)
{
  uint8_t value8 = 0;
  uint32_t value32 = 0;
  int rc;
  if ((int)slave == torque_slave) {
    uint8_t mode = 0x0A;
    int16_t torque = 0;
    uint8_t display = 0;
    int display_size = (int)sizeof(display);
    rc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
    printf("slave=%u sdo 0x6060=0x0A rc=%d\n", slave, rc);
    rc = ecx_SDOread(context, slave, 0x6061, 0x00, FALSE, &display_size, &display, EC_TIMEOUTRXM);
    printf("slave=%u sdo 0x6061=0x%02x rc=%d\n", slave, display, rc);
    rc = ecx_SDOwrite(context, slave, 0x6071, 0x00, FALSE, sizeof(torque), &torque, EC_TIMEOUTRXM);
    printf("slave=%u sdo 0x6071=0 rc=%d\n", slave, rc);
    fflush(stdout);

    ecx_SDOwrite(context, slave, 0x1600, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value32 = 0x60400010;
    ecx_SDOwrite(context, slave, 0x1600, 1, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60710010;
    ecx_SDOwrite(context, slave, 0x1600, 2, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value8 = 2;
    ecx_SDOwrite(context, slave, 0x1600, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);

    value8 = 0;
    ecx_SDOwrite(context, slave, 0x1A00, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value32 = 0x60410010;
    ecx_SDOwrite(context, slave, 0x1A00, 1, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60770010;
    ecx_SDOwrite(context, slave, 0x1A00, 2, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60780010;
    ecx_SDOwrite(context, slave, 0x1A00, 3, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value8 = 3;
    rc = ecx_SDOwrite(context, slave, 0x1A00, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    printf("slave=%u torque pdo map rc=%d\n", slave, rc);
    fflush(stdout);
    return 1;
  }

  {
    uint8_t mode = 0x01;
    uint32_t profile_velocity = 30000;
    uint32_t profile_acceleration = 15566;
    uint32_t profile_deceleration = 15566;
    rc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
    printf("slave=%u sdo 0x6060=0x01 rc=%d\n", slave, rc);
    ecx_SDOwrite(context, slave, 0x6081, 0x00, FALSE, sizeof(profile_velocity),
                 &profile_velocity, EC_TIMEOUTRXM);
    ecx_SDOwrite(context, slave, 0x6083, 0x00, FALSE, sizeof(profile_acceleration),
                 &profile_acceleration, EC_TIMEOUTRXM);
    ecx_SDOwrite(context, slave, 0x6084, 0x00, FALSE, sizeof(profile_deceleration),
                 &profile_deceleration, EC_TIMEOUTRXM);
    ecx_SDOwrite(context, slave, 0x1600, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value32 = 0x607A0020;
    ecx_SDOwrite(context, slave, 0x1600, 1, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60FE0020;
    ecx_SDOwrite(context, slave, 0x1600, 2, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60400010;
    ecx_SDOwrite(context, slave, 0x1600, 3, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value8 = 3;
    ecx_SDOwrite(context, slave, 0x1600, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value8 = 0;
    ecx_SDOwrite(context, slave, 0x1A00, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value32 = 0x60640020;
    ecx_SDOwrite(context, slave, 0x1A00, 1, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60FD0020;
    ecx_SDOwrite(context, slave, 0x1A00, 2, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60410010;
    ecx_SDOwrite(context, slave, 0x1A00, 3, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value8 = 3;
    rc = ecx_SDOwrite(context, slave, 0x1A00, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    printf("slave=%u pdo map rc=%d\n", slave, rc);
    fflush(stdout);
  }
  return 1;
}

static void on_signal(int signo)
{
  (void)signo;
  stop_requested = 1;
}

static double monotonic_sec(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

static void command_cycle(int16_t torque, uint16_t moving_control)
{
  int slave;
  for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
    if (slave == torque_slave) {
      if (torque_out != NULL) {
        torque_out->target_torque = torque;
        torque_out->control_word = moving_control;
      }
      continue;
    }
    if (position_out[slave] != NULL) {
      position_out[slave]->digital_output = 0;
      position_out[slave]->control_word = 0;
    }
  }
}

static void return_to_init(void)
{
  int n;
  if (!opened) {
    return;
  }
  command_cycle(0, 0x0006);
  for (n = 0; n < 20; ++n) {
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    usleep(500);
  }
  ctx.slavelist[0].state = EC_STATE_INIT | EC_STATE_ACK;
  ecx_writestate(&ctx, 0);
  ecx_statecheck(&ctx, 0, EC_STATE_INIT, 200000);
  ecx_readstate(&ctx);
  printf("after-init slave0=0x%04x\n", ctx.slavelist[0].state);
  fflush(stdout);
}

int main(int argc, char **argv)
{
  const char *ifname = (argc > 1) ? argv[1] : "eno1";
  int requested_torque = (argc > 3) ? atoi(argv[3]) : 50;
  double hold_sec = (argc > 4) ? atof(argv[4]) : 1.0;
  int count;
  int slave;
  int cycle;
  int safeop_seen = 0;
  int op_requested = 0;
  int op_seen = 0;
  int enabled_seen = 0;
  int rc = 0;
  double enabled_at = 0.0;
  double op_at = 0.0;
  double last_print = 0.0;
  int16_t command_torque = 0;
  uint16_t moving_control = 0x0006;

  torque_slave = (argc > 2) ? atoi(argv[2]) : 1;
  if (torque_slave < 1 || torque_slave > MOTOR_COUNT) {
    fprintf(stderr, "从站号必须是 1 到 15\n");
    return 2;
  }
  if (requested_torque == 0 || requested_torque > MAX_ABS_TORQUE ||
      requested_torque < -MAX_ABS_TORQUE) {
    fprintf(stderr, "扭矩千分比必须非 0，且绝对值不超过 %d\n", MAX_ABS_TORQUE);
    return 2;
  }
  if (hold_sec <= 0.0 || hold_sec > 3.0) {
    fprintf(stderr, "出力时间必须在 0 到 3 秒之间\n");
    return 2;
  }

  printf("one-axis-torque ifname=%s slave=%d %s torque=%d permille seconds=%.2f\n",
         ifname, torque_slave, joint_name(torque_slave), requested_torque, hold_sec);
  printf("周期同步扭矩 0x0A。使能完成前目标扭矩为 0。其余轴不上电。\n");
  printf("请站在机器人旁边。该轴会持续出力，不会停在某个角度。\n");
  fflush(stdout);

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  if (!ecx_init(&ctx, ifname)) {
    fprintf(stderr, "ecx_init failed\n");
    return 1;
  }
  opened = 1;
  count = ecx_config_init(&ctx);
  printf("slavecount=%d\n", count);
  fflush(stdout);
  if (count != MOTOR_COUNT) {
    fprintf(stderr, "从站数不是 15\n");
    return_to_init();
    ecx_close(&ctx);
    return 1;
  }
  for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
    ctx.slavelist[slave].PO2SOconfig = pdo_config;
  }
  ecx_config_map_group(&ctx, iomap, 0);
  ecx_configdc(&ctx);
  for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
    ecx_dcsync0(&ctx, (uint16)slave, TRUE, 500000, 0);
    if (slave == torque_slave) {
      if (ctx.slavelist[slave].Obytes != (int)sizeof(TorqueOutput) ||
          ctx.slavelist[slave].Ibytes != (int)sizeof(TorqueInput) ||
          ctx.slavelist[slave].outputs == NULL || ctx.slavelist[slave].inputs == NULL) {
        fprintf(stderr, "slave=%d 扭矩 PDO 长度不符 obytes=%d ibytes=%d\n",
                slave, ctx.slavelist[slave].Obytes, ctx.slavelist[slave].Ibytes);
        return_to_init();
        ecx_close(&ctx);
        return 1;
      }
      torque_out = (TorqueOutput *)ctx.slavelist[slave].outputs;
      torque_in = (TorqueInput *)ctx.slavelist[slave].inputs;
      torque_out->control_word = 0;
      torque_out->target_torque = 0;
      continue;
    }
    if (ctx.slavelist[slave].Obytes != (int)sizeof(PositionOutput) ||
        ctx.slavelist[slave].Ibytes != (int)sizeof(PositionInput) ||
        ctx.slavelist[slave].outputs == NULL || ctx.slavelist[slave].inputs == NULL) {
      fprintf(stderr, "slave=%d 位置 PDO 长度不符 obytes=%d ibytes=%d\n",
              slave, ctx.slavelist[slave].Obytes, ctx.slavelist[slave].Ibytes);
      return_to_init();
      ecx_close(&ctx);
      return 1;
    }
    position_out[slave] = (PositionOutput *)ctx.slavelist[slave].outputs;
    position_in[slave] = (PositionInput *)ctx.slavelist[slave].inputs;
    position_out[slave]->control_word = 0;
    position_out[slave]->digital_output = 0;
    position_out[slave]->target_position = 0;
  }

  ctx.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&ctx, 0);
  last_print = monotonic_sec();
  for (cycle = 0; cycle < 30000 && !stop_requested; ++cycle) {
    int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    int ready = 0;
    uint16_t status = 0;
    for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
      if (slave == torque_slave) {
        if (wkc > 0 && torque_in != NULL && torque_in->status_word != 0) {
          held[slave] = 1;
          status = torque_in->status_word;
        }
      } else if (wkc > 0 && position_in[slave] != NULL &&
                 position_in[slave]->status_word != 0 && !held[slave]) {
        position_out[slave]->target_position = position_in[slave]->actual_position;
        held[slave] = 1;
      }
      if (held[slave]) {
        ++ready;
      }
    }
    if (!op_seen) {
      command_cycle(0, 0x0000);
    } else if (!enabled_seen) {
      moving_control = torque_control_word(status);
      command_torque = 0;
      command_cycle(command_torque, moving_control);
      if (operation_enabled(status)) {
        enabled_seen = 1;
        enabled_at = monotonic_sec();
        printf("enabled slave=%d status=0x%04x，开始出力 %d\n",
               torque_slave, status, requested_torque);
        fflush(stdout);
      } else if (monotonic_sec() - op_at > 8.0) {
        printf("slave=%d 8 秒内没有进入使能，status=0x%04x，目标扭矩保持 0\n",
               torque_slave, status);
        fflush(stdout);
        rc = 1;
        break;
      }
    } else {
      double since_enable = monotonic_sec() - enabled_at;
      if (since_enable < hold_sec) {
        command_torque = (int16_t)requested_torque;
        moving_control = 0x000F;
      } else if (since_enable < hold_sec + 0.5) {
        command_torque = 0;
        moving_control = 0x000F;
      } else {
        printf("torque done status=0x%04x actual=%d\n",
               torque_in->status_word, torque_in->actual_torque);
        fflush(stdout);
        break;
      }
      command_cycle(command_torque, moving_control);
    }
    ecx_send_processdata(&ctx);

    if ((cycle % 20) == 0) {
      uint16 want = op_requested ? EC_STATE_OPERATIONAL : EC_STATE_SAFE_OP;
      ecx_statecheck(&ctx, 0, want, 1000);
    }
    if (!safeop_seen && (ctx.slavelist[0].state & 0x0f) >= EC_STATE_SAFE_OP) {
      safeop_seen = 1;
      printf("safeop cycle=%d bus=0x%04x ready=%d\n", cycle, ctx.slavelist[0].state, ready);
      fflush(stdout);
    }
    if (safeop_seen && !op_requested && ready == MOTOR_COUNT) {
      ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
      ecx_writestate(&ctx, 0);
      op_requested = 1;
      printf("request op cycle=%d\n", cycle);
      fflush(stdout);
    }
    if (op_requested && ctx.slavelist[0].state == EC_STATE_OPERATIONAL && !op_seen) {
      op_seen = 1;
      op_at = monotonic_sec();
      printf("op bus=0x%04x，从站 %d 目标扭矩仍为 0\n", ctx.slavelist[0].state, torque_slave);
      fflush(stdout);
    }
    if (!op_seen && cycle > 12000) {
      fprintf(stderr, "没有进入 OP，bus=0x%04x\n", ctx.slavelist[0].state);
      rc = 1;
      break;
    }
    if (monotonic_sec() - last_print > 0.5 && torque_in != NULL && torque_out != NULL) {
      last_print = monotonic_sec();
      printf("slave=%d status=0x%04x torque=%d target=%d current=%d cw=0x%04x bus=0x%04x\n",
             torque_slave, torque_in->status_word, torque_in->actual_torque,
             torque_out->target_torque, torque_in->actual_current,
             torque_out->control_word, ctx.slavelist[0].state);
      fflush(stdout);
    }
    usleep(500);
  }

  return_to_init();
  ecx_close(&ctx);
  opened = 0;
  return rc;
}
