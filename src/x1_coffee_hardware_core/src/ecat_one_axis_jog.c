/* 直连 EtherCAT，只让一台电机转起来。
 * 使能顺序参考 ZeroErr Steering_Wheel_Applications 的 MAIN：
 *   控制字先写 6，0.5 秒后写 47（0x002F），运行模式 3（轮廓速度），再写目标速度。
 * 其余 14 台仍是轮廓位置，目标锁在当前编码器，控制字保持 0。
 * 用法: ecat_one_axis_jog [网口] [从站 1-15] [目标速度 计数/秒] [秒]
 * 默认 eno1、从站 1（腰）、5000 计数/秒、2 秒。速度绝对值不超过 20000。
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
#define MAX_ABS_VELOCITY 20000

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

/* 手册轮廓速度 Rx：控制字 + 目标速度。Tx：状态字 + 实际速度。 */
typedef struct __attribute__((packed))
{
  uint16_t control_word;
  int32_t target_velocity;
} VelocityOutput;

typedef struct __attribute__((packed))
{
  uint16_t status_word;
  int32_t actual_velocity;
} VelocityInput;

static ecx_contextt ctx;
static uint8_t iomap[8192];
static int opened;
static int moving_slave = 1;
static volatile sig_atomic_t stop_requested;
static PositionOutput *position_out[MOTOR_COUNT + 1];
static PositionInput *position_in[MOTOR_COUNT + 1];
static VelocityOutput *velocity_out;
static VelocityInput *velocity_in;
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

static int pdo_config(ecx_contextt *context, uint16 slave)
{
  uint8_t value8 = 0;
  uint32_t value32 = 0;
  int rc;
  if ((int)slave == moving_slave) {
    uint8_t mode = 0x03;
    uint32_t profile_acceleration = 1000;
    uint32_t profile_deceleration = 1000;
    rc = ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
    printf("slave=%u sdo 0x6060=0x03 rc=%d\n", slave, rc);
    ecx_SDOwrite(context, slave, 0x6083, 0x00, FALSE, sizeof(profile_acceleration),
                 &profile_acceleration, EC_TIMEOUTRXM);
    ecx_SDOwrite(context, slave, 0x6084, 0x00, FALSE, sizeof(profile_deceleration),
                 &profile_deceleration, EC_TIMEOUTRXM);
    ecx_SDOwrite(context, slave, 0x1600, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value32 = 0x60400010;
    ecx_SDOwrite(context, slave, 0x1600, 1, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x60FF0020;
    ecx_SDOwrite(context, slave, 0x1600, 2, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value8 = 2;
    ecx_SDOwrite(context, slave, 0x1600, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value8 = 0;
    ecx_SDOwrite(context, slave, 0x1A00, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    value32 = 0x60410010;
    ecx_SDOwrite(context, slave, 0x1A00, 1, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value32 = 0x606C0020;
    ecx_SDOwrite(context, slave, 0x1A00, 2, FALSE, sizeof(value32), &value32, EC_TIMEOUTRXM);
    value8 = 2;
    rc = ecx_SDOwrite(context, slave, 0x1A00, 0, FALSE, sizeof(value8), &value8, EC_TIMEOUTRXM);
    printf("slave=%u velocity pdo map rc=%d\n", slave, rc);
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

static void command_cycle(int32_t velocity, uint16_t moving_control)
{
  int slave;
  for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
    if (slave == moving_slave) {
      if (velocity_out != NULL) {
        velocity_out->target_velocity = velocity;
        velocity_out->control_word = moving_control;
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
  int32_t target_velocity = (argc > 3) ? (int32_t)atoi(argv[3]) : 5000;
  double spin_sec = (argc > 4) ? atof(argv[4]) : 2.0;
  int count;
  int slave;
  int cycle;
  int safeop_seen = 0;
  int op_requested = 0;
  int op_seen = 0;
  int rc = 0;
  double op_at = 0.0;
  double last_print = 0.0;
  uint16_t moving_control = 0x0006;
  int32_t command_velocity = 0;

  moving_slave = (argc > 2) ? atoi(argv[2]) : 1;
  if (moving_slave < 1 || moving_slave > MOTOR_COUNT) {
    fprintf(stderr, "从站号必须是 1 到 15\n");
    return 2;
  }
  if (target_velocity == 0 || target_velocity > MAX_ABS_VELOCITY ||
      target_velocity < -MAX_ABS_VELOCITY) {
    fprintf(stderr, "目标速度必须非 0，且绝对值不超过 %d 计数/秒\n", MAX_ABS_VELOCITY);
    return 2;
  }
  if (spin_sec <= 0.0 || spin_sec > 5.0) {
    fprintf(stderr, "转动时间必须在 0 到 5 秒之间\n");
    return 2;
  }

  printf("one-axis-velocity ifname=%s slave=%d %s velocity=%d counts/s seconds=%.2f\n",
         ifname, moving_slave, joint_name(moving_slave), target_velocity, spin_sec);
  printf("参考转向轮示例：模式 3，控制字 6 保持 0.5 秒后改为 0x002F，再给目标速度。\n");
  printf("请站在机器人旁边。其余轴不上电。\n");
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
    if (slave == moving_slave) {
      if (ctx.slavelist[slave].Obytes != (int)sizeof(VelocityOutput) ||
          ctx.slavelist[slave].Ibytes != (int)sizeof(VelocityInput) ||
          ctx.slavelist[slave].outputs == NULL || ctx.slavelist[slave].inputs == NULL) {
        fprintf(stderr, "slave=%d 速度 PDO 长度不符 obytes=%d ibytes=%d\n",
                slave, ctx.slavelist[slave].Obytes, ctx.slavelist[slave].Ibytes);
        return_to_init();
        ecx_close(&ctx);
        return 1;
      }
      velocity_out = (VelocityOutput *)ctx.slavelist[slave].outputs;
      velocity_in = (VelocityInput *)ctx.slavelist[slave].inputs;
      velocity_out->control_word = 0;
      velocity_out->target_velocity = 0;
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
    for (slave = 1; slave <= MOTOR_COUNT; ++slave) {
      if (slave == moving_slave) {
        if (wkc > 0 && velocity_in != NULL && velocity_in->status_word != 0) {
          held[slave] = 1;
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
    } else {
      double since_op = monotonic_sec() - op_at;
      if (since_op < 0.5) {
        moving_control = 0x0006;
        command_velocity = 0;
      } else if (since_op < 0.5 + spin_sec) {
        moving_control = 0x002F;
        command_velocity = target_velocity;
      } else if (since_op < 1.0 + spin_sec) {
        moving_control = 0x002F;
        command_velocity = 0;
      } else {
        printf("spin done status=0x%04x velocity=%d\n",
               velocity_in->status_word, velocity_in->actual_velocity);
        fflush(stdout);
        break;
      }
      command_cycle(command_velocity, moving_control);
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
      printf("op bus=0x%04x，从站 %d 先写控制字 0x0006\n", ctx.slavelist[0].state, moving_slave);
      fflush(stdout);
    }
    if (!op_seen && cycle > 12000) {
      fprintf(stderr, "没有进入 OP，bus=0x%04x\n", ctx.slavelist[0].state);
      rc = 1;
      break;
    }
    if (monotonic_sec() - last_print > 0.5 && velocity_in != NULL) {
      last_print = monotonic_sec();
      printf("slave=%d status=0x%04x vel=%d target_vel=%d cw=0x%04x bus=0x%04x\n",
             moving_slave, velocity_in->status_word, velocity_in->actual_velocity,
             velocity_out->target_velocity, velocity_out->control_word, ctx.slavelist[0].state);
      fflush(stdout);
    }
    usleep(500);
  }

  return_to_init();
  ecx_close(&ctx);
  opened = 0;
  return rc;
}
