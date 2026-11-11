#include <ecrt.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#define AXES 15
#define VENDOR 0x5a65726f
#define PRODUCT 0x29252
#define CYCLE_NS 1000000
#define EXPECTED_WKC (AXES * 3)
#define SERVO_READY_NS 5000000000LL
#define OFFICIAL_ENABLE_CYCLES 1000
#define FEEDBACK_TIMEOUT_CYCLES 15000
#define ENABLE_TIMEOUT_CYCLES 10000

enum drive_state { DRIVE_OTHER, DRIVE_ENABLED, DRIVE_FAULT };

static enum drive_state decode_drive_state(uint16_t status)
{
  switch (status & 0x006f) {
    case 0x0027: return DRIVE_ENABLED;
    case 0x0008: return DRIVE_FAULT;
    default: return DRIVE_OTHER;
  }
}

static volatile sig_atomic_t stop;
static void on_signal(int s) {(void)s; stop = 1;}
static void sleep_cycle(struct timespec *next)
{
  next->tv_nsec += CYCLE_NS;
  while (next->tv_nsec >= 1000000000L) {next->tv_nsec -= 1000000000L; ++next->tv_sec;}
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
}

int main(void)
{
  ec_master_t *master = NULL; ec_domain_t *domain = NULL;
  ec_slave_config_t *sc[AXES] = {0}; unsigned int off[AXES][6];
  uint8_t *data; ec_master_state_t ms = {}; ec_domain_state_t ds = {};
  const ec_pdo_entry_info_t rx[] = {{0x607a,0,32},{0x60fe,0,32},{0x6040,0,16}};
  const ec_pdo_entry_info_t tx[] = {{0x6064,0,32},{0x60fd,0,32},{0x6041,0,16}};
  const ec_pdo_info_t pdos[] = {{0x1600,3,rx},{0x1a00,3,tx}};
  const ec_sync_info_t syncs[] = {{0,EC_DIR_OUTPUT,0,NULL,EC_WD_DISABLE},{1,EC_DIR_INPUT,0,NULL,EC_WD_DISABLE},
    {2,EC_DIR_OUTPUT,1,&pdos[0],EC_WD_ENABLE},{3,EC_DIR_INPUT,1,&pdos[1],EC_WD_DISABLE},{0xff}};
  struct timespec next; int i;
  signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
  master = ecrt_request_master(0); if (!master) {fprintf(stderr,"IgH: request master failed\n"); return 1;}
  domain = ecrt_master_create_domain(master); if (!domain) {fprintf(stderr,"IgH: create domain failed\n"); return 1;}
  for (i=0;i<AXES;++i) {
    sc[i] = ecrt_master_slave_config(master, 0, (uint16_t)i, VENDOR, PRODUCT);
    if (!sc[i]) {fprintf(stderr,"IgH: slave %d config failed\n",i+1); return 1;}
    /* 1.6 兼容补丁会对 ZeroErr 固定布局跳过 CoE 重写；1.7 可替换为 EC_PDO_MODE_FIXED。 */
    if (ecrt_slave_config_pdos(sc[i], EC_END, syncs)) {
      fprintf(stderr,"IgH: slave %d fixed PDO configuration failed\n",i+1); return 1;
    }
    if (ecrt_slave_config_sdo8(sc[i], 0x6060, 0, 0x08) ||
        ecrt_slave_config_sdo8(sc[i], 0x60c2, 1, 0x01) ||
        ecrt_slave_config_sdo8(sc[i], 0x60c2, 2, 0xfd)) {
      fprintf(stderr,"IgH: slave %d startup SDO config failed\n",i+1); return 1;
    }
    /* 驱动器 PDO 映射对象只读：保留从站默认映射，仅注册已确认的 entry。 */
    const uint8_t sync[] = {2, 2, 2, 3, 3, 3};
    const unsigned int entry_pos[] = {0, 1, 2, 0, 1, 2};
    for (unsigned int k = 0; k < 6; ++k) {
      const int value = ecrt_slave_config_reg_pdo_entry_pos(sc[i], sync[k], 0, entry_pos[k], domain, NULL);
      if (value < 0) {fprintf(stderr,"IgH: slave %d PDO position %u/%u registration failed\n",i+1,sync[k],entry_pos[k]); return 1;}
      off[i][k] = (unsigned int)value;
    }
    /* 暂时关闭 DC，先隔离 PDO/Domain 状态。 */
  }
  if (ecrt_master_activate(master)) {fprintf(stderr,"IgH: activate failed\n"); return 1;}
  data = ecrt_domain_data(domain); if (!data) {fprintf(stderr,"IgH: domain data failed\n"); return 1;}
  clock_gettime(CLOCK_MONOTONIC,&next);
  printf("IgH probe: 15 slaves, 1 kHz, position hold; waiting 5 s before enable\n");
  int hold_latched = 0;
  int first_feedback_cycle = -1;
  int enable_started = 0;
  int all_enabled = 0;
  int enabled_count = 0;
  int result = 1;
  for (i=0; !stop; ++i) {
    ecrt_master_receive(master); ecrt_domain_process(domain); ecrt_domain_state(domain,&ds); ecrt_master_state(master,&ms);
    const int valid = ms.link_up && ms.al_states == EC_AL_STATE_OP &&
      ds.wc_state == EC_WC_COMPLETE && ds.working_counter == EXPECTED_WKC;
    if (valid && !hold_latched) {
      /* 首个完整反馈帧先锁存各轴实测位置，禁止用零位置作为启动目标。 */
      for (int j=0;j<AXES;++j) {
        const int32_t actual = EC_READ_S32(data+off[j][3]);
        EC_WRITE_S32(data+off[j][0], actual);
      }
      hold_latched = 1;
      first_feedback_cycle = i;
      printf("IgH: all feedback valid (WKC=%u); latched 15 actual positions\n", ds.working_counter);
    }
    if (!valid && hold_latched) {
      fprintf(stderr,"IgH: master/PDO feedback became invalid (link=%u AL=0x%02x WKC=%u domain=%u); aborting\n",
        ms.link_up, ms.al_states, ds.working_counter, ds.wc_state);
      result = 1;
      break;
    }
    if (!hold_latched && i > FEEDBACK_TIMEOUT_CYCLES) {
      fprintf(stderr,"IgH: no complete 15-axis feedback within 15 s (WKC=%u state=%u)\n",
        ds.working_counter, ds.wc_state);
      break;
    }
    const int64_t cycles_since_feedback = hold_latched ? i - first_feedback_cycle : 0;
    const int ready_elapsed = hold_latched &&
      (cycles_since_feedback * CYCLE_NS >= SERVO_READY_NS);
    const int enable_step = ready_elapsed ?
      (int)(cycles_since_feedback - SERVO_READY_NS / CYCLE_NS) : -1;
    enabled_count = 0;
    int enable_fault = 0;
    for (int j=0;j<AXES;++j) {
      uint16_t cw = 0x0006;
      const enum drive_state state = valid ? decode_drive_state(EC_READ_U16(data+off[j][5])) : DRIVE_OTHER;
      if (state == DRIVE_FAULT) {
        fprintf(stderr,"IgH: slave %d reports CiA-402 fault; not resetting it\n",j+1);
        enable_fault = 1;
        goto cycle_done;
      }
      if (ready_elapsed) {
        /* 沿用 eRob_CSP.cpp 已验证的 400/200/200/200 拍接通节拍。 */
        if (enable_step < 400) cw = 0x0080;
        else if (enable_step < 600) cw = 0x0006;
        else if (enable_step < 800) cw = 0x0007;
        else cw = 0x000f;
        if (state == DRIVE_ENABLED) ++enabled_count;
      }
      /* 目标位置只在首个完整反馈帧锁存一次，之后保持不变。 */
      EC_WRITE_U32(data+off[j][1], 0);
      EC_WRITE_U16(data+off[j][2], cw);
    }
    if (ready_elapsed && !enable_started) {
      enable_started = 1;
      printf("IgH: 5 s servo-ready delay elapsed; starting the 400/200/200/200-cycle eRob enable sequence\n");
    }
    if (ready_elapsed && enable_step >= OFFICIAL_ENABLE_CYCLES &&
      enabled_count == AXES && !all_enabled) {
      printf("IgH: all 15 drives report Operation Enabled; holding measured positions\n");
      fflush(stdout);
      all_enabled = 1;
      result = 0;
    }
    if (all_enabled && enabled_count != AXES) {
      fprintf(stderr,"IgH: a drive left Operation Enabled; stopping enable hold\n");
      result = 1;
      break;
    }
    if (!all_enabled && enable_started && i > (first_feedback_cycle +
      SERVO_READY_NS / CYCLE_NS + OFFICIAL_ENABLE_CYCLES + ENABLE_TIMEOUT_CYCLES)) {
      fprintf(stderr,"IgH: enable timeout; enabled %d/%d drives\n",enabled_count,AXES);
      for (int j=0;j<AXES;++j) {
        const uint16_t status = EC_READ_U16(data+off[j][5]);
        if (decode_drive_state(status) != DRIVE_ENABLED) {
          fprintf(stderr,"  slave %d statusword=0x%04x target_position=%d\n",
            j+1, status, EC_READ_S32(data+off[j][0]));
        }
      }
      break;
    }
cycle_done:
    ecrt_domain_queue(domain);
    ecrt_master_send(master); sleep_cycle(&next);
    if (enable_fault) break;
  }
  if (stop) fprintf(stderr,"IgH: interrupted; shutting drives down\n");
  if (stop || result != 0) {
    /* 失败或中断时仅下发 CiA-402 Shutdown，不自动复位驱动器故障。 */
    for (int n=0; n<100; ++n) {
      ecrt_master_receive(master); ecrt_domain_process(domain);
      for (int j=0;j<AXES;++j) {
        EC_WRITE_U16(data+off[j][2], 0x0006);
      }
      ecrt_domain_queue(domain); ecrt_master_send(master); sleep_cycle(&next);
    }
  }
  printf("IgH probe done: master=%u domain=%u working_counter=%u result=%d\n",
    ms.al_states,ds.working_counter,ds.wc_state,result);
  ecrt_release_master(master); return result;
}
