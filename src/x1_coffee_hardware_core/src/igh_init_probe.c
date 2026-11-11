#include <ecrt.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#define AXES 15
#define VENDOR 0x5a65726f
#define PRODUCT 0x29252
#define CYCLE_NS 1000000

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
  const uint64_t app_start = ((uint64_t)next.tv_sec - 946684800ULL) * 1000000000ULL + (uint64_t)next.tv_nsec;
  printf("IgH probe: 15 slaves, 1 kHz, safe outputs\n");
  for (i=0; !stop && i<15000; ++i) {
    (void)app_start;
    ecrt_master_receive(master); ecrt_domain_process(domain); ecrt_domain_state(domain,&ds); ecrt_master_state(master,&ms);
    for (int j=0;j<AXES;++j) {
      EC_WRITE_U32(data+off[j][0], 0);  /* 目标位置安全保持 */
      EC_WRITE_U32(data+off[j][1], 0);  /* 数字输出 */
      const uint16_t cw = i < 400 ? 0x0006 : (i < 600 ? 0x0007 : 0x000F);
      EC_WRITE_U16(data+off[j][2], cw);  /* CiA402 official sequence */
    }
    ecrt_domain_queue(domain);
    ecrt_master_send(master); sleep_cycle(&next);
  }
  printf("IgH probe done: master=%u domain=%u working_counter=%u\n",ms.al_states,ds.working_counter,ds.wc_state);
  ecrt_release_master(master); return 0;
}
