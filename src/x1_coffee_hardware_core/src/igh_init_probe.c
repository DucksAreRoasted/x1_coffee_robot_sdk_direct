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
  ec_pdo_entry_reg_t regs[AXES * 6 + 1] = {}; unsigned int r = 0;
  const ec_pdo_entry_info_t rx[] = {{0x607a,0,32},{0x60fe,0,32},{0x6040,0,16}};
  const ec_pdo_entry_info_t tx[] = {{0x6064,0,32},{0x60fd,0,32},{0x6041,0,16}};
  const ec_pdo_info_t pdos[] = {{0x1600,3,rx},{0x1a00,3,tx}};
  const ec_sync_info_t sync[] = {{0,EC_DIR_OUTPUT,0,NULL,EC_WD_DISABLE},{1,EC_DIR_INPUT,0,NULL,EC_WD_DISABLE},
    {2,EC_DIR_OUTPUT,1,&pdos[0],EC_WD_ENABLE},{3,EC_DIR_INPUT,1,&pdos[1],EC_WD_DISABLE},{0xff}};
  struct timespec next; int i;
  signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
  master = ecrt_request_master(0); if (!master) {fprintf(stderr,"IgH: request master failed\n"); return 1;}
  domain = ecrt_master_create_domain(master); if (!domain) {fprintf(stderr,"IgH: create domain failed\n"); return 1;}
  for (i=0;i<AXES;++i) {
    sc[i] = ecrt_master_slave_config(master, 0, (uint16_t)i, VENDOR, PRODUCT);
    if (!sc[i] || ecrt_slave_config_pdos(sc[i], EC_END, sync)) {fprintf(stderr,"IgH: slave %d config failed\n",i+1); return 1;}
    regs[r++] = (ec_pdo_entry_reg_t){0, (uint16_t)i, VENDOR, PRODUCT, 0x607a,0, &off[i][0]};
    regs[r++] = (ec_pdo_entry_reg_t){0, (uint16_t)i, VENDOR, PRODUCT, 0x60fe,0, &off[i][1]};
    regs[r++] = (ec_pdo_entry_reg_t){0, (uint16_t)i, VENDOR, PRODUCT, 0x6040,0, &off[i][2]};
    regs[r++] = (ec_pdo_entry_reg_t){0, (uint16_t)i, VENDOR, PRODUCT, 0x6064,0, &off[i][3]};
    regs[r++] = (ec_pdo_entry_reg_t){0, (uint16_t)i, VENDOR, PRODUCT, 0x60fd,0, &off[i][4]};
    regs[r++] = (ec_pdo_entry_reg_t){0, (uint16_t)i, VENDOR, PRODUCT, 0x6041,0, &off[i][5]};
    ecrt_slave_config_dc(sc[i], 0x0300, CYCLE_NS, 0, 0, 0);
  }
  if (ecrt_domain_reg_pdo_entry_list(domain, regs)) {fprintf(stderr,"IgH: PDO registration failed\n"); return 1;}
  if (ecrt_master_activate(master)) {fprintf(stderr,"IgH: activate failed\n"); return 1;}
  data = ecrt_domain_data(domain); if (!data) {fprintf(stderr,"IgH: domain data failed\n"); return 1;}
  clock_gettime(CLOCK_MONOTONIC,&next); printf("IgH probe: 15 slaves, 1 kHz, safe outputs\n");
  for (i=0; !stop && i<15000; ++i) {
    ecrt_master_receive(master); ecrt_domain_process(domain); ecrt_domain_state(domain,&ds); ecrt_master_state(master,&ms);
    for (int j=0;j<AXES;++j) {EC_WRITE_U32(data+off[j][0],0); EC_WRITE_U32(data+off[j][1],0); EC_WRITE_U16(data+off[j][2],0);}
    ecrt_domain_queue(domain); ecrt_master_application_time((uint64_t)i*CYCLE_NS); ecrt_master_send(master); sleep_cycle(&next);
  }
  printf("IgH probe done: master=%u domain=%u working_counter=%u\n",ms.al_states,ds.working_counter,ds.wc_state);
  ecrt_release_master(master); return 0;
}
