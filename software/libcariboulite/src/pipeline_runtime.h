#pragma once
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
// Shared by menu hardware controls and both pipelines: never duplicate this lock.
extern pthread_mutex_t g_hw_lock;
#define HW_LOCK() pthread_mutex_lock(&g_hw_lock)
#define HW_UNLOCK() pthread_mutex_unlock(&g_hw_lock)
extern bool nbfm_tx_ready, nbfm_rx_ready;
extern volatile bool nbfm_tx_active, nbfm_rx_active;
uint64_t mono_ns(void);
int set_rt_and_affinity_prio(int prio, int cpu_req);
