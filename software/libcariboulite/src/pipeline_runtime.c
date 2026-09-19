#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "pipeline_runtime.h"
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

pthread_mutex_t g_hw_lock = PTHREAD_MUTEX_INITIALIZER;
bool nbfm_tx_ready = false, nbfm_rx_ready = false;
volatile bool nbfm_tx_active = false, nbfm_rx_active = false;

uint64_t mono_ns(void){
    struct timespec ts; 
	clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}

int set_rt_and_affinity_prio(int prio, int cpu_req)
{
    struct sched_param sp = { .sched_priority = prio };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        sp.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
    }

    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    if (ncpu < 1) ncpu = 1;
    int cpu = (cpu_req >= 0) ? cpu_req : 0;
    if (cpu >= ncpu) cpu = (int)(ncpu - 1);

    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc != 0) {
        fprintf(stderr, "[affinity] pid=%ld th=%lu prio=%d cpu_req=%d FAILED: %s\n",
                (long)getpid(), (unsigned long)pthread_self(), prio, cpu_req, strerror(rc));
        return -1;
    }
#ifdef __linux__
    int on = sched_getcpu();
    fprintf(stderr, "[affinity] th=%lu prio=%d pinned to CPU %d/%ld\n",
            (unsigned long)pthread_self(), prio, on, ncpu);
#endif
    mlockall(MCL_CURRENT | MCL_FUTURE);
    return 0;
}
