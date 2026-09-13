#!/usr/bin/env python3
"""Exercise actual TX tone/stop code with a simulated clock and hardware."""
from pathlib import Path
import subprocess
import tempfile
s=(Path(__file__).resolve().parents[1]/'src/app_menu.c').read_text()
a=s.index('static inline int ms_to_frames_10ms(');b=s.index('\nint tx_pipeline_start(',a)
c=s.index('static void tx_wait_fifo_drain(',b);d=s.index('\nvoid tx_pipeline_destroy(',c)
code=s[a:b]+s[c:d]
pre=r'''
#include <assert.h>
#include <pthread.h>
static pthread_mutex_t g_tx_injection_lock = PTHREAD_MUTEX_INITIALIZER;
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#define HW_LOCK() ((void)0)
#define HW_UNLOCK() ((void)0)
#define cariboulite_channel_dir_tx 1
#define caribou_fpga_io_ctrl_rfm_low_power 0
typedef int smi_stream_state_en;
typedef struct { int count; } rf10_stats_t;
typedef struct { int smi,fpga; } sys_st;
typedef struct {
    bool inited,running;
    struct { bool active; struct { float hz; int frames_left; } inj; } tx_ctrl;
    struct { bool active; } dsp_ctrl;
    sys_st* sys; void* radio; int txq;
} tx_pipeline_t;
static bool nbfm_tx_active;
static uint64_t now;
static int idle_calls,off_calls,low_calls,mode;
static tx_pipeline_t* current;
static uint64_t mono_ns(void) { return now; }
static int fake_sleep(const struct timespec* t,struct timespec* rest) {
    now+=2000000;
    if((mode==1 || mode==3) && current->tx_ctrl.inj.frames_left>0) --current->tx_ctrl.inj.frames_left;
    if(mode==2 && now>=20000000) nbfm_tx_active=false;
    return 0;
}
#define nanosleep fake_sleep
static void rf10_fifo_get_stats(int* q,rf10_stats_t* s) { s->count=(mode==3); }
static int caribou_smi_set_driver_streaming_state(int* s,int state) { assert(state==0);++idle_calls;return 0; }
static int cariboulite_radio_activate_channel(void* r,int dir,bool active) { assert(!active);++off_calls;return 0; }
static int caribou_fpga_set_io_ctrl_mode(int* f,int debug,int mode) { ++low_calls;return 0; }
'''
post=r'''
int main(void) {
    sys_st sys={0}; tx_pipeline_t p={0}; current=&p;
    for(int test=0;test<6;++test) {
        p=(tx_pipeline_t){.sys=&sys,.inited=true,.running=true};
        p.tx_ctrl.active=p.dsp_ctrl.active=true;
        nbfm_tx_active=true;now=0;idle_calls=off_calls=low_calls=0;
        mode=test==0?1:test==1?0:test==2?2:test==5?3:0;
        if(test==3) nbfm_tx_active=false;
        if(test==4) p.dsp_ctrl.active=false;
        tx_pipeline_stop(&p);
        assert(!p.running && !nbfm_tx_active && p.tx_ctrl.inj.frames_left==0);
        assert(idle_calls==1 && off_calls==1 && low_calls==1);
        assert(now<=1602000000ULL);
        if(test==3 || test==4) assert(now==0);
        if(test==5) assert(now>=600000000ULL);
        if(test==1) assert(now>=1000000000ULL);
        tx_pipeline_stop(&p);assert(off_calls==1);
    }
    puts("PASS: normal tail, stalled producer/full FIFO, mid-wait failure, inactive workers, repeated stop");
}
'''
with tempfile.TemporaryDirectory(prefix='tx-stop-') as directory:
    c=Path(directory)/'test.c';exe=Path(directory)/'test';c.write_text(pre+code+post)
    subprocess.run(['cc','-Wall','-Wextra','-Werror','-Wno-unused-parameter',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
