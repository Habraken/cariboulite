#!/usr/bin/env python3
"""Exercise actual TX stop code against simulated producer, FIFO and cyclic DMA.

Each simulated RF sample carries a tone/silence marker. The driver accepts only
complete native quarters and refills its four-period ring as the real driver
does. All time and services are local; no radio hardware is accessed.
"""
from pathlib import Path
import subprocess
import tempfile

src = Path(__file__).resolve().parents[1] / 'src'
s = (src / 'tx_pipeline.c').read_text()
h = s.index('static unsigned tx_tail_padding_frames(')
i = s.index('\nint tx_pipeline_init(', h)
a = s.index('static inline int ms_to_frames_10ms(')
b = s.index('\nint tx_pipeline_start(', a)
c = s.index('static bool tx_wait_written(', b)
d = s.index('\nvoid tx_pipeline_destroy(', c)
code = s[h:i] + s[a:b] + s[c:d]
pre = r'''
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <limits.h>
static pthread_mutex_t g_tx_injection_lock = PTHREAD_MUTEX_INITIALIZER;
#define HW_LOCK() ((void)0)
#define HW_UNLOCK() ((void)0)
#define cariboulite_channel_dir_tx 1
#define caribou_fpga_io_ctrl_rfm_low_power 0
typedef int smi_stream_state_en;
typedef struct { size_t cap,count; } rf10_stats_t;
typedef struct { int smi,fpga; } sys_st;
typedef struct {
    bool inited,running;
    struct {
        bool active;
        size_t frame_samples;
        unsigned tail_padding_frames;
        uint64_t next_sequence;
        atomic_uint_fast64_t written_sequence;
        struct { float hz; int frames_left; uint64_t last_sequence; bool hold_silence; } inj;
    } tx_ctrl;
    struct { bool active; } dsp_ctrl;
    sys_st* sys;
    void* radio;
    rf10_stats_t txq;
} tx_pipeline_t;
static bool nbfm_tx_active;
static uint64_t now;
static tx_pipeline_t* current;
static int idle_calls,off_calls,low_calls;
static uint64_t mono_ns(void) { return now; }

/* kfifo and cyclic DMA operate in samples, matching the public SMI getter. */
#define NATIVE_SAMPLES 131072u
#define QUARTER_SAMPLES (NATIVE_SAMPLES / 4)
#define DRIVER_STORAGE (32 * NATIVE_SAMPLES)
static unsigned char kernel_fifo[DRIVER_STORAGE],dma_ring[NATIVE_SAMPLES];
static size_t kernel_head,kernel_tail,kernel_count,kernel_cap,dma_pos;
static unsigned rf_fs;
static uint64_t tone_samples,generated_tone_frames;
typedef struct { uint64_t sequence; bool tone,last; } frame_t;
static frame_t app_fifo[64],writer_frame;
static size_t app_head,app_tail,app_count,writer_off;
static bool writer_busy,freeze_producer;
static uint64_t writer_ready_ns,final_written_ns,final_queued_ns;
static unsigned post_frames;
enum { NORMAL, STALLED_PRODUCER, FAILED_WORKER, STALLED_WRITER,
       DELAY_LAST_WRITE, STALL_LAST_WRITE };
static int mode;

static void push_kernel(bool tone,size_t samples) {
    assert(kernel_count+samples<=kernel_cap);
    for(size_t i=0;i<samples;++i) {
        kernel_fifo[kernel_tail]=tone;
        kernel_tail=(kernel_tail+1)%DRIVER_STORAGE;
    }
    kernel_count+=samples;
}
static void advance_rf(void) {
    for(unsigned i=0;i<rf_fs/500;++i) {
        tone_samples+=dma_ring[dma_pos];
        if(++dma_pos==NATIVE_SAMPLES) dma_pos=0;
        if(dma_pos%QUARTER_SAMPLES==0) {
            /* Refill the quarter that just finished; DMA reads it one lap later. */
            size_t completed=(dma_pos+NATIVE_SAMPLES-QUARTER_SAMPLES)%NATIVE_SAMPLES;
            size_t previous=(completed+NATIVE_SAMPLES-QUARTER_SAMPLES)%NATIVE_SAMPLES;
            if(kernel_count>=QUARTER_SAMPLES) {
                for(size_t j=0;j<QUARTER_SAMPLES;++j) {
                    dma_ring[completed+j]=kernel_fifo[kernel_head];
                    kernel_head=(kernel_head+1)%DRIVER_STORAGE;
                }
                kernel_count-=QUARTER_SAMPLES;
            } else {
                memcpy(dma_ring+completed,dma_ring+previous,QUARTER_SAMPLES);
            }
        }
    }
}
static void produce_frame(void) {
    if(mode==STALLED_PRODUCER || freeze_producer || app_count==64) return;
    if(current->tx_ctrl.inj.frames_left<=0) return;
    bool tone=current->tx_ctrl.inj.hz==2475.0f;
    assert(tone || current->tx_ctrl.inj.hz==0.0f);
    bool last=!tone && generated_tone_frames==25 &&
              post_frames+1==current->tx_ctrl.tail_padding_frames;
    frame_t f={.sequence=++current->tx_ctrl.next_sequence,.tone=tone,.last=last};
    app_fifo[app_tail]=f;app_tail=(app_tail+1)%64;++app_count;
    /* Match producer acknowledgement only after successful enqueue. */
    current->tx_ctrl.inj.last_sequence=f.sequence;
    --current->tx_ctrl.inj.frames_left;
    if(tone) ++generated_tone_frames;
    else if(generated_tone_frames==25) ++post_frames;
    if(last) final_queued_ns=now;
    if(last && (mode==DELAY_LAST_WRITE || mode==STALL_LAST_WRITE)) freeze_producer=true;
}
static void write_frame(void) {
    if(mode==STALLED_WRITER) return;
    for(;;) {
        if(!writer_busy) {
            if(!app_count) return;
            writer_frame=app_fifo[app_head];app_head=(app_head+1)%64;--app_count;
            writer_off=0;writer_busy=true;
            writer_ready_ns=now;
            if(writer_frame.last && mode==DELAY_LAST_WRITE) writer_ready_ns+=300000000;
        }
        if(writer_frame.last && mode==STALL_LAST_WRITE) return;
        if(now<writer_ready_ns) return;
        size_t count=current->tx_ctrl.frame_samples-writer_off;
        if(count>kernel_cap-kernel_count) count=kernel_cap-kernel_count;
        if(!count) return;
        push_kernel(writer_frame.tone,count);
        writer_off+=count;
        if(writer_off<current->tx_ctrl.frame_samples) return;
        /* A dequeued FIFO frame remains pending until every sample is written. */
        atomic_store(&current->tx_ctrl.written_sequence,writer_frame.sequence);
        if(writer_frame.last) final_written_ns=now;
        writer_busy=false;
    }
}
static int fake_sleep(const struct timespec* t,struct timespec* rest) {
    assert(t->tv_sec==0 && t->tv_nsec==2000000);
    now+=2000000;
    advance_rf();
    if(mode==FAILED_WORKER && now>=20000000) nbfm_tx_active=false;
    if(nbfm_tx_active && current->tx_ctrl.active && current->dsp_ctrl.active) {
        if(now%10000000==0) produce_frame();
        write_frame();
    }
    current->txq.count=app_count;
    return 0;
}
#define nanosleep fake_sleep
static int caribou_smi_set_driver_streaming_state(int* s,int state) {
    assert(state==0);++idle_calls;return 0;
}
static int cariboulite_radio_activate_channel(void* r,int dir,bool active) {
    assert(!active);++off_calls;return 0;
}
static int caribou_fpga_set_io_ctrl_mode(int* f,int debug,int setting) {
    ++low_calls;return 0;
}
'''
post = r'''
static void reset(tx_pipeline_t* p,sys_st* sys,unsigned fs,unsigned multiplier,int scenario) {
    *p=(tx_pipeline_t){.sys=sys,.inited=true,.running=true,.txq={.cap=64}};
    p->tx_ctrl.active=p->dsp_ctrl.active=true;
    p->tx_ctrl.frame_samples=fs/100;
    p->tx_ctrl.tail_padding_frames=tx_tail_padding_frames(NATIVE_SAMPLES,multiplier,fs/100);
    atomic_init(&p->tx_ctrl.written_sequence,0);
    current=p;nbfm_tx_active=true;now=0;mode=scenario;
    idle_calls=off_calls=low_calls=0;
    rf_fs=fs;tone_samples=generated_tone_frames=post_frames=0;
    kernel_head=kernel_tail=kernel_count=dma_pos=0;
    kernel_cap=multiplier*NATIVE_SAMPLES;
    memset(kernel_fifo,0,sizeof(kernel_fifo));memset(dma_ring,0,sizeof(dma_ring));
    /* Worst-case preexisting driver FIFO backlog at the moment Stop is pressed. */
    push_kernel(false,kernel_cap);
    app_head=app_tail=app_count=writer_off=0;
    writer_busy=freeze_producer=false;writer_ready_ns=final_written_ns=final_queued_ns=0;
}
static uint64_t deadline_bound(const tx_pipeline_t* p) {
    uint64_t budget=(5+25+p->tx_ctrl.tail_padding_frames)*10000000ULL+
                    (p->txq.cap+1)*10000000ULL+600000000ULL;
    if(budget<1000000000ULL) budget=1000000000ULL;
    return budget+(p->txq.cap+1)*10000000ULL+602000000ULL;
}
static void assert_shutdown(tx_pipeline_t* p) {
    assert(!p->running && !nbfm_tx_active && p->tx_ctrl.inj.frames_left==0);
    assert(idle_calls==1 && off_calls==1 && low_calls==1);
    assert(now<=deadline_bound(p));
    tx_pipeline_stop(p);assert(idle_calls==1 && off_calls==1 && low_calls==1);
}
int main(void) {
    sys_st sys={0};tx_pipeline_t p;
    const unsigned rates[]={1000000,2000000,4000000};
    const unsigned multipliers[]={6,16};
    for(unsigned r=0;r<3;++r) for(unsigned m=0;m<2;++m) {
        for(int scenario=NORMAL;scenario<=STALL_LAST_WRITE;++scenario) {
            reset(&p,&sys,rates[r],multipliers[m],scenario);
            assert(p.tx_ctrl.tail_padding_frames>=25);
            tx_pipeline_stop(&p);
            assert_shutdown(&p);
            if(scenario==NORMAL || scenario==DELAY_LAST_WRITE) {
                assert(generated_tone_frames==25);
                assert(tone_samples==25*p.tx_ctrl.frame_samples);
                assert(atomic_load(&p.tx_ctrl.written_sequence)>=p.tx_ctrl.inj.last_sequence);
                assert(final_written_ns && now>=final_written_ns);
                if(scenario==DELAY_LAST_WRITE) assert(freeze_producer && p.txq.count==0);
            }
            if(scenario==STALLED_PRODUCER) assert(now>=1000000000ULL);
            if(scenario==FAILED_WORKER) assert(now==20000000);
            if(scenario==STALL_LAST_WRITE) {
                assert(writer_busy && p.txq.count==0);
                assert(atomic_load(&p.tx_ctrl.written_sequence)<p.tx_ctrl.inj.last_sequence);
                assert(now>=final_queued_ns+1250000000ULL &&
                       now<=final_queued_ns+1252000000ULL);
            }
        }
        printf("PASS %u MS/s, FIFO multiplier %u: full 250 ms 2475 Hz tail through DMA, writer completion, bounded failures\n",
               rates[r]/1000000,multipliers[m]);
    }
    for(unsigned worker=0;worker<3;++worker) {
        reset(&p,&sys,1000000,6,NORMAL);
        if(worker==2) p.tx_ctrl.active=false;
        else if(worker==1) p.dsp_ctrl.active=false;
        else nbfm_tx_active=false;
        tx_pipeline_stop(&p);assert_shutdown(&p);assert(now==0);
    }
    /* Demonstrate the regression: fixed 250 ms padding truncates at 1 MS/s. */
    reset(&p,&sys,1000000,6,NORMAL);
    p.tx_ctrl.tail_padding_frames=25;
    kernel_head=kernel_tail=kernel_count=0;
    push_kernel(false,2*NATIVE_SAMPLES); /* Enough queued data for a partial cutoff. */
    tx_pipeline_stop(&p);
    assert(tone_samples>0 && tone_samples<25*p.tx_ctrl.frame_samples);
    puts("PASS: previous fixed padding reproduces cutoff; inactive and repeated stop remain safe");
}
'''
with tempfile.TemporaryDirectory(prefix='tx-stop-') as directory:
    c = Path(directory) / 'test.c'
    exe = Path(directory) / 'test'
    c.write_text(pre + code + post)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', str(c),
                    '-pthread', '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=10)
    print(result.stdout, end='')
    if result.returncode:
        print(result.stderr, end='')
        result.check_returncode()
