#define _GNU_SOURCE
#include "tx_pipeline.h"
#include "pipeline_runtime.h"
#include "mod_worker.h"
#include "tone_source.h"
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Compile the current writer body, including its static implementation.
#include "tx_writer_thread.inc"

pthread_mutex_t g_hw_lock = PTHREAD_MUTEX_INITIALIZER;
volatile bool nbfm_tx_active, nbfm_rx_active;
bool nbfm_tx_ready, nbfm_rx_ready;
uint64_t mono_ns(void) { static uint64_t now; return now += 10000000; }
int set_rt_and_affinity_prio(int priority, int cpu)
{ (void)priority; (void)cpu; return 0; }
int __wrap_clock_nanosleep(clockid_t clock, int flags,
                          const struct timespec* request, struct timespec* remain)
{ (void)clock; (void)flags; (void)request; (void)remain; return 0; }

static unsigned tone_changes, mic_reads;
int __real_tone_source_set(audio_source_t*, float, float);
int __wrap_tone_source_set(audio_source_t* source, float hz, float amplitude)
{ ++tone_changes; return __real_tone_source_set(source, hz, amplitude); }
static audio_source_result_t read_mic(audio_source_t* source, float* dst, size_t count)
{
    (void)source;
    ++mic_reads;
    for (size_t i=0; i<count; ++i) dst[i]=0.75f;
    return (audio_source_result_t){count, AUDIO_SOURCE_OK, 0};
}
static const audio_source_ops_t mic_ops = { .read=read_mic };

static tx_writer_ctrl_st* producer_tx;
static dsp_producer_ctrl_t* producer;
static pthread_mutex_t gate_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_condition = PTHREAD_COND_INITIALIZER;
static bool gate_enabled, gate_entered, gate_release, gate_success;
static unsigned producer_puts;
static uint64_t blocked_sequence;

bool rf10_fifo_put(rf10_fifo_t* fifo, const rf10_frame_t* frame, int timeout)
{
    assert(fifo==producer->fifo && timeout==-1);
    ++producer_puts;
    assert(frame->tx_sequence==17+producer_puts);
    for (size_t i=0; i<producer_tx->frame_samples; ++i) {
        assert(frame->data[i].i==(producer_tx->iq_rf[i].i | 1));
        assert(frame->data[i].q==producer_tx->iq_rf[i].q);
    }
    if (gate_enabled) {
        pthread_mutex_lock(&gate_lock);
        blocked_sequence=frame->tx_sequence;
        gate_entered=true;
        pthread_cond_signal(&gate_condition);
        while (!gate_release) pthread_cond_wait(&gate_condition, &gate_lock);
        bool success=gate_success;
        pthread_mutex_unlock(&gate_lock);
        producer->active=false;
        return success;
    }
    if (producer_puts==1) {
        bool nonzero=false;
        for (size_t i=0; i<480; ++i) nonzero |= producer_tx->a48k[i]!=0;
        assert(nonzero); // The trailing cue precedes the silent padding.
    } else {
        for (size_t i=0; i<480; ++i) assert(producer_tx->a48k[i]==0);
        assert(!mic_reads && tone_changes==1);
    }
    if (producer_puts==3) producer->active=false;
    return true;
}

static void producer_case(unsigned rate, bool block, bool success, bool tone_mode)
{
    rf10_fifo_t fifo={0};
    float audio[480];
    iq16_t* iq=calloc(rate/100, sizeof(*iq));
    nbfm_cfg_t config={48000,rate,2500,0,4000,1};
    audio_source_t mic={ {48000,1}, &mic_ops };
    tx_writer_ctrl_st tx={ .fm=nbfm_create(&config), .a48k=audio, .iq_rf=iq,
        .tone=tone_source_open(600,0.4f,(audio_format_t){48000,1}),
        .tone_mode=tone_mode, .mic=&mic, .tone_hz=600, .tone_amp=0.4f,
        .frame_samples=rate/100, .next_sequence=17, .fifo=&fifo,
        .inj={ .frames_left=1, .hz=2475, .last_sequence=8, .hold_silence=true } };
    dsp_producer_ctrl_t ctrl={ .active=true, .tx=&tx, .fifo=&fifo };
    assert(iq && tx.fm && tx.tone);
    producer_tx=&tx; producer=&ctrl;
    producer_puts=tone_changes=mic_reads=0;
    nbfm_tx_active=true;
    gate_enabled=block; gate_entered=gate_release=false; gate_success=success;
    if (block) {
        pthread_t thread;
        assert(!pthread_create(&thread,NULL,nbfm_mod_thread,&ctrl));
        pthread_mutex_lock(&gate_lock);
        struct timespec deadline;
        assert(!clock_gettime(CLOCK_REALTIME,&deadline)); deadline.tv_sec+=2;
        while (!gate_entered)
            assert(!pthread_cond_timedwait(&gate_condition,&gate_lock,&deadline));
        assert(blocked_sequence==18);
        pthread_mutex_lock(&g_tx_injection_lock);
        assert(tx.inj.frames_left==1 && tx.inj.last_sequence==8);
        pthread_mutex_unlock(&g_tx_injection_lock);
        gate_release=true;
        pthread_cond_signal(&gate_condition);
        pthread_mutex_unlock(&gate_lock);
        assert(!pthread_join(thread,NULL));
        assert(producer_puts==1);
        assert(tx.inj.frames_left==(success?0:1));
        assert(tx.inj.last_sequence==(success?18u:8u));
    } else {
        nbfm_mod_thread(&ctrl);
        assert(producer_puts==3 && tx.next_sequence==20);
        assert(tx.inj.frames_left==0 && tx.inj.last_sequence==18);
        assert(!mic_reads && tone_changes==1);
    }
    audio_source_destroy(tx.tone); nbfm_destroy(tx.fm); free(iq);
}

static tx_writer_ctrl_st* writer;
static unsigned fetched, write_calls, poll_calls, completed;
static size_t accepted;
static bool hard_failure, midframe_stop;
static uint64_t frame_sequence;
static int16_t marker_i(uint64_t sequence, size_t sample)
{ return (int16_t)((sequence*3+sample*23)%30000-15000); }
static int16_t marker_q(uint64_t sequence, size_t sample)
{ return (int16_t)((sequence*101+sample*19)%29000-14500); }

bool rf10_fifo_get(rf10_fifo_t* fifo, rf10_frame_t* frame, int timeout)
{
    assert(fifo==writer->fifo && timeout==-1);
    if (fetched) {
        assert(accepted==writer->frame_samples);
        assert(atomic_load(&writer->written_sequence)==frame_sequence);
        ++completed;
    }
    if (fetched==3) { writer->active=false; return false; }
    frame_sequence=21+fetched++;
    frame->tx_sequence=frame_sequence;
    for (size_t i=0; i<writer->frame_samples; ++i)
        frame->data[i]=(iq16_t){marker_i(frame_sequence,i),marker_q(frame_sequence,i)};
    accepted=0;
    return true;
}
size_t caribou_smi_get_native_batch_samples(caribou_smi_st* smi)
{ (void)smi; return 8192; }
int caribou_smi_set_driver_streaming_state(caribou_smi_st* smi, smi_stream_state_en state)
{ (void)smi; assert(state==smi_stream_idle || state==smi_stream_tx_channel); return 0; }
int cariboulite_radio_activate_channel(cariboulite_radio_state_st* radio,
                                      cariboulite_channel_dir_en direction, bool active)
{ assert(radio==writer->radio && direction==cariboulite_channel_dir_tx && !active); return 0; }
int __wrap_poll(struct pollfd* fds, nfds_t count, int timeout)
{
    assert(count==1 && timeout==10 && fds[0].events==POLLOUT);
    ++poll_calls;
    fds[0].revents=poll_calls%13==0?0:POLLOUT;
    return poll_calls%11==0?0:1;
}
int caribou_smi_write_samples(caribou_smi_st* smi, caribou_smi_channel_en channel,
                             const caribou_smi_sample_complex_int16* samples, int count)
{
    assert(smi==&writer->radio->sys->smi && channel==caribou_smi_channel_900);
    assert(atomic_load(&writer->written_sequence)==frame_sequence-1);
    size_t expected=writer->frame_samples-accepted;
    if (expected>2048) expected=2048; // Native getter returns samples; quarter=8192/4.
    assert(count==(int)expected);
    for (int i=0; i<count; ++i) {
        assert(samples[i].i==marker_i(frame_sequence,accepted+(size_t)i));
        assert(samples[i].q==marker_q(frame_sequence,accepted+(size_t)i));
    }
    ++write_calls;
    if (write_calls==2 && hard_failure) {
        errno=EIO; writer->active=false; return -1;
    }
    if (write_calls==2 && midframe_stop) {
        nbfm_tx_active=false; writer->active=false;
        accepted+=7; return 7;
    }
    if (write_calls%7==2) return 0;
    if (write_calls%7==3) { errno=EAGAIN; return -1; }
    size_t sent=write_calls%7==1?17:503+accepted%97;
    if (sent>(size_t)count) sent=(size_t)count;
    accepted+=sent;
    return (int)sent;
}
static void writer_case(unsigned rate, bool fail, bool stop)
{
    sys_st sys={0}; rf10_fifo_t fifo={0};
    sys.smi.filedesc=123456; // fcntl may fail; the SMI and poll calls are mocked.
    sys.radio_low.sys=&sys;
    tx_writer_ctrl_st tx={ .active=true, .radio=&sys.radio_low,
                          .frame_samples=rate/100, .fifo=&fifo };
    atomic_init(&tx.written_sequence,20);
    writer=&tx;
    fetched=write_calls=poll_calls=completed=0; accepted=0;
    hard_failure=fail; midframe_stop=stop; nbfm_tx_active=true;
    tx_writer_thread_func(&tx);
    if (fail || stop) {
        assert(fetched==1 && accepted<tx.frame_samples);
        assert(atomic_load(&tx.written_sequence)==20);
        assert(!completed);
        if (fail) assert(!nbfm_tx_active);
    } else {
        assert(fetched==3 && completed==3);
        assert(atomic_load(&tx.written_sequence)==23);
    }
}
int main(void)
{
    for (unsigned rate=1000000; rate<=4000000; rate*=2) {
        producer_case(rate,true,true,true);
        producer_case(rate,true,false,true);
        producer_case(rate,false,true,true);
        producer_case(rate,false,true,false);
        writer_case(rate,false,false);
        writer_case(rate,true,false);
        writer_case(rate,false,true);
        printf("PASS: %u MS/s: blocked/failed enqueue preserves injection progress, "
               "padding stays silent, partial writes preserve markers, complete-frame acknowledgment\n",
               rate/1000000);
    }
}
