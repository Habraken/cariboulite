/* Include the implementation to exercise its private pipeline and FIFO types. */
#include "app_menu.c"
#include "mod_worker.h"

static bool real_threads, live[1024];
static int creates, fail_create, joins, hardware_active;
static bool fail_malloc, fail_calloc, fail_demod_create;
nbfm_demod_t* __real_nbfm_demod_create(const nbfm_demod_config_t*);
nbfm_demod_t* __wrap_nbfm_demod_create(const nbfm_demod_config_t* c) {
    return fail_demod_create ? NULL : __real_nbfm_demod_create(c);
}
static size_t fail_calloc_count;
static void* metadata_allocation;
static bool track_metadata;
static pthread_barrier_t reader_ready;
void __real_free(void*);
void __wrap_free(void* p) {
    if (p == metadata_allocation) metadata_allocation = NULL;
    __real_free(p);
}
void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
int __real_pthread_create(pthread_t*, const pthread_attr_t*, void*(*)(void*), void*);
int __real_pthread_cancel(pthread_t);
int __real_pthread_join(pthread_t, void**);
void *__wrap_malloc(size_t n) {
    void* p = fail_malloc ? NULL : __real_malloc(n);
    if (track_metadata) metadata_allocation = p;
    return p;
}
void *__wrap_calloc(size_t n, size_t s) { return (fail_calloc || (fail_calloc_count && n==fail_calloc_count)) ? NULL : __real_calloc(n,s); }
int __wrap_pthread_create(pthread_t* t, const pthread_attr_t* a, void*(*fn)(void*), void* v) {
    if (real_threads) return __real_pthread_create(t,a,fn,v);
    ++creates;
    if (creates == fail_create) return EAGAIN;
    assert(creates < 1024); *t = (pthread_t)creates; live[creates] = true; return 0;
}
int __wrap_pthread_cancel(pthread_t t) {
    if (real_threads) return __real_pthread_cancel(t);
    assert(t && t < 1024 && live[t]); return 0;
}
int __wrap_pthread_join(pthread_t t, void** out) {
    if (real_threads) return __real_pthread_join(t,out);
    assert(t && t < 1024 && live[t]); live[t] = false; ++joins; return 0;
}
static double tuned_frequency;
static bool fail_tune;
int __wrap_cariboulite_radio_set_frequency(cariboulite_radio_state_st* r, bool b, double* f) {
    if (fail_tune) return -1;
    tuned_frequency = *f;
    *f += 1; // driver returns achieved frequency: must not mutate const parameters
    return 0;
}
int __wrap_cariboulite_radio_activate_channel(cariboulite_radio_state_st* r, cariboulite_channel_dir_en d, bool a) {
    hardware_active = a; return 0;
}
static smi_stream_state_en last_stream;
int __wrap_caribou_smi_set_driver_streaming_state(caribou_smi_st* s, smi_stream_state_en e) { last_stream=e; return 0; }
int __wrap_caribou_fpga_set_io_ctrl_mode(caribou_fpga_st* f, uint8_t d, caribou_fpga_io_ctrl_rfm_en m) { return 0; }
static rx_reader_ctrl_st* capture_once;
static unsigned rssi_reads;
static uint16_t expected_rssi_register;
static uint8_t measured_rssi;
static int rssi_read_error;
int __wrap_at86rf215_read_buffer(at86rf215_st* dev, uint16_t reg, uint8_t* value, uint8_t length) {
    (void)dev;
    assert(capture_once && reg == expected_rssi_register && length == 1);
    ++rssi_reads; *value = measured_rssi;
    return rssi_read_error;
}
int __wrap_cariboulite_radio_read_samples(cariboulite_radio_state_st* r,
        cariboulite_sample_complex_int16* b, cariboulite_sample_meta* m, size_t n) {
    if (capture_once) {
        memset(b, 0, n*sizeof(*b));
        capture_once->active = false;
        return (int)n;
    }
    pthread_barrier_wait(&reader_ready);
    for (;;) { pthread_testcancel(); usleep(1000); }
    return 0;
}
int __wrap_cariboulite_radio_set_rx_sample_rate_flt(cariboulite_radio_state_st* r, float fs) { return 0; }
static float test_tx_rate = 4000000;
int __wrap_cariboulite_radio_set_tx_samp_cutoff_flt(cariboulite_radio_state_st* r, float fs) { test_tx_rate=fs; return 0; }
int __wrap_cariboulite_radio_get_tx_samp_cutoff_flt(cariboulite_radio_state_st* r, float* fs) { *fs=test_tx_rate; return 0; }
int __wrap_caribou_fpga_get_sys_ctrl_tx_sample_gap(caribou_fpga_st* f, uint8_t* gap) { *gap=4000000/test_tx_rate-1; return 0; }
int __wrap_cariboulite_radio_set_tx_power(cariboulite_radio_state_st* r, int power) { return 0; }
static void check_clean(rx_pipeline_t* p) {
    assert(!p->inited && !p->running && !hardware_active);
    assert(!p->demod.dsp && !p->demod.sink);
    for (int i=1; i<=creates; ++i) assert(!live[i]);
    rx_pipeline_destroy(p); /* repeated cleanup is harmless */
}
static aud10_fifo_t audio;
static rf10_fifo_t rf;
static void* waiter(void* arg) {
    int which = *(int*)arg;
    aud10_frame_t af = {0}; rf10_frame_t frame = {0};
    if (which == 0) aud10_fifo_get(&audio,&af,-1);
    if (which == 1) aud10_fifo_put(&audio,&af,-1);
    if (which == 2) rf10_fifo_get(&rf,&frame,-1);
    if (which == 3) rf10_fifo_put(&rf,&frame,-1);
    return NULL;
}
static void test_rssi_capture(void) {
    sys_st sys = {0};
    sys.radio_low.sys = sys.radio_high.sys = &sys;
    sys.radio_low.type = cariboulite_channel_s1g;
    sys.radio_high.type = cariboulite_channel_hif;
    rf10_fifo_t fifo;
    rf10_fifo_init(&fifo, 2, true);
    cariboulite_sample_complex_int16 buffer[20];
    atomic_uint flags;
    atomic_init(&flags, RX_SQUELCH_CARRIER);
    rx_reader_ctrl_st ctrl = {.active=true,.radio=&sys.radio_high,
        .rx_buffer=buffer,.rx_buffer_size=20,.rx_fifo=&fifo,.squelch_flags=&flags};
    capture_once=&ctrl;
    for (unsigned trial=0; trial<5; ++trial) {
        ctrl.active=true;
        ctrl.radio=trial==1 ? &sys.radio_low : &sys.radio_high;
        expected_rssi_register=trial==1 ? REG_RF09_RSSI : REG_RF24_RSSI;
        measured_rssi=trial==2 ? 127 : (uint8_t)(int8_t)-90;
        rssi_read_error=trial==3 ? -1 : 0;
        atomic_store(&flags, trial==4 ? 0 : RX_SQUELCH_CARRIER);
        unsigned before=rssi_reads;
        rx_reader_thread_func(&ctrl);
        rf10_frame_t frame;
        assert(rf10_fifo_get(&fifo,&frame,0));
        assert(frame.rssi_valid == (trial<2));
        if(trial<2) assert(frame.rssi_dbm==-90);
        assert(rssi_reads==before+(trial!=4));
        assert(pthread_mutex_trylock(&g_hw_lock)==0);
        pthread_mutex_unlock(&g_hw_lock);
    }
    capture_once=NULL;
    rf10_fifo_destroy(&fifo);
}
// Lifecycle mocks do not run DSP threads; acknowledge cue consumption so a
// successful TX start exercises real tuning/activation without generating RF.
static void* consume_injection(void* arg) {
    tx_pipeline_t* tx=arg;
    for (;;) {
        pthread_mutex_lock(&g_tx_injection_lock);
        tx->tx_ctrl.inj.frames_left=0;
        pthread_mutex_unlock(&g_tx_injection_lock);
        usleep(1000);
    }
    return NULL;
}
int main(void) {
    test_rssi_capture();
    double parsed=123;
    assert(monitor_parse_frequency("430.125",true,&parsed) && parsed==430125000);
    assert(monitor_parse_frequency(" 145.500 ",true,&parsed) && parsed==145500000);
    const char* invalid[]={"", "nan", "inf", "-1", "0", "6000", "430foo", "1e999", "430 100"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
        assert(!monitor_parse_frequency(invalid[i],true,&parsed));
        assert(parsed==145500000);
    }
    assert(!monitor_parse_frequency("430",false,&parsed));
    assert(monitor_parse_frequency("2385",false,&parsed));
    assert(monitor_parse_frequency("2495",false,&parsed));
    assert(!monitor_parse_frequency("2495.001",false,&parsed));
    sys_st sys = {0}; cariboulite_radio_state_st radio = {0}; rx_pipeline_t p;
    rx_params_t par = {.pcm_dev="null", .fs_rf=4000000, .fs_audio=48000};
    assert(rx_pipeline_init(&p,&sys,&radio,&par) == 0);
    assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_NOISE);
    rx_pipeline_set_squelch(&p,false,true);
    assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_CARRIER);
    rx_pipeline_set_squelch(&p,false,false);
    assert(atomic_load(&p.demod.squelch_flags)==0);
    rx_pipeline_destroy(&p); check_clean(&p); /* no reader ever created */
    assert(rx_pipeline_init(&p,&sys,&radio,&par) == 0);
    rx_pipeline_set_squelch(&p,false,true);
    for (int i=0; i<20; ++i) {
        assert(rx_pipeline_start(&p) == 0);
        rx_pipeline_stop(&p); rx_pipeline_stop(&p);
        assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_CARRIER);
    }
    rx_pipeline_destroy(&p); check_clean(&p);
    assert(rx_pipeline_init(&p,&sys,&radio,&par) == 0);
    assert(rx_pipeline_start(&p) == 0);
    rx_pipeline_destroy(&p); check_clean(&p); /* destroy while running */
    for (int failure=1; failure<=2; ++failure) {
        fail_create = creates + failure;
        assert(rx_pipeline_init(&p,&sys,&radio,&par) < 0);
        check_clean(&p); fail_create = 0;
    }
    fail_calloc = true;
    assert(rx_pipeline_init(&p,&sys,&radio,&par) < 0);
    fail_calloc = false; check_clean(&p);
    assert(rx_pipeline_init(&p,&sys,&radio,&par) == 0);
    fail_malloc = true;
    assert(rx_pipeline_start(&p) < 0 && !hardware_active);
    fail_malloc = false;
    fail_create = creates + 1;
    assert(rx_pipeline_start(&p) < 0 && !hardware_active);
    fail_create = 0;
    assert(rx_pipeline_start(&p) == 0); /* recover after failed start */
    rx_pipeline_destroy(&p); check_clean(&p);
    for(int rate=2000000;rate<=4000000;rate+=2000000) {
        par.fs_rf=rate;
        assert(rx_pipeline_init(&p,&sys,&radio,&par)==0);
        assert(rx_pipeline_start(&p)==0);
        assert(p.rx_ctrl.rx_buffer_size==(size_t)rate/100);
        rx_pipeline_destroy(&p);check_clean(&p);
    }
    par.fs_rf=4000000;
    assert(rx_pipeline_init(&p,&sys,&sys.radio_high,&par)==0);
    assert(rx_pipeline_start(&p)==0);
    assert(last_stream==smi_stream_rx_channel_1);
    rx_pipeline_destroy(&p);check_clean(&p);
    assert(rx_pipeline_init(&p,&sys,&sys.radio_low,&par)==0);
    assert(rx_pipeline_start(&p)==0);
    assert(last_stream==smi_stream_rx_channel_0);
    rx_pipeline_destroy(&p);check_clean(&p);
    par.pcm_dev = "cariboulite_test_missing_device";
    assert(rx_pipeline_init(&p,&sys,&radio,&par) < 0); check_clean(&p);

    tx_pipeline_t tx;
    tx_params_t tp = {.tone_mode=true, .f_dev_hz=2500, .out_scale=4000};
    for (int test=0;test<7;++test) {
        if(test==1) fail_create=creates+1;
        if(test==2) fail_create=creates+2;
        if(test==3) fail_calloc=true;
        if(test==4) fail_calloc_count=480;
        if(test==5) fail_calloc_count=40000;
        if(test==6) tp.mic_dev="cariboulite_test_missing_device";
        int ret=tx_pipeline_init(&tx,&sys,&radio,&tp);
        assert(test==0 ? ret==0 : ret<0);
        tx_pipeline_destroy(&tx); tx_pipeline_destroy(&tx);
        assert(!tx.inited && !tx.running);
        assert(!tx.tx_ctrl.mic && !tx.tx_ctrl.fm && !tx.tx_ctrl.a48k && !tx.tx_ctrl.iq_rf);
        for(int i=1;i<=creates;++i) assert(!live[i]);
        fail_create=0;fail_calloc=false;fail_calloc_count=0;
    }
    // Shared tuner must be restored on every direction start, including after
    // both initializers have tuned it, and tuning failures must block activation.
    for(unsigned fs=2000000;fs<=4000000;fs+=2000000) {
        tx_params_t split_tx={.tone_mode=true,.f_dev_hz=2500,.out_scale=4000,
            .freq_hz=430125000,.rf_fs=fs};
        rx_params_t split_rx={.pcm_dev="null",.freq_hz=145500000,.fs_rf=fs,.fs_audio=48000};
        assert(monitor_init_pipelines(&tx,&p,&sys,&split_tx,&split_rx));
        assert(split_tx.freq_hz==430125000 && split_rx.freq_hz==145500000);
        assert(tuned_frequency==split_rx.freq_hz);
        pthread_t consumer;
        assert(__real_pthread_create(&consumer,NULL,consume_injection,&tx)==0);
        assert(monitor_start_tx(&tx,&p,&split_tx)==0);
        assert(tx.running && !p.running && tuned_frequency==split_tx.freq_hz);
        assert(monitor_start_rx(&tx,&p,&split_rx)==0);
        assert(p.running && !tx.running && tuned_frequency==split_rx.freq_hz);
        fail_tune=true;
        assert(monitor_start_tx(&tx,&p,&split_tx)!=0);
        assert(!tx.running && !p.running && !hardware_active);
        assert(monitor_start_rx(&tx,&p,&split_rx)!=0);
        assert(!p.running && !hardware_active);
        fail_tune=false;
        assert(monitor_start_rx(&tx,&p,&split_rx)==0);
        assert(tuned_frequency==split_rx.freq_hz);
        assert(__real_pthread_cancel(consumer)==0);
        assert(__real_pthread_join(consumer,NULL)==0);
        rx_pipeline_destroy(&p); tx_pipeline_destroy(&tx);
        check_clean(&p);
    }
    tp.mic_dev=NULL;
    assert(tx_pipeline_init(&tx,&sys,&radio,&tp)==0);
    tx_pipeline_destroy(&tx);

    for(unsigned fs=2000000;fs<=4000000;fs+=2000000) {
        tp.rf_fs=fs;
        assert(tx_pipeline_init(&tx,&sys,&radio,&tp)==0);
        assert(tx.tx_ctrl.frame_samples==fs/100);
        assert(test_tx_rate==fs);
        float audio[480]={0};
        nbfm_push_audio(tx.tx_ctrl.fm,audio,480);
        assert(nbfm_pull_iq(tx.tx_ctrl.fm,tx.tx_ctrl.iq_rf,fs/100)==fs/100);
        tx_pipeline_destroy(&tx);
    }
    tp.rf_fs=0;

    par.pcm_dev="null";
    fail_demod_create=true;
    assert(rx_pipeline_init(&p,&sys,&radio,&par)<0); check_clean(&p);
    fail_demod_create=false;
    for(int failure=1;failure<=4;++failure) {
        rx_pipeline_t rx={0};
        fail_create=creates+failure;
        assert(!monitor_init_pipelines(&tx,&rx,&sys,&tp,&par));
        assert(!tx.inited && !rx.inited);
        for(int i=1;i<=creates;++i) assert(!live[i]);
        fail_create=0;
    }
    assert(monitor_init_pipelines(&tx,&p,&sys,&tp,&par));
    monitor_loopback_t loopback = {0};
    nbfm_demod_t* original = p.demod.dsp;
    int original_creates = creates;
    for (int busy=0; busy<4; ++busy) {
        tx.running = busy==0; p.running = busy==1;
        loopback.armed = busy==2; loopback.active = busy==3;
        assert(monitor_cycle_rx_mode(&tx,&p,&loopback,&sys,&par)==-EBUSY);
        assert(par.mode==FM_MODE_NBFM && p.demod.dsp==original && creates==original_creates);
    }
    tx.running = p.running = loopback.armed = loopback.active = false;
    for (int rate=2000000; rate<=4000000; rate*=2) {
        par.fs_rf = rate;
        assert(monitor_cycle_rx_mode(&tx,&p,&loopback,&sys,&par)==0);
        assert(par.mode==FM_MODE_WBFM && p.demod.mode==FM_MODE_WBFM && !p.running);
        rx_pipeline_set_squelch(&p,true,true);
        assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_CARRIER);
        assert(monitor_cycle_rx_mode(&tx,&p,&loopback,&sys,&par)==0);
        assert(par.mode==FM_MODE_NBFM && p.demod.mode==FM_MODE_NBFM && !p.running);
        assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_NOISE);
    }
    fail_calloc = true;
    assert(monitor_cycle_rx_mode(&tx,&p,&loopback,&sys,&par)!=0);
    assert(!p.inited && par.mode==FM_MODE_NBFM);
    fail_calloc = false;
    rx_pipeline_destroy(&p);tx_pipeline_destroy(&tx);

    /* Empty reads/full writes must honor the requested monotonic deadline. */
    for (int which=0; which<4; ++which) {
        aud10_fifo_init(&audio,1); rf10_fifo_init(&rf,1,false);
        aud10_frame_t af = {0}; rf10_frame_t frame = {0};
        if (which==1) audio.count=1;
        if (which==3) rf.count=1;
        struct timespec begin,end;
        clock_gettime(CLOCK_MONOTONIC,&begin);
        bool ok = which==0 ? aud10_fifo_get(&audio,&af,100) :
                  which==1 ? aud10_fifo_put(&audio,&af,100) :
                  which==2 ? rf10_fifo_get(&rf,&frame,100) :
                             rf10_fifo_put(&rf,&frame,100);
        clock_gettime(CLOCK_MONOTONIC,&end);
        double elapsed = (end.tv_sec-begin.tv_sec)*1000.0 +
                         (end.tv_nsec-begin.tv_nsec)/1000000.0;
        assert(!ok && elapsed>=90 && elapsed<2000);
        printf("FIFO timed wait %d: %.1f ms\n",which,elapsed);
        aud10_fifo_destroy(&audio); rf10_fifo_destroy(&rf);
    }

    real_threads = true;
    // Actual waiting DSP/writer threads must join before DSP/sink destruction.
    assert(rx_pipeline_init(&p,&sys,&radio,&par)==0);
    usleep(10000);
    rx_pipeline_destroy(&p); check_clean(&p);

    for (int which=0; which<4; ++which) {
        aud10_fifo_init(&audio,1); rf10_fifo_init(&rf,1,false);
        if (which==1) audio.count=1;
        if (which==3) rf.count=1;
        pthread_t t; void* result;
        assert(pthread_create(&t,NULL,waiter,&which)==0);
        assert(pthread_cancel(t)==0);
        assert(pthread_join(t,&result)==0 && result==PTHREAD_CANCELED);
        assert(pthread_mutex_trylock(&audio.m)==0); pthread_mutex_unlock(&audio.m);
        assert(pthread_mutex_trylock(&rf.m)==0); pthread_mutex_unlock(&rf.m);
        aud10_fifo_destroy(&audio); rf10_fifo_destroy(&rf);
    }
    cariboulite_sample_complex_int16 samples[40000];
    radio.sys = &sys;
    rx_reader_ctrl_st reader = {.active=true, .radio=&radio, .rx_buffer=samples};
    pthread_t thread;
    assert(pthread_barrier_init(&reader_ready,NULL,2)==0);
    track_metadata = true;
    assert(pthread_create(&thread,NULL,rx_reader_thread_func,&reader)==0);
    pthread_barrier_wait(&reader_ready);
    assert(metadata_allocation != NULL);
    assert(pthread_cancel(thread)==0);
    assert(pthread_join(thread,NULL)==0);
    assert(metadata_allocation == NULL);
    track_metadata = false;
    pthread_barrier_destroy(&reader_ready);
    puts("PASS: TX/RX lifecycle, monitor startup failures, FIFO timing and cancellation");
}
