/* Include the implementation to exercise its private pipeline and FIFO types. */
#include "app_menu.c"
#include "nbfm_demod.h"
#include "mod_worker.h"
#include "caribou_smi/kernel/smi_stream_dev.h"
#include <stdarg.h>
#include <sys/ioctl.h>

static bool real_threads, live[1024];
static int creates, fail_create, joins, hardware_active;
static bool fail_malloc, fail_calloc, fail_demod_create;
audio_demod_t* __real_audio_demod_create(audio_demod_mode_t, const audio_demod_config_t*);
audio_demod_t* __wrap_audio_demod_create(audio_demod_mode_t mode, const audio_demod_config_t* c) {
    return fail_demod_create ? NULL : __real_audio_demod_create(mode,c);
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
static cariboulite_radio_state_st* tuned_radio;
static cariboulite_radio_state_st* activated_radio;
static bool fail_tune;
int __wrap_cariboulite_radio_set_frequency(cariboulite_radio_state_st* r, bool b, double* f) {
    if (fail_tune) return -1;
    tuned_radio = r;
    tuned_frequency = *f;
    *f += 1; // driver returns achieved frequency: must not mutate const parameters
    return 0;
}
int __wrap_cariboulite_radio_activate_channel(cariboulite_radio_state_st* r, cariboulite_channel_dir_en d, bool a) {
    activated_radio = r;
    hardware_active = a; return 0;
}
static smi_stream_state_en last_stream;
int __wrap_caribou_smi_set_driver_streaming_state(caribou_smi_st* s, smi_stream_state_en e) { last_stream=e; return 0; }
static size_t native_batch_samples = 131072;
static int fifo_multiplier = 16;
static bool fail_fifo_query;
static unsigned native_batch_reads, fifo_multiplier_reads;
size_t __wrap_caribou_smi_get_native_batch_samples(caribou_smi_st* smi) {
    assert(smi);
    ++native_batch_reads;
    return native_batch_samples;
}
int __real_ioctl(int, unsigned long, ...);
int __wrap_ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    void* argument = va_arg(args, void*);
    va_end(args);
    if (request != SMI_STREAM_IOC_GET_FIFO_MULT)
        return __real_ioctl(fd, request, argument);
    ++fifo_multiplier_reads;
    if (fail_fifo_query) { errno=ENOTTY; return -1; }
    assert(argument);
    *(int*)argument=fifo_multiplier;
    return 0;
}
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
static float test_rx_rate;
static cariboulite_radio_state_st* rate_rx_radio;
int __wrap_cariboulite_radio_set_rx_sample_rate_flt(cariboulite_radio_state_st* r, float fs) {
    test_rx_rate=fs; rate_rx_radio=r; return 0;
}
static float test_tx_rate = 4000000;
static cariboulite_radio_state_st* rate_tx_radio;
static int forced_tx_gap = -1;
static unsigned tx_gap_reads;
int __wrap_cariboulite_radio_set_tx_samp_cutoff_flt(cariboulite_radio_state_st* r, float fs) {
    test_tx_rate=fs; rate_tx_radio=r; return 0;
}
int __wrap_cariboulite_radio_get_tx_samp_cutoff_flt(cariboulite_radio_state_st* r, float* fs) { *fs=test_tx_rate; return 0; }
int __wrap_caribou_fpga_get_sys_ctrl_tx_sample_gap(caribou_fpga_st* f, uint8_t* gap) {
    ++tx_gap_reads;
    *gap=forced_tx_gap<0 ? 4000000/test_tx_rate-1 : forced_tx_gap;
    return 0;
}
int __wrap_cariboulite_radio_set_tx_power(cariboulite_radio_state_st* r, int power) { return 0; }
static const struct {
    unsigned fs;
    size_t frame_samples;
    uint8_t tx_gap;
} monitor_rates[] = {
    {1000000, 10000, 3}, {2000000, 20000, 1}, {4000000, 40000, 0}
};
static const unsigned monitor_tail_padding[] = {227, 114, 57};
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
        if (tx->tx_ctrl.inj.frames_left>0) {
            // Emulate enqueue followed by complete writer acceptance, before
            // signalling completion of the injected sequence.
            tx->tx_ctrl.inj.last_sequence += tx->tx_ctrl.inj.frames_left;
            atomic_store(&tx->tx_ctrl.written_sequence,
                         tx->tx_ctrl.inj.last_sequence);
            tx->tx_ctrl.inj.frames_left=0;
        }
        pthread_mutex_unlock(&g_tx_injection_lock);
        usleep(1000);
    }
    return NULL;
}
int main(void) {
    test_rssi_capture();
    sys_st sys = {0};
    sys.radio_low.sys = sys.radio_high.sys = &sys;
    sys.radio_low.type = cariboulite_channel_s1g;
    sys.radio_high.type = cariboulite_channel_hif;
    sys.board_info.numeric_product_id = system_type_cariboulite_full;
    double parsed=123;
    assert(monitor_parse_frequency("430.125",&sys.radio_high,&parsed) && parsed==430125000);
    assert(monitor_parse_frequency(" 145.500 ",&sys.radio_high,&parsed) && parsed==145500000);
    const char* invalid[]={"", "nan", "inf", "-1", "0", "6000", "430foo", "1e999", "430 100"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
        assert(!monitor_parse_frequency(invalid[i],&sys.radio_high,&parsed));
        assert(parsed==145500000);
    }
    sys.board_info.numeric_product_id = system_type_cariboulite_ism;
    assert(!monitor_parse_frequency("430",&sys.radio_high,&parsed) && parsed==145500000);
    assert(monitor_parse_frequency("2385",&sys.radio_high,&parsed) && parsed==2385000000);
    assert(monitor_parse_frequency("2495",&sys.radio_high,&parsed) && parsed==2495000000);
    assert(!monitor_parse_frequency("2495.001",&sys.radio_high,&parsed) && parsed==2495000000);
    // The S1G connector has the same two native bands on both board variants.
    for(unsigned board=0;board<2;++board) {
        sys.board_info.numeric_product_id = board ? system_type_cariboulite_full : system_type_cariboulite_ism;
        assert(monitor_parse_frequency("377",&sys.radio_low,&parsed) && parsed==CARIBOULITE_S1G_MIN1);
        assert(monitor_parse_frequency("530",&sys.radio_low,&parsed) && parsed==CARIBOULITE_S1G_MAX1);
        assert(monitor_parse_frequency("779",&sys.radio_low,&parsed) && parsed==CARIBOULITE_S1G_MIN2);
        assert(monitor_parse_frequency("1020",&sys.radio_low,&parsed) && parsed==CARIBOULITE_S1G_MAX2);
        assert(monitor_parse_frequency(" 430.125 \t",&sys.radio_low,&parsed) && parsed==430125000);
        const char* invalid_s1g[]={"376.999", "530.001", "600", "778.999", "1020.001", "2385"};
        for(unsigned i=0;i<sizeof(invalid_s1g)/sizeof(*invalid_s1g);++i) {
            assert(!monitor_parse_frequency(invalid_s1g[i],&sys.radio_low,&parsed));
            assert(parsed==430125000);
        }
        for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) {
            assert(!monitor_parse_frequency(invalid[i],&sys.radio_low,&parsed));
            assert(parsed==430125000);
        }
    }
    cariboulite_radio_state_st radio = {.sys=&sys, .type=cariboulite_channel_s1g};
    rx_pipeline_t p;
    rx_params_t par = {.freq_hz=430100000,.pcm_dev="null", .fs_rf=4000000, .fs_audio=48000};
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
    for(unsigned rate=0;rate<sizeof(monitor_rates)/sizeof(*monitor_rates);++rate) {
        par.fs_rf=monitor_rates[rate].fs;
        assert(rx_pipeline_init(&p,&sys,&radio,&par)==0);
        assert(rx_pipeline_start(&p)==0);
        assert(p.rx_ctrl.rx_buffer_size==monitor_rates[rate].frame_samples);
        assert(test_rx_rate==monitor_rates[rate].fs && rate_rx_radio==&radio);
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
    tx_params_t tp = {.freq_hz=430100000,.tone_mode=true, .f_dev_hz=2500, .out_scale=4000};
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
    for(unsigned rate=0;rate<sizeof(monitor_rates)/sizeof(*monitor_rates);++rate) {
        unsigned fs=monitor_rates[rate].fs;
        forced_tx_gap=monitor_rates[rate].tx_gap;
        unsigned before_gap_reads=tx_gap_reads;
        tx_params_t split_tx={.tone_mode=true,.f_dev_hz=2500,.out_scale=4000,
            .freq_hz=430125000,.rf_fs=fs};
        rx_params_t split_rx={.pcm_dev="null",.freq_hz=868500000,.fs_rf=fs,.fs_audio=48000};
        assert(monitor_init_pipelines(&tx,&p,&sys,&sys.radio_high,&split_tx,&split_rx));
        assert(tx.radio==&sys.radio_high && p.radio==&sys.radio_high);
        assert(tx.tx_ctrl.radio==&sys.radio_high && tx.dsp_ctrl.tx==&tx.tx_ctrl);
        assert(p.rx_ctrl.radio==&sys.radio_high);
        assert(tx_pipeline_frame_samples(&tx)==monitor_rates[rate].frame_samples);
        assert(tx.tx_ctrl.tail_padding_frames==monitor_tail_padding[rate]);
        assert(rx_pipeline_frame_samples(&p)==monitor_rates[rate].frame_samples);
        assert(test_tx_rate==fs && rate_tx_radio==&sys.radio_high);
        assert(tx_gap_reads==before_gap_reads+1);
        assert(split_tx.freq_hz==430125000 && split_rx.freq_hz==868500000);
        assert(tuned_radio==&sys.radio_high && tuned_frequency==split_rx.freq_hz);
        pthread_t consumer;
        assert(__real_pthread_create(&consumer,NULL,consume_injection,&tx)==0);
        assert(monitor_start_tx(&tx,&p,&split_tx)==0);
        assert(tx.running && !p.running && tuned_frequency==split_tx.freq_hz);
        assert(tuned_radio==&sys.radio_high && activated_radio==&sys.radio_high);
        assert(last_stream==smi_stream_tx_channel);
        assert(monitor_start_rx(&tx,&p,&split_rx)==0);
        assert(p.running && !tx.running && tuned_frequency==split_rx.freq_hz);
        assert(tuned_radio==&sys.radio_high && activated_radio==&sys.radio_high);
        assert(last_stream==smi_stream_rx_channel_1 && p.rx_ctrl.radio==&sys.radio_high);
        assert(test_rx_rate==fs && rate_rx_radio==&sys.radio_high);
        fail_tune=true;
        assert(monitor_start_tx(&tx,&p,&split_tx)!=0);
        assert(!tx.running && !p.running && !hardware_active);
        assert(monitor_start_rx(&tx,&p,&split_rx)!=0);
        assert(!p.running && !hardware_active);
        fail_tune=false;
        assert(monitor_start_rx(&tx,&p,&split_rx)==0);
        assert(tuned_frequency==split_rx.freq_hz);
        assert(last_stream==smi_stream_rx_channel_1);
        assert(__real_pthread_cancel(consumer)==0);
        assert(__real_pthread_join(consumer,NULL)==0);
        rx_pipeline_destroy(&p); tx_pipeline_destroy(&tx);
        check_clean(&p);
    }
    forced_tx_gap=-1;
    tp.mic_dev=NULL;
    assert(tx_pipeline_init(&tx,&sys,&radio,&tp)==0);
    tx_pipeline_destroy(&tx);

    for(unsigned rate=0;rate<sizeof(monitor_rates)/sizeof(*monitor_rates);++rate) {
        unsigned fs=monitor_rates[rate].fs;
        forced_tx_gap=monitor_rates[rate].tx_gap;
        tp.rf_fs=fs;
        assert(tx_pipeline_init(&tx,&sys,&radio,&tp)==0);
        assert(tx.tx_ctrl.frame_samples==monitor_rates[rate].frame_samples);
        assert(tx.tx_ctrl.tail_padding_frames==monitor_tail_padding[rate]);
        assert(test_tx_rate==fs);
        float audio[480]={0};
        nbfm_push_audio(tx.tx_ctrl.fm,audio,480);
        assert(nbfm_pull_iq(tx.tx_ctrl.fm,tx.tx_ctrl.iq_rf,fs/100)==fs/100);
        tx_pipeline_destroy(&tx);
    }
    // Tail padding accounts for the kernel FIFO and cyclic DMA, and falls back
    // conservatively when old drivers cannot report valid buffer information.
    forced_tx_gap=-1;
    const struct {
        size_t native;
        int multiplier;
        bool query_error;
        unsigned rate;
        unsigned expected_frames;
    } padding_cases[] = {
        {131072, 2, false, 1000000, 43},
        {131072, 2, false, 2000000, 25},
        {131072, 32, false, 1000000, 436},
        {131072, 32, false, 2000000, 218},
        {131072, 32, false, 4000000, 109},
        {131072, 16, true, 1000000, 436},
        {131072, 0, false, 1000000, 436},
        {131072, 1, false, 1000000, 436},
        {131072, 33, false, 1000000, 436},
        {131072, -1, false, 1000000, 436},
        {0, 16, false, 1000000, 227},
        {65536, 16, false, 1000000, 114},
        {4096, 2, false, 1000000, 25}
    };
    for(unsigned i=0;i<sizeof(padding_cases)/sizeof(*padding_cases);++i) {
        native_batch_samples=padding_cases[i].native;
        fifo_multiplier=padding_cases[i].multiplier;
        fail_fifo_query=padding_cases[i].query_error;
        tp.rf_fs=padding_cases[i].rate;
        unsigned before_native=native_batch_reads, before_fifo=fifo_multiplier_reads;
        assert(tx_pipeline_init(&tx,&sys,&radio,&tp)==0);
        assert(tx.tx_ctrl.tail_padding_frames==padding_cases[i].expected_frames);
        assert(native_batch_reads==before_native+1 && fifo_multiplier_reads==before_fifo+1);
        tx_pipeline_destroy(&tx);
        assert(!tx.inited && !tx.running && !hardware_active);
        for(int j=1;j<=creates;++j) assert(!live[j]);
    }
    native_batch_samples=131072;
    fifo_multiplier=16;
    fail_fifo_query=false;
    // At 1 MS/s, stale gaps from the 2/4 MS/s settings must block TX startup.
    tp.rf_fs=1000000;
    for(forced_tx_gap=0;forced_tx_gap<=1;++forced_tx_gap) {
        int before_creates=creates;
        unsigned before_gap_reads=tx_gap_reads;
        assert(tx_pipeline_init(&tx,&sys,&radio,&tp)<0);
        assert(tx_gap_reads==before_gap_reads+1 && creates==before_creates);
        assert(!tx.inited && !tx.running && !hardware_active);
        assert(!tx.tx_ctrl.fm && !tx.tx_ctrl.a48k && !tx.tx_ctrl.iq_rf);
        tx_pipeline_destroy(&tx);
    }
    forced_tx_gap=-1;
    tp.rf_fs=0;

    par.pcm_dev="null";
    fail_demod_create=true;
    assert(rx_pipeline_init(&p,&sys,&radio,&par)<0); check_clean(&p);
    fail_demod_create=false;
    for(int failure=1;failure<=4;++failure) {
        rx_pipeline_t rx={0};
        fail_create=creates+failure;
        assert(!monitor_init_pipelines(&tx,&rx,&sys,&sys.radio_high,&tp,&par));
        assert(!tx.inited && !rx.inited);
        for(int i=1;i<=creates;++i) assert(!live[i]);
        fail_create=0;
    }
    assert(monitor_init_pipelines(&tx,&p,&sys,&sys.radio_high,&tp,&par));
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
    for (unsigned rate=0; rate<sizeof(monitor_rates)/sizeof(*monitor_rates); ++rate) {
        par.fs_rf = monitor_rates[rate].fs;
        rx_params_t saved=par;
        assert(monitor_cycle_rx_mode(&tx,&p,&loopback,&sys,&par)==0);
        assert(par.mode==FM_MODE_WBFM && p.demod.mode==FM_MODE_WBFM && !p.running);
        assert(p.radio==&sys.radio_high && p.rx_ctrl.radio==&sys.radio_high);
        assert(tx.radio==&sys.radio_high && tx.tx_ctrl.radio==&sys.radio_high);
        assert(par.freq_hz==saved.freq_hz && par.pcm_dev==saved.pcm_dev &&
               par.pcm_gain==saved.pcm_gain && par.deemph_tau_s==saved.deemph_tau_s &&
               par.fs_rf==saved.fs_rf && par.fs_audio==saved.fs_audio &&
               par.noise_squelch_disabled==saved.noise_squelch_disabled &&
               par.carrier_squelch_enabled==saved.carrier_squelch_enabled);
        assert(p.demod.pcm_gain==saved.pcm_gain && p.demod.deemph_tau==saved.deemph_tau_s);
        assert(!audio_demod_capabilities(p.demod.dsp));
        rx_pipeline_set_squelch(&p,true,true);
        assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_CARRIER);
        assert(monitor_start_rx(&tx,&p,&par)==0);
        assert(last_stream==smi_stream_rx_channel_1 && activated_radio==&sys.radio_high);
        assert(p.rx_ctrl.rx_buffer_size==monitor_rates[rate].frame_samples);
        assert(test_rx_rate==monitor_rates[rate].fs && rate_rx_radio==&sys.radio_high);
        rx_pipeline_stop(&p);
        assert(monitor_cycle_rx_mode(&tx,&p,&loopback,&sys,&par)==0);
        assert(par.mode==FM_MODE_NBFM && p.demod.mode==FM_MODE_NBFM && !p.running);
        assert(p.radio==&sys.radio_high && p.rx_ctrl.radio==&sys.radio_high);
        assert(atomic_load(&p.demod.squelch_flags)==RX_SQUELCH_NOISE);
        assert(audio_demod_capabilities(p.demod.dsp)==AUDIO_DEMOD_CAP_NOISE_SQUELCH);
        assert(par.freq_hz==saved.freq_hz && par.pcm_gain==saved.pcm_gain &&
               par.deemph_tau_s==saved.deemph_tau_s && par.fs_rf==saved.fs_rf);
        assert(monitor_start_rx(&tx,&p,&par)==0);
        assert(last_stream==smi_stream_rx_channel_1 && activated_radio==&sys.radio_high);
        assert(p.rx_ctrl.rx_buffer_size==monitor_rates[rate].frame_samples);
        assert(test_rx_rate==monitor_rates[rate].fs && rate_rx_radio==&sys.radio_high);
        rx_pipeline_stop(&p);
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
    puts("PASS: S1G/HiF frequency bands, HiF monitor routing/rates/modes, TX tail padding plans, TX/RX lifecycle, startup failures, FIFO timing and cancellation");
}
