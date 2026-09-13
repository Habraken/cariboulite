/* Include the implementation to exercise its private pipeline and FIFO types. */
#include "app_menu.c"

static bool real_threads, live[1024];
static int creates, fail_create, joins, hardware_active;
static bool fail_malloc, fail_calloc;
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
int __wrap_cariboulite_radio_set_frequency(cariboulite_radio_state_st* r, bool b, double* f) { return 0; }
int __wrap_cariboulite_radio_activate_channel(cariboulite_radio_state_st* r, cariboulite_channel_dir_en d, bool a) {
    hardware_active = a; return 0;
}
int __wrap_caribou_smi_set_driver_streaming_state(caribou_smi_st* s, smi_stream_state_en e) { return 0; }
int __wrap_caribou_fpga_set_io_ctrl_mode(caribou_fpga_st* f, uint8_t d, caribou_fpga_io_ctrl_rfm_en m) { return 0; }
int __wrap_cariboulite_radio_read_samples(cariboulite_radio_state_st* r,
        cariboulite_sample_complex_int16* b, cariboulite_sample_meta* m, size_t n) {
    pthread_barrier_wait(&reader_ready);
    for (;;) { pthread_testcancel(); usleep(1000); }
    return 0;
}
int __wrap_cariboulite_radio_set_tx_power(cariboulite_radio_state_st* r, int power) { return 0; }
static void check_clean(rx_pipeline_t* p) {
    assert(!p->inited && !p->running && !hardware_active);
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
int main(void) {
    sys_st sys = {0}; cariboulite_radio_state_st radio = {0}; rx_pipeline_t p;
    rx_params_t par = {.pcm_dev="null", .fs_rf=4000000, .fs_audio=48000};
    assert(rx_pipeline_init(&p,&sys,&radio,&par) == 0);
    rx_pipeline_destroy(&p); check_clean(&p); /* no reader ever created */
    assert(rx_pipeline_init(&p,&sys,&radio,&par) == 0);
    for (int i=0; i<20; ++i) {
        assert(rx_pipeline_start(&p) == 0);
        rx_pipeline_stop(&p); rx_pipeline_stop(&p);
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
        assert(!tx.tx_ctrl.mic && !tx.tx_ctrl.fm && !tx.tx_ctrl.a48k && !tx.tx_ctrl.iq4m);
        for(int i=1;i<=creates;++i) assert(!live[i]);
        fail_create=0;fail_calloc=false;fail_calloc_count=0;
    }
    tp.mic_dev=NULL;
    assert(tx_pipeline_init(&tx,&sys,&radio,&tp)==0);
    tx_pipeline_destroy(&tx);

    par.pcm_dev="null";
    for(int failure=1;failure<=4;++failure) {
        rx_pipeline_t rx={0};
        fail_create=creates+failure;
        assert(!monitor_init_pipelines(&tx,&rx,&sys,&tp,&par));
        assert(!tx.inited && !rx.inited);
        for(int i=1;i<=creates;++i) assert(!live[i]);
        fail_create=0;
    }
    assert(monitor_init_pipelines(&tx,&p,&sys,&tp,&par));
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
