#define nbfm_demod legacy_nbfm_demod
#define nbfm_demod_t legacy_nbfm_demod_t
#define nbfm_demod_config_t legacy_nbfm_demod_config_t
#define nbfm_demod_result_t legacy_nbfm_demod_result_t
#define fm_demod_mode_t legacy_fm_demod_mode_t
#define FM_MODE_NBFM legacy_FM_MODE_NBFM
#define FM_MODE_WBFM legacy_FM_MODE_WBFM
#define nbfm_demod_create legacy_nbfm_demod_create
#define wbfm_demod_create legacy_wbfm_demod_create
#define nbfm_demod_reset legacy_nbfm_demod_reset
#define nbfm_demod_set_audio legacy_nbfm_demod_set_audio
#define nbfm_demod_process legacy_nbfm_demod_process
#define nbfm_demod_process_with_raw legacy_nbfm_demod_process_with_raw
#define nbfm_demod_destroy legacy_nbfm_demod_destroy
#include "fixtures/legacy_wbfm_demod/nbfm_demod.h"
#undef nbfm_demod
#undef nbfm_demod_t
#undef nbfm_demod_config_t
#undef nbfm_demod_result_t
#undef fm_demod_mode_t
#undef FM_MODE_NBFM
#undef FM_MODE_WBFM
#undef nbfm_demod_create
#undef wbfm_demod_create
#undef nbfm_demod_reset
#undef nbfm_demod_set_audio
#undef nbfm_demod_process
#undef nbfm_demod_process_with_raw
#undef nbfm_demod_destroy
#include "nbfm_demod.h"
#include "math_compat.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>

void* __real_calloc(size_t, size_t);
void __real_free(void*);
static size_t live, allocations, bytes;
static int fail_allocation;
void* __wrap_calloc(size_t n, size_t size)
{
    if (fail_allocation) { errno=ENOMEM; return NULL; }
    void* p=__real_calloc(n,size);
    if (p) { ++live; ++allocations; bytes+=n*size; }
    return p;
}
void __wrap_free(void* p) { if(p) { assert(live); --live; } __real_free(p); }

int main(void)
{
    size_t total = 0;
    for (unsigned rate=2000000; rate<=4000000; rate*=2) {
        size_t n=rate/5;
        iq16_t *iq=malloc(n*sizeof(*iq));
        assert(iq);
        // Include weak/noisy RF and silence, not only clean synthetic FM.
        for (int scenario=0; scenario<4; ++scenario) {
        unsigned rng=0x12345678;
        double phase=0;
        for (size_t i=0; i<n; ++i) {
            double t=(double)i/rate;
            phase=remainder(phase+2*M_PI*(13000+75000*sin(2*M_PI*1000*t)
                  +7500*sin(2*M_PI*19000*t))/rate,2*M_PI);
            double amplitude=scenario==1?1200:12000;
            rng=rng*1664525u+1013904223u;
            double ni=(double)(rng>>16)-32768;
            rng=rng*1664525u+1013904223u;
            double nq=(double)(rng>>16)-32768;
            iq[i]=(iq16_t){(int16_t)lrint(amplitude*cos(phase)+(scenario==1?ni*0.1:0)),
                          (int16_t)lrint(amplitude*sin(phase)+(scenario==1?nq*0.1:0))};
            if(scenario==2) iq[i]=(iq16_t){(int16_t)ni,(int16_t)nq};
            if(scenario==3 || (scenario==1 && i%10000<80)) iq[i]=(iq16_t){0,0};
        }
        for (int mode=0; mode<2; ++mode) {
            nbfm_demod_config_t config={rate,48000,0,10000};
            legacy_nbfm_demod_config_t old_config={rate,48000,0,10000};
            fail_allocation=1;
            assert(!(mode?wbfm_demod_create(&config):nbfm_demod_create(&config)));
            assert(errno==ENOMEM && !live);
            fail_allocation=0;
            size_t initial_bytes=bytes;
            nbfm_demod_t *a=mode?wbfm_demod_create(&config):nbfm_demod_create(&config);
            size_t current_bytes=bytes-initial_bytes;
            initial_bytes=bytes;
            legacy_nbfm_demod_t *b=mode?legacy_wbfm_demod_create(&old_config):legacy_nbfm_demod_create(&old_config);
            assert(a && b);
            size_t old_bytes=bytes-initial_bytes;
            double current_cpu=0,old_cpu=0,current_peak=0,old_peak=0;
            size_t created_allocations=allocations;
            for (int pass=0; pass<3; ++pass) {
                nbfm_demod_reset(a); legacy_nbfm_demod_reset(b);
                size_t used=0, call=0;
                while (used<n) {
                    size_t count=1+(call*7919)%10007;
                    if (count>n-used) count=n-used;
                    size_t cap=call%11==0?0:1+call%127;
                    int16_t pcm_a[128],pcm_b[128];
                    float raw_a[128],raw_b[128];
                    if (call==31 || call==97) {
                        nbfm_demod_reset(a); legacy_nbfm_demod_reset(b);
                    }
                    if (call%23==0) {
                        float tau=call%3==0?0:call%3==1?50e-6f:75e-6f;
                        float gain=call%2?10000:1000000;
                        assert(nbfm_demod_set_audio(a,tau,gain)==legacy_nbfm_demod_set_audio(b,tau,gain));
                    }
                    double correction=(pass-1)*0.0005;
                    int tap=call%2;
                    clock_t start=clock();
                    nbfm_demod_result_t x=nbfm_demod_process_with_raw(a,iq+used,count,pcm_a,tap?raw_a:NULL,cap,correction);
                    double elapsed=(double)(clock()-start)/CLOCKS_PER_SEC;
                    current_cpu+=elapsed;
                    if(elapsed>current_peak) current_peak=elapsed;
                    start=clock();
                    legacy_nbfm_demod_result_t y=legacy_nbfm_demod_process_with_raw(b,iq+used,count,pcm_b,tap?raw_b:NULL,cap,correction);
                    elapsed=(double)(clock()-start)/CLOCKS_PER_SEC;
                    old_cpu+=elapsed;
                    if(elapsed>old_peak) old_peak=elapsed;
                    assert(allocations==created_allocations);
                    assert(x.error==y.error && x.consumed==y.consumed && x.produced==y.produced);
                    assert(!x.error && x.consumed<=count && x.produced<=cap);
                    assert(!memcmp(pcm_a,pcm_b,x.produced*sizeof(*pcm_a)));
                    if(tap) assert(!memcmp(raw_a,raw_b,x.produced*sizeof(*raw_a)));
                    assert(cap?x.consumed>0:!x.consumed && !x.produced);
                    used+=x.consumed; total+=x.produced; ++call;
                }
            }
            nbfm_demod_destroy(a); legacy_nbfm_demod_destroy(b);
            assert(!live);
            printf("scenario %d %s %u Hz: state bytes %zu (reference %zu), CPU %.3fs (reference %.3fs), max call %.3fms (reference %.3fms)\n",
                   scenario,mode?"WBFM":"NBFM",rate,current_bytes,old_bytes,current_cpu,old_cpu,1000*current_peak,1000*old_peak);
        }
        }
        __real_free(iq);
    }
    printf("FM frozen reference: %zu exact PCM samples and progress/raw comparisons passed\n",total);
}
