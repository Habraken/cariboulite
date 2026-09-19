/* Compare with the accepted step-5 modulator, frozen as a numerical oracle. */
#define nbfm_mod legacy_nbfm_mod
#define nbfm_mod_t legacy_nbfm_mod_t
#define nbfm_cfg_t legacy_nbfm_cfg_t
#define nbfm_create legacy_nbfm_create
#define nbfm_destroy legacy_nbfm_destroy
#define nbfm_push_audio legacy_nbfm_push_audio
#define nbfm_pull_iq legacy_nbfm_pull_iq
#include "fixtures/legacy_nbfm_mod/nbfm_mod.c"
#undef nbfm_mod
#undef nbfm_mod_t
#undef nbfm_cfg_t
#undef nbfm_create
#undef nbfm_destroy
#undef nbfm_push_audio
#undef nbfm_pull_iq
#include "nbfm_mod.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

static int fail_allocation, live_allocations;
void* __real_calloc(size_t, size_t);
void __real_free(void*);
void* __wrap_calloc(size_t n, size_t size)
{
    if (fail_allocation && --fail_allocation==0) return NULL;
    void* p=__real_calloc(n,size);
    if (p) ++live_allocations;
    return p;
}
void __wrap_free(void* p)
{
    if (p) --live_allocations;
    __real_free(p);
}
static iq16_t expected[40000], actual[40000];
static void equal_iq(size_t count)
{
    for(size_t i=0;i<count;++i) {
        assert(expected[i].i==actual[i].i && expected[i].q==actual[i].q);
    }
}
static void compare_waveforms(unsigned rate, int linear, double tau)
{
    nbfm_cfg_t cfg={48000,rate,2500,tau,4000,linear};
    legacy_nbfm_cfg_t old_cfg={48000,rate,2500,tau,4000,linear};
    legacy_nbfm_mod_t* old=legacy_nbfm_create(&old_cfg); assert(old);
    nbfm_mod_t* m=nbfm_create(&cfg); assert(m);
    nbfm_mod_t* split=nbfm_create(&cfg); assert(split);
    for(size_t block=0;block<12;++block) {
        float audio[480];
        for(size_t i=0;i<480;++i) {
            // Tone, speech-like mixture, clipping and silence; phase continuous.
            double time=(block*480+i)/48000.0;
            audio[i]=block==3?0:(float)(0.6*sin(2*M_PI*600*time)+0.8*sin(2*M_PI*1900*time));
        }
        assert(legacy_nbfm_push_audio(old,audio,480)==480);
        assert(legacy_nbfm_pull_iq(old,expected,rate/100)==rate/100);
        nbfm_result_t r=nbfm_process(m,audio,480,actual,rate/100);
        assert(!r.error && r.consumed==480 && r.produced==rate/100 && !r.held_audio);
        assert(nbfm_buffered_audio(m)==0);
        equal_iq(r.produced);
        // Feed the same block in arbitrary input pieces, then use tiny outputs.
        size_t offset=0, turn=0;
        while(offset<480) {
            const size_t sizes[]={1,7,31,113};
            size_t count=sizes[turn++%4]; if(count>480-offset) count=480-offset;
            r=nbfm_process(split,audio+offset,count,NULL,0);
            assert(!r.error && r.consumed==count && !r.produced && !r.held_audio);
            offset+=r.consumed;
        }
        offset=0;
        while(offset<rate/100) {
            const size_t sizes[]={1,2,17,997,61};
            size_t count=sizes[turn++%5]; if(count>rate/100-offset) count=rate/100-offset;
            r=nbfm_process(split,NULL,0,actual+offset,count);
            assert(!r.error && !r.consumed && r.produced==count && !r.held_audio);
            offset+=r.produced;
        }
        equal_iq(rate/100);
    }
    // Underrun must retain the historical signal and explicitly count held ticks.
    legacy_nbfm_pull_iq(old,expected,rate/100);
    nbfm_result_t r=nbfm_process(m,NULL,0,actual,rate/100);
    assert(r.held_audio==480 && r.produced==rate/100 && !r.error);
    equal_iq(r.produced);
    legacy_nbfm_destroy(old); nbfm_destroy(m); nbfm_destroy(split);
    printf("PASS: %u Hz, interpolation=%d, tau=%g: 13 blocks identical, split buffers and underrun\n",rate,linear,tau);
}
static void reset_and_errors(unsigned rate)
{
    nbfm_cfg_t cfg={48000,rate,2500,75e-6,32767,1};
    nbfm_mod_t* m=nbfm_create(&cfg); assert(m);
    nbfm_mod_t* fresh=nbfm_create(&cfg); assert(fresh);
    float audio[4097]; for(size_t i=0;i<4097;++i) audio[i]=0.75;
    nbfm_result_t r=nbfm_process(m,audio,4097,NULL,0);
    assert(!r.error && r.consumed==4096 && !r.produced && nbfm_buffered_audio(m)==4096);
    r=nbfm_process(m,audio,1,NULL,0);
    assert(!r.error && !r.consumed && !r.produced);
    r=nbfm_process(m,audio,1,actual,rate/100);
    assert(!r.error && !r.consumed && r.produced==rate/100 && !r.held_audio);
    assert(nbfm_buffered_audio(m)==4096-480);
    r=nbfm_process(m,audio,1,actual,13); // retry previously unaccepted input
    assert(r.consumed==1 && r.produced==13 && !r.held_audio);
    nbfm_reset(m); assert(nbfm_buffered_audio(m)==0);
    r=nbfm_process(m,audio,480,actual,rate/100);
    nbfm_result_t f=nbfm_process(fresh,audio,480,expected,rate/100);
    assert(r.consumed==f.consumed && r.produced==f.produced && !r.held_audio);
    equal_iq(r.produced);
    // Invalid arguments/nonfinite samples must neither change state nor write output.
    actual[0]=(iq16_t){123,456};
    audio[1]=NAN;
    assert(nbfm_process(m,audio,2,actual,1).error==-EINVAL);
    assert(actual[0].i==123 && actual[0].q==456);
    assert(nbfm_process(m,NULL,1,actual,1).error==-EINVAL);
    assert(nbfm_process(m,audio,0,NULL,1).error==-EINVAL);
    audio[1]=0.75;
    nbfm_process(m,audio,480,actual,rate/100);
    nbfm_process(fresh,audio,480,expected,rate/100); equal_iq(rate/100);
    nbfm_reset(m); nbfm_reset(m);
    r=nbfm_process(m,NULL,0,actual,rate/100);
    assert(r.held_audio==480);
    for(size_t i=0;i<r.produced;++i) assert(actual[i].i==32767 && actual[i].q==0);
    nbfm_destroy(m); nbfm_destroy(fresh);
    printf("PASS: %u Hz, queue saturation/retry, transactional errors, reset and full-scale IQ\n",rate);
}
int main(void)
{
    nbfm_cfg_t cfg={48000,4000000,2500,0,12000,1};
    for(int field=0;field<10;++field) {
        nbfm_cfg_t bad=cfg;
        switch(field) {
        case 0: bad.audio_fs=44100; break;
        case 1: bad.rf_fs=3000000; break;
        case 2: bad.f_dev_hz=NAN; break;
        case 3: bad.f_dev_hz=-1; break;
        case 4: bad.f_dev_hz=24001; break;
        case 5: bad.preemph_tau_s=-1; break;
        case 6: bad.preemph_tau_s=INFINITY; break;
        case 7: bad.out_scale=NAN; break;
        case 8: bad.out_scale=32768; break;
        case 9: bad.linear_interp=2; break;
        }
        assert(!nbfm_create(&bad) && errno==EINVAL && live_allocations==0);
    }
    for(int allocation=1;allocation<=2;++allocation) {
        fail_allocation=allocation;
        assert(!nbfm_create(&cfg) && errno==ENOMEM && live_allocations==0);
    }
    assert(!nbfm_process(NULL,NULL,0,NULL,0).consumed);
    assert(nbfm_process(NULL,NULL,0,NULL,0).error==-EINVAL);
    assert(nbfm_push_audio(NULL,NULL,0)==0 && errno==EINVAL);
    assert(nbfm_pull_iq(NULL,NULL,0)==0 && errno==EINVAL);
    nbfm_reset(NULL); nbfm_destroy(NULL);
    nbfm_mod_t* defaults=nbfm_create(NULL); assert(defaults);
    nbfm_result_t d=nbfm_process(defaults,NULL,0,actual,1);
    assert(d.produced==1 && actual[0].i==12000 && actual[0].q==0);
    nbfm_destroy(defaults);
    for(unsigned rate=2000000;rate<=4000000;rate*=2) {
        for(int linear=0;linear<=1;++linear) {
            compare_waveforms(rate,linear,0);
            compare_waveforms(rate,linear,75e-6);
        }
        reset_and_errors(rate);
    }
    assert(live_allocations==0);
    puts("PASS: configuration validation, allocation failure cleanup, defaults, zero lengths");
}
