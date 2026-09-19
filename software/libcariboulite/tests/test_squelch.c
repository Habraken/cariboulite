#define _GNU_SOURCE
#include "noise_squelch.h"
#include "carrier_squelch.h"
#include "demod_worker.h"
#include "pipeline_transport.h"
#include "nbfm_mod.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned block, out_block, rate, mode, seed;
static double energy[4];
static nbfm_demod_ctrl_t* worker;
static nbfm_mod_t* mod;
static double phase;
int set_rt_and_affinity_prio(int prio, int cpu) { (void)prio; (void)cpu; return 0; }
bool rf10_fifo_get(rf10_fifo_t* f, rf10_frame_t* frame, int timeout)
{
    (void)f; (void)timeout;
    if (block == 240) { worker->active = false; return false; }
    unsigned stage = block / 60;
    float audio[480];
    for (unsigned i=0; i<480; ++i) { audio[i]=0.4f*sinf(phase); phase+=2*M_PI*600/48000; }
    nbfm_result_t r=nbfm_process(mod,audio,480,frame->data,rate/100);
    assert(r.consumed==480 && r.produced==rate/100 && !r.error);
    if ((mode == 0 || mode == 3) && stage == 1) {
        // Broadband RF noise, not just a manufactured audio detection signal.
        for (unsigned i=0;i<rate/100;++i) {
            seed=seed*1664525u+1013904223u; frame->data[i].i=(int16_t)(seed>>16);
            seed=seed*1664525u+1013904223u; frame->data[i].q=(int16_t)(seed>>16);
        }
    }
    frame->rssi_valid = !(mode == 1 && stage == 3);
    frame->rssi_dbm = (mode == 3 ? stage == 3 : stage == 1 || (mode == 2 && stage == 3)) ? -115 : -80;
    // Exercise live bypass while noise is enabled and carrier blocks audio.
    if (mode == 2 && stage == 3) atomic_store(&worker->squelch_flags,0);
    ++block;
    return true;
}
bool aud10_fifo_put(aud10_fifo_t* f, const aud10_frame_t* frame, int timeout)
{
    (void)f; (void)timeout;
    unsigned stage=out_block/60, offset=out_block%60;
    if (offset>=40 && stage<4)
        for (unsigned i=0;i<480;++i) energy[stage]+=(double)frame->pcm[i]*frame->pcm[i];
    ++out_block;
    return true;
}
void aud10_fifo_peek_depth(aud10_fifo_t* f,size_t* count,size_t* cap)
{ (void)f; *count=12; *cap=24; }

static void detectors(void)
{
    noise_squelch_t n={0};
    for(unsigned i=0;i<1439;++i) assert(!noise_squelch_process(&n,0));
    assert(noise_squelch_process(&n,0));
    for(unsigned i=0;i<24000;++i) noise_squelch_process(&n,0.8f*sinf(2*M_PI*3000*i/48000));
    assert(n.open); // speech-band energy must not trip the detector
    for(unsigned i=0;i<2400;++i) noise_squelch_process(&n,0.8f*sinf(2*M_PI*10000*i/48000));
    assert(n.open); // brief noise burst does not close the gate
    for(unsigned i=0;i<24000;++i) noise_squelch_process(&n,0.8f*sinf(2*M_PI*10000*i/48000));
    assert(!n.open);
    for(unsigned i=0;i<24000;++i) noise_squelch_process(&n,0.4f*sinf(2*M_PI*600*i/48000));
    assert(n.open);
    assert(!noise_squelch_process(&n,NAN));
    assert(!noise_squelch_process(&n,INFINITY));
    carrier_squelch_t c={0};
    assert(!carrier_squelch_process(&c,-97,true));
    assert(!carrier_squelch_process(&c,-97,true));
    assert(carrier_squelch_process(&c,-97,true));
    for(unsigned i=0;i<50;++i) assert(carrier_squelch_process(&c,-100,true));
    for(unsigned i=0;i<50;++i) assert(carrier_squelch_process(&c,-102,true)); // exact close boundary retains state
    for(unsigned i=0;i<14;++i) assert(carrier_squelch_process(&c,-103,true));
    assert(!carrier_squelch_process(&c,-103,true));
    for(unsigned i=0;i<50;++i) assert(!carrier_squelch_process(&c,-100,true));
    for(unsigned i=0;i<3;++i) carrier_squelch_process(&c,-80,true);
    assert(c.open && !carrier_squelch_process(&c,127,true));
    assert(!carrier_squelch_process(&c,NAN,true));
    assert(!carrier_squelch_process(&c,-80,false));
}
static void raw_tap(void)
{
    nbfm_demod_config_t a={rate,48000,50e-6f,8000}, b={rate,48000,0,1234};
    nbfm_demod_t *d1=nbfm_demod_create(&a), *d2=nbfm_demod_create(&b);
    assert(d1 && d2);
    iq16_t iq[40000];
    for(unsigned i=0;i<rate/100;++i) {
        double ph=0.5*sin(2*M_PI*600*i/rate);
        iq[i]=(iq16_t){(int16_t)(10000*cos(ph)),(int16_t)(10000*sin(ph))};
    }
    float r1[480],r2[480]; int16_t p1[480],p2[480];
    nbfm_demod_result_t x=nbfm_demod_process_with_raw(d1,iq,rate/100,p1,r1,480,0);
    nbfm_demod_result_t y=nbfm_demod_process_with_raw(d2,iq,rate/100,p2,r2,480,0);
    assert(!x.error && !y.error && x.consumed==y.consumed && x.produced==y.produced);
    assert(!memcmp(r1,r2,x.produced*sizeof(float))); // independent of gain and filtering
    assert(memcmp(p1,p2,x.produced*sizeof(int16_t)));
    nbfm_demod_destroy(d1); nbfm_demod_destroy(d2);
}
static void integration(void)
{
    rf10_fifo_t rf={0}; aud10_fifo_t af={0}; audio_sink_t sink={0};
    nbfm_demod_ctrl_t c={.active=true,.fifo_in=&rf,.afifo_out=&af,
        .sink=&sink,.fs_rf=rate,.fs_audio=48000,.pcm_rate=48000,
        .pcm_gain=8000,.deemph_tau=50e-6f};
    atomic_init(&c.squelch_flags, mode==0 ? RX_SQUELCH_NOISE :
        mode==1 ? RX_SQUELCH_CARRIER : RX_SQUELCH_NOISE|RX_SQUELCH_CARRIER);
    nbfm_demod_config_t dc={rate,48000,50e-6f,8000};
    nbfm_cfg_t mc={48000,rate,2500,0,4000,1};
    c.dsp=nbfm_demod_create(&dc); mod=nbfm_create(&mc); assert(c.dsp && mod);
    worker=&c; block=out_block=0; phase=0; seed=123; memset(energy,0,sizeof(energy));
    nbfm_demod_thread(&c);
    assert(out_block>=239); // closed squelch must still enqueue audio on time
    assert(energy[0]>1e8 && energy[1]==0 && energy[2]>1e8);
    if(mode==1 || mode==3) assert(energy[3]==0); else assert(energy[3]>1e8);
    nbfm_demod_destroy(c.dsp); nbfm_destroy(mod);
}
int main(void)
{
    detectors();
    for(rate=2000000;rate<=4000000;rate*=2) {
        raw_tap();
        for(mode=0;mode<4;++mode) integration();
    }
    puts("Squelch detectors and RX worker integration pass at 2/4 MS/s");
}
