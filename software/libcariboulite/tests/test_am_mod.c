#include "tx_pipeline.h"
#include "audio_mod.h"
#include "radio_rates.h"
#include "math_compat.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static double reference(unsigned rate,float depth,float amplitude,float hz,int split_check)
{
    unsigned d=radio_rate_divider(rate);double fs=4000000./d;
    nbfm_cfg_t cfg={48000,rate,2500,0,4000,1};
    nbfm_mod_t* m=audio_mod_create(AUDIO_MOD_AM,&cfg,depth);
    nbfm_mod_t* split=audio_mod_create(AUDIO_MOD_AM,&cfg,depth);assert(m&&split);
    float audio[480];iq16_t* iq=calloc(40000,sizeof(*iq)),*other=calloc(40000,sizeof(*other));
    rf10_frame_t* frame=calloc(1,sizeof(*frame));assert(iq&&other&&frame);
    tx_writer_ctrl_st tx={.fm=m,.a48k=audio,.iq_rf=iq,.rf_rate_hz=rate,.frame_samples=(40000+d-1)/d};
    uint64_t total=0,n=0;double sum=0,power=0,ss=0,cc=0;
    for(unsigned block=0;block<120;block++) {
        for(unsigned j=0;j<480;j++)audio[j]=amplitude*sin(2*M_PI*hz*(block*480+j)/48000.);
        assert(!tx_generate_frame(&tx,frame));
        size_t count=frame->sample_count;
        assert(count==((uint64_t)(block+1)*40000/d-(uint64_t)block*40000/d));
        if(split_check) {
            for(unsigned j=0;j<480;j+=17) {
                size_t k=480-j;if(k>17)k=17;
                nbfm_result_t r=nbfm_process(split,audio+j,k,NULL,0);assert(!r.error&&r.consumed==k);
            }
            for(size_t off=0;off<count;) {
                size_t k=count-off;if(k>113)k=113;
                nbfm_result_t r=nbfm_process(split,NULL,0,other+off,k);
                assert(!r.error&&!r.held_audio&&r.produced==k);off+=k;
            }
            assert(!memcmp(iq,other,count*sizeof(*iq)));
        }
        for(unsigned j=0;j<count;j++,total++) {
            assert(iq[j].q==0 && iq[j].i>=0 && iq[j].i<=4000);
            assert(frame->data[j].i==iq[j].i && frame->data[j].q==0);
            if(total>=fs*.2) {
                double y=iq[j].i/2000.-1,t=total/fs;
                sum+=y;power+=y*y;ss+=y*sin(2*M_PI*hz*t);cc+=y*cos(2*M_PI*hz*t);n++;
            }
        }
    }
    assert(total==UINT64_C(4800000)/d && tx.rf_frame_phase==0);
    double mod=2*hypot(ss,cc)/n;
    assert(fabs(sum/n)<.0001);
    if(depth>0 && amplitude<=1 && hz==1000) {
        double w=M_PI*1000/48000;
        double expected=depth*amplitude/sqrt(1+pow(.3,4))/sqrt(1+pow(1./3,8))*pow(sin(w)/w,2);
        assert(fabs(mod-expected)<.0005);
        double residual=fmax(1e-12,power/n-mod*mod/2);
        assert(10*log10(mod*mod/(2*residual))>50);
    }
    if(depth==0)assert(power==0);
    nbfm_reset(m);nbfm_reset(split);tx.rf_frame_phase=0;
    assert(!tx_generate_frame(&tx,frame));
    nbfm_result_t r=nbfm_process(split,audio,480,other,frame->sample_count);
    assert(!r.error&&!r.held_audio&&!memcmp(iq,other,frame->sample_count*sizeof(*iq)));
    printf("PASS AM TX %u depth %.2f input %.2f tone %.0f: measured modulation %.6f, carrier error %.6f, split/reset exact\n",rate,depth,amplitude,hz,mod,sum/n);
    free(iq);free(other);free(frame);nbfm_destroy(m);nbfm_destroy(split);return mod;
}
int main(void)
{
    nbfm_cfg_t cfg={48000,1000000,2500,0,4000,1};
    assert(!audio_mod_create(AUDIO_MOD_AM,&cfg,NAN));
    assert(!audio_mod_create(AUDIO_MOD_AM,&cfg,-.01f));
    assert(!audio_mod_create(AUDIO_MOD_AM,&cfg,1.01f));
    assert(!audio_mod_create((audio_mod_mode_t)9,&cfg,.8f));
    cfg.preemph_tau_s=50e-6;assert(!audio_mod_create(AUDIO_MOD_AM,&cfg,.8f));cfg.preemph_tau_s=0;
    nbfm_mod_t* m=audio_mod_create(AUDIO_MOD_AM,&cfg,.8f);assert(m);
    float invalid=NAN;iq16_t out[1000];
    nbfm_result_t r=nbfm_process(m,&invalid,1,out,1000);assert(r.error&&!r.produced&&!r.consumed);
    r=nbfm_process(m,NULL,0,out,1000);assert(!r.error&&r.held_audio>0);
    for(unsigned k=0;k<1000;k++)assert(out[k].i==2000&&out[k].q==0);
    nbfm_destroy(m);
    const unsigned rates[]={4000000,2000000,1333333,1000000,800000,666667,500000,400000};
    for(unsigned k=0;k<8;k++)reference(rates[k],.8f,.5f,1000,1);
    reference(1000000,0,.5f,1000,0);reference(1000000,1,1,1000,1);
    double normal=reference(1000000,.8f,.5f,1000,0);
    assert(reference(1000000,.8f,.5f,100,0)/normal<.12);
    assert(reference(1000000,.8f,.5f,8000,0)/normal<.02);
    reference(1000000,1,100,1000,0);
    puts("PASS AM TX: native rates/packets, carrier/depth, bounded envelope, voice filtering, fractional clocks, queue contracts, reset and underflow");
}
