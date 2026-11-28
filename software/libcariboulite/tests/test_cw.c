#include "cw_keyer.h"
#include "audio_demod.h"
#include "audio_mod.h"
#include "math_compat.h"
#include "radio_rates.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void ticks(cw_keyer_t* k,unsigned n,bool expected)
{for(unsigned i=0;i<n;++i)assert(cw_keyer_tick(k)==expected);}
static void keyer(void)
{
    cw_keyer_t k={0};assert(cw_keyer_configure(&k,CW_IAMBIC,12,false));
    cw_keyer_contacts(&k,3);ticks(&k,4800,true);ticks(&k,4800,false);
    ticks(&k,14400,true); /* dah after dit */
    cw_keyer_contacts(&k,0);ticks(&k,4800,false);ticks(&k,10000,false); /* no trailing element */
    assert(cw_keyer_configure(&k,CW_IAMBIC,12,false));
    cw_keyer_contacts(&k,1);ticks(&k,100,true);
    cw_keyer_contacts(&k,3);ticks(&k,100,true);cw_keyer_contacts(&k,1);
    ticks(&k,4600,true);ticks(&k,4800,false);ticks(&k,4800,true); /* no opposite-tap memory */
    cw_keyer_release(&k);assert(cw_keyer_configure(&k,CW_ULTIMATIC,12,false));
    cw_keyer_contacts(&k,1);ticks(&k,4800,true);ticks(&k,4800,false);
    cw_keyer_contacts(&k,3);ticks(&k,14400,true);ticks(&k,4800,false);ticks(&k,14400,true);
    cw_keyer_contacts(&k,1);ticks(&k,4800,false);ticks(&k,4800,true);
    cw_keyer_release(&k);assert(cw_keyer_configure(&k,CW_ULTIMATIC,12,false));
    cw_keyer_contacts(&k,3);ticks(&k,4800,true);ticks(&k,4800,false); /* simultaneous -> dit */
    for(int type=CW_STRAIGHT;type<=CW_COOTIE;type+=3){
        cw_keyer_release(&k);assert(cw_keyer_configure(&k,type,12,true));
        cw_keyer_contacts(&k,1);ticks(&k,10000,true);cw_keyer_contacts(&k,3);ticks(&k,10000,true);
        cw_keyer_contacts(&k,2);ticks(&k,10000,true);cw_keyer_contacts(&k,0);ticks(&k,10000,false);
    }
    for(unsigned wpm=5;wpm<=30;++wpm){
        cw_keyer_release(&k);assert(cw_keyer_configure(&k,CW_IAMBIC,wpm,false));
        cw_keyer_contacts(&k,1);bool previous=false;unsigned edges=0;
        uint64_t position=0;
        while(edges<1001){bool mark=cw_keyer_tick(&k);if(mark&&!previous){
            assert(fabs((double)position-(double)edges*115200/wpm)<1.01);++edges;
        }previous=mark;++position;}
        assert(!cw_keyer_configure(&k,CW_IAMBIC,12,false));
    }
    cw_envelope_t env={0};float previous=0;
    for(unsigned i=0;i<240;++i){float v=cw_envelope_tick(&env,true,5);assert(v>=previous&&v<=1);previous=v;}
    assert(previous>.9999);for(unsigned i=0;i<240;++i){float v=cw_envelope_tick(&env,false,5);assert(v<=previous&&v>=0);previous=v;}assert(previous<1e-6);
    puts("CW keyer: timing 5–30 PARIS WPM, iambic/ultimatic, direct contacts and envelopes PASS");
}
static double receive(unsigned rate,unsigned bw,bool inverted,int shift,double detune)
{
    audio_demod_config_t cfg={rate,48000,0,8000};audio_demod_t* d=audio_demod_create(AUDIO_DEMOD_CW,&cfg);assert(d);
    assert(!audio_demod_set_cw(d,bw,600,shift,inverted));
    assert(audio_demod_set_cw(d,123,600,shift,inverted)<0);
    double fs=radio_rate_exact(rate),power=0;unsigned measured=0;uint64_t total=0;
    for(unsigned block=0;block<(unsigned)(fs*.16/257)+1;++block){
        iq16_t iq[257];int16_t pcm[17];
        for(unsigned i=0;i<257;++i){double phase=2*M_PI*(-shift+detune)*(total+i)/fs;iq[i]=(iq16_t){lrint(2048*cos(phase)),lrint(2048*sin(phase))};}total+=257;
        size_t offset=0;
        while(offset<257){audio_demod_result_t r=audio_demod_process(d,iq+offset,257-offset,pcm,17,0);assert(!r.error&&(r.consumed||r.produced));offset+=r.consumed;
            for(size_t i=0;i<r.produced;++i){if(block>fs*.06/257){power+=(double)pcm[i]*pcm[i];++measured;}}}
    }
    audio_demod_reset(d);assert(audio_demod_process(d,NULL,0,NULL,0,0).consumed==0);
    audio_demod_destroy(d);assert(measured>1000);return sqrt(power/measured);
}
static void dsp(void)
{
    const unsigned rates[]={4000000,2000000,1333333,1000000,800000,666667,500000,400000};
    for(unsigned j=0;j<8;++j){
        unsigned rate=rates[j];
        for(unsigned bw=250;bw<=1000;bw*=2){
            double wanted=receive(rate,bw,j%2,j%2?-12500:12500,0);
            double unwanted=receive(rate,bw,j%2,j%2?-12500:12500,2000);
            assert(wanted>2700&&wanted<2900);assert(unwanted<wanted*.01);
        }
        nbfm_cfg_t cfg={.audio_fs=48000,.rf_fs=rate,.out_scale=4000,.linear_interp=1};
        audio_mod_t* m=audio_mod_create(AUDIO_MOD_CW,&cfg,0);assert(m);
        float gate[480];iq16_t iq[40000];unsigned divider=radio_rate_divider(rate),phase=0;
        for(unsigned frame=0;frame<3;++frame){
            for(unsigned i=0;i<480;++i)gate[i]=frame==1?1:0;
            unsigned count=(phase+40000)/divider;phase=(phase+40000)%divider;
            nbfm_result_t r=nbfm_process(m,gate,480,iq,count);assert(!r.error&&r.produced==count&&r.consumed==480&&!r.held_audio);
            for(unsigned i=200;i<count;++i){assert(iq[i].q==0);assert(iq[i].i==(frame==1?4000:0));}
        }
        nbfm_destroy(m);
    }
    puts("CW DSP: all native rates, carrier power/space, 250/500/1000 Hz filters, offset and inversion PASS");
}
int main(void){keyer();dsp();return 0;}
