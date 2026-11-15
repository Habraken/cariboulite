/* Narrow voice AM: amplitude-preserving RF filtering, envelope detection,
 * carrier-referenced slow AGC, 300 Hz HP / 3 kHz LP, then 48 kHz PCM.
 * No device access or allocations in process/reset/setters. */
#include "fm_demod_internal.h"
#include "nbfm_channel_filter.h"
#include "math_compat.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AM_CHANNEL_TAPS 193
#define AM_BOOTSTRAP_SAMPLES 1000 /* 20 ms, including channel-filter settling */
typedef struct { double b0,b1,b2,a1,a2,z1,z2; } am_biquad_t;
typedef struct {
    nbfm_demod_t base;
    nbfm_channel_filter_t frontend;
    float coeff[AM_CHANNEL_TAPS],i[2*AM_CHANNEL_TAPS],q[2*AM_CHANNEL_TAPS];
    unsigned pos,bootstrap,mix_phase,mix_step,rf_divider;
    float mix_cos[320],mix_sin[320];
    bool agc_disabled;
    float carrier,carrier_alpha,gain;
    am_biquad_t hp,lp[2];
    float previous,current;
    double phase;
} am_state_t;

static bool finite_float(float x)
{
    uint32_t bits;memcpy(&bits,&x,sizeof(bits));
    return (bits&0x7f800000u)!=0x7f800000u;
}
static bool finite_double(double x)
{
    uint64_t bits;memcpy(&bits,&x,sizeof(bits));
    return (bits&UINT64_C(0x7ff0000000000000))!=UINT64_C(0x7ff0000000000000);
}
static void biquad_init(am_biquad_t* b,double hz,double q,bool high)
{
    double w=2*M_PI*hz/48000,c=cos(w),a=sin(w)/(2*q),d=1+a;
    double sign=high?1:-1;
    *b=(am_biquad_t){.b0=(1+sign*c)/(2*d),.b1=-sign*(1+sign*c)/d,
        .b2=(1+sign*c)/(2*d),.a1=-2*c/d,.a2=(1-a)/d};
}
static float biquad_run(am_biquad_t* b,float x)
{
    double y=b->b0*x+b->z1;
    b->z1=b->b1*x-b->a1*y+b->z2;b->z2=b->b2*x-b->a2*y;
    return (float)y;
}
int am_demod_set_audio(nbfm_demod_t* dsp,float tau,float gain)
{
    if(!dsp||dsp->mode!=AUDIO_DEMOD_AM||!finite_float(tau)||tau<0||
       !finite_float(gain)||gain<0)return -EINVAL;
    // FM de-emphasis is deliberately bypassed for AM.
    ((am_state_t*)dsp)->gain=gain;return 0;
}
int audio_demod_set_am_agc(audio_demod_t* dsp,bool enabled)
{
    if(!dsp||dsp->mode!=AUDIO_DEMOD_AM)return -EINVAL;
    ((am_state_t*)dsp)->agc_disabled=!enabled;return 0;
}
int audio_demod_set_am_offset(audio_demod_t* dsp,int hz)
{
    if(!dsp || dsp->mode!=AUDIO_DEMOD_AM || (hz!=0 && hz!=12500 && hz!=-12500))return -EINVAL;
    am_state_t* s=(am_state_t*)dsp;
    s->mix_step=hz==0 ? 0 : hz>0 ? s->rf_divider : 320-s->rf_divider;
    s->mix_phase=0;
    return 0;
}
void am_demod_reset(nbfm_demod_t* dsp)
{
    am_state_t* s=(am_state_t*)dsp;if(!s)return;
    nbfm_channel_filter_reset(&s->frontend);
    memset(s->i,0,sizeof(s->i));memset(s->q,0,sizeof(s->q));
    s->mix_phase=0;
    s->pos=0;s->carrier=0;s->bootstrap=AM_BOOTSTRAP_SAMPLES;
    s->previous=s->current=0;s->phase=0;
    s->hp.z1=s->hp.z2=0;
    for(unsigned k=0;k<2;++k)s->lp[k].z1=s->lp[k].z2=0;
}
nbfm_demod_t* am_demod_create(const nbfm_demod_config_t* c)
{
    if(!c||!radio_rate_divider(c->rf_rate)||c->audio_rate!=48000||
       !finite_float(c->deemph_tau)||c->deemph_tau<0||
       !finite_float(c->pcm_gain)||c->pcm_gain<0){errno=EINVAL;return NULL;}
    am_state_t* s=calloc(1,sizeof(*s));if(!s)return NULL;
    s->base.mode=AUDIO_DEMOD_AM;s->gain=c->pcm_gain;
    s->rf_divider=radio_rate_divider(c->rf_rate);
    for(unsigned k=0;k<320;k++){
        s->mix_cos[k]=(float)cos(2*M_PI*k/320);
        s->mix_sin[k]=(float)sin(2*M_PI*k/320);
    }
    nbfm_channel_filter_init(&s->frontend,c->rf_rate);
    double sum=0;
    for(unsigned k=0;k<AM_CHANNEL_TAPS;++k){
        int t=(int)k-AM_CHANNEL_TAPS/2;
        double v=t?sin(2*M_PI*3900*t/50000)/(M_PI*t):2.*3900/50000;
        v*=.42-.5*cos(2*M_PI*k/(AM_CHANNEL_TAPS-1))+.08*cos(4*M_PI*k/(AM_CHANNEL_TAPS-1));
        s->coeff[k]=(float)v;sum+=v;
    }
    for(unsigned k=0;k<AM_CHANNEL_TAPS;++k)s->coeff[k]/=(float)sum;
    s->carrier_alpha=-expm1f(-1.f/(50000*.25f));
    biquad_init(&s->hp,300,.7071067811865476,true);
    biquad_init(&s->lp[0],3000,.541196100146197,false);
    biquad_init(&s->lp[1],3000,1.306562964876377,false);
    am_demod_reset(&s->base);return &s->base;
}
audio_demod_result_t am_demod_process_with_raw(nbfm_demod_t* dsp,
    const iq16_t* input,size_t count,int16_t* output,float* raw,size_t capacity,double correction)
{
    am_state_t* s=(am_state_t*)dsp;audio_demod_result_t result={0};
    if(!s||(!input&&count)||(!output&&capacity)||!finite_double(correction)||fabs(correction)>.0005){result.error=-EINVAL;return result;}
    for(size_t k=0;k<count&&result.produced<capacity;++k){
        ++result.consumed;float i,q;
        iq16_t sample=input[k];
        if(s->mix_step) {
            unsigned phase=s->mix_phase;
            float cs=s->mix_cos[phase],sn=s->mix_sin[phase];
            float ri=sample.i*cs-sample.q*sn,rq=sample.i*sn+sample.q*cs;
            sample.i=(int16_t)lrintf(fmaxf(-32768,fminf(32767,ri)));
            sample.q=(int16_t)lrintf(fmaxf(-32768,fminf(32767,rq)));
            phase+=s->mix_step;s->mix_phase=phase>=320 ? phase-320 : phase;
        }
        if(!nbfm_channel_filter_push(&s->frontend,sample,&i,&q))continue;
        unsigned p=s->pos;
        s->i[p]=s->i[p+AM_CHANNEL_TAPS]=i;s->q[p]=s->q[p+AM_CHANNEL_TAPS]=q;
        s->pos=p+1==AM_CHANNEL_TAPS?0:p+1;
        const float* hi=s->i+s->pos,*hq=s->q+s->pos;
        float fi=0,fq=0;
        for(unsigned j=0;j<AM_CHANNEL_TAPS;++j){fi+=s->coeff[j]*hi[j];fq+=s->coeff[j]*hq[j];}
        float envelope=sqrtf(fi*fi+fq*fq);
        if(s->bootstrap){s->carrier=fmaxf(s->carrier,envelope);--s->bootstrap;}
        else {
            // A new strong transmission must not spend a quarter second at
            // the old weak-signal gain. Normal <=100% AM stays below this
            // threshold, so this guard does not track individual voice peaks.
            if(envelope>2.5f*fmaxf(s->carrier,128.f))s->carrier=envelope;
            else s->carrier+=s->carrier_alpha*(envelope-s->carrier);
        }
        // Cap automatic gain at 32x relative to 13-bit IQ full scale. Manual
        // mode uses a fixed IQ reference; --rx-gain remains the volume control.
        float reference=s->agc_disabled?4096.f:fmaxf(s->carrier,128.f);
        s->previous=s->current;s->current=envelope/reference;
        double r=(48000./50000)*(1+correction);s->phase+=r;
        if(s->phase>=1){
            float v=s->current+(float)((s->phase-1)/r)*(s->previous-s->current);
            v=biquad_run(&s->hp,v);
            v=biquad_run(&s->lp[1],biquad_run(&s->lp[0],v));
            if(raw)raw[result.produced]=v;
            if(s->base.filtered_audio)s->base.filtered_audio[result.produced]=v;
            float pcm=fmaxf(-32768.f,fminf(32767.f,v*s->gain));
            output[result.produced++]=(int16_t)lrintf(pcm);s->phase-=1;
        }
    }
    return result;
}
