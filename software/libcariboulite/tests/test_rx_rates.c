#include "audio_demod.h"
#include "radio_rates.h"
#include "rf_resampler.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define SAMPLES 8000
static int16_t one[SAMPLES],split[SAMPLES];static float raw_one[SAMPLES],raw_split[SAMPLES];
static void frontend(unsigned divider,unsigned target)
{
    rf_resampler_t s;assert(rf_resampler_init(&s,divider,target,target==20?80000:100000));
    double fs=4000000.0/divider,fo=4000000.0/target,cr=0,ci=0,power=0;size_t count=0,measured=0;
    for(size_t n=0;n<(size_t)(fs*.03);++n){double a=2*M_PI*40000*n/fs;float i,q;
        if(rf_resampler_push(&s,(float)cos(a),(float)sin(a),&i,&q)){
            double b=2*M_PI*40000*count/fo;if(count>fo*.01){cr+=i*cos(b)+q*sin(b);ci+=q*cos(b)-i*sin(b);power+=i*i+q*q;++measured;}++count;
        }
    }
    double residual=power/measured-(cr*cr+ci*ci)/(measured*measured);
    assert(residual<1e-7);printf("RF polyphase div=%u target=%u residual=%.3g\n",divider,target,residual);
}
static size_t process(audio_demod_t *d,iq16_t *iq,size_t n,int16_t *pcm,float *raw,bool fragmented)
{
    size_t pos=0,got=0;unsigned step=0;
    while(pos<n){size_t chunk=fragmented?1+(step++*191)%4096:n-pos;if(chunk>n-pos)chunk=n-pos;
        size_t cap=fragmented?1+step%31:SAMPLES-got;
        audio_demod_result_t r=audio_demod_process_with_raw(d,iq+pos,chunk,pcm+got,raw+got,cap,0);
        assert(!r.error&&r.consumed&&r.produced<=cap);pos+=r.consumed;got+=r.produced;assert(got<SAMPLES);
    }
    return got;
}
int main(void)
{
    for(unsigned div=1;div<=10;++div)if(radio_rate_divider(radio_rate_label(div))==div){if(20%div)frontend(div,20);if(16%div)frontend(div,16);}
    const unsigned rates[]={400000,500000,666667,800000,1000000,1333333,2000000,4000000};
    for(unsigned mode=0;mode<2;++mode)for(unsigned k=0;k<8;++k){unsigned rate=rates[k];double fs=radio_rate_exact(rate);size_t n=(size_t)llround(fs*.15);iq16_t *iq=malloc(n*sizeof(*iq));assert(iq);
        double deviation=mode?75000:1000;
        for(size_t j=0;j<n;++j){double t=j/fs,a=2*M_PI*3000*t+deviation/1000*sin(2*M_PI*1000*t);iq[j]=(iq16_t){(int16_t)lrint(12000*cos(a)),(int16_t)lrint(12000*sin(a))};}
        audio_demod_config_t cfg={.rf_rate=rate,.audio_rate=48000,.deemph_tau=50e-6f,.pcm_gain=8000};audio_demod_t *d=audio_demod_create((audio_demod_mode_t)mode,&cfg);assert(d);
        size_t got=process(d,iq,n,one,raw_one,false);assert(fabs(got-n*48000/fs)<=2);
        audio_demod_reset(d);size_t fragmented=process(d,iq,n,split,raw_split,true);assert(got==fragmented&&memcmp(one,split,got*2)==0&&memcmp(raw_one,raw_split,got*4)==0);
        double dc=0,a=0,b=0,power=0;size_t start=got-4800;
        for(size_t j=start;j<got;++j)dc+=raw_one[j];dc/=4800;
        for(size_t j=start;j<got;++j){double x=raw_one[j]-dc,t=2*M_PI*1000*j/48000;a+=x*cos(t);b+=x*sin(t);power+=x*x;}
        double amp=2*hypot(a,b)/4800,residual=power/4800-amp*amp/2,snr=10*log10((amp*amp/2)/fmax(residual,1e-20));
        assert(fabs(amp-(mode?1.0:.4))<.015&&snr>35);
        printf("%s %u label (%.6f actual): %zu samples, tone amplitude %.6f fit SNR %.2f dB, chunk/reset exact\n",mode?"WBFM":"NBFM",rate,fs,got,amp,snr);
        audio_demod_destroy(d);free(iq);
    }
    puts("PASS: all eight RX rates, rational timing/phase, NBFM/WBFM tones, block invariance and reset");
}
