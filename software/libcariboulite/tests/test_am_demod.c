#include "audio_demod.h"
#include "radio_rates.h"
#include "math_compat.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static size_t decode(audio_demod_t* d,const iq16_t* iq,size_t n,int16_t* pcm,float* raw,bool chunks)
{
    size_t in=0,out=0,turn=0;
    while(in<n){
        size_t count=chunks?1+(turn*7919)%30011:n-in;if(count>n-in)count=n-in;
        size_t cap=chunks?1+(turn*37)%701:1024;
        audio_demod_result_t r=audio_demod_process_with_raw(d,iq+in,count,pcm+out,raw+out,cap,0);
        assert(!r.error&&r.consumed&&r.consumed<=count&&r.produced<=cap);
        in+=r.consumed;out+=r.produced;++turn;
    }
    return out;
}
static void signal(iq16_t* iq,size_t n,double fs,double carrier,double depth,double hz,double offset,bool adjacent)
{
    for(size_t k=0;k<n;++k){
        double t=k/fs,a=carrier*(1+depth*sin(2*M_PI*hz*t)),p=2*M_PI*offset*t+.37;
        double i=a*cos(p),q=a*sin(p);
        if(adjacent){double b=2048*(1+.5*sin(2*M_PI*2300*t)),c=2*M_PI*8333.333333*t+.8;i+=b*cos(c);q+=b*sin(c);}
        iq[k]=(iq16_t){(int16_t)lrint(i),(int16_t)lrint(q)};
    }
}
static double amplitude(const float* raw,size_t a,size_t b,double hz,double* snr)
{
    double ss=0,cc=0,p=0,mean=0;
    for(size_t k=a;k<b;++k){double t=k/48000.;ss+=raw[k]*sin(2*M_PI*hz*t);cc+=raw[k]*cos(2*M_PI*hz*t);p+=(double)raw[k]*raw[k];mean+=raw[k];}
    double n=b-a,amp=2*hypot(ss,cc)/n;
    if(snr)*snr=10*log10((amp*amp/2)/fmax(p/n-mean*mean/(n*n)-amp*amp/2,1e-15));
    return amp;
}
static void rate_case(unsigned rate)
{
    double fs=radio_rate_exact(rate);size_t n=(size_t)(fs*2);
    iq16_t* iq=malloc(n*sizeof(*iq));int16_t* pcm=malloc(97000*sizeof(*pcm)),*splitpcm=malloc(97000*sizeof(*pcm));
    float* raw=malloc(97000*sizeof(*raw)),*splitraw=malloc(97000*sizeof(*raw));assert(iq&&pcm&&splitpcm&&raw&&splitraw);
    audio_demod_config_t c={rate,48000,75e-6f,8000};
    audio_demod_t *d=audio_demod_create(AUDIO_DEMOD_AM,&c),*split=audio_demod_create(AUDIO_DEMOD_AM,&c);assert(d&&split);
    assert(audio_demod_capabilities(d)==0);
    signal(iq,n,fs,512,.6,1000,750,false);
    size_t count=decode(d,iq,n,pcm,raw,false),other=decode(split,iq,n,splitpcm,splitraw,true);
    assert(count>=95998&&count<=96000&&count==other);
    assert(!memcmp(pcm,splitpcm,count*sizeof(*pcm))&&!memcmp(raw,splitraw,count*sizeof(*raw)));
    double snr,amp=amplitude(raw,72000,91200,1000,&snr);
    assert(amp>.585&&amp<.61&&snr>40);
    // De-emphasis is ignored in AM; reset retains AGC selection and exactly
    // reproduces startup/filter/resampler state.
    audio_demod_reset(d);audio_demod_process(d,iq,13,pcm,32,0);audio_demod_reset(d);
    assert(!audio_demod_set_audio(d,0,8000));
    other=decode(d,iq,n,pcm,raw,true);
    assert(other==count&&!memcmp(raw,splitraw,count*sizeof(*raw))&&!memcmp(pcm,splitpcm,count*sizeof(*pcm)));
    // Manual amplitude scaling and independent PCM gain.
    audio_demod_reset(d);assert(!audio_demod_set_am_agc(d,false));
    other=decode(d,iq,n,pcm,raw,false);double manual=amplitude(raw,72000,91200,1000,NULL);
    assert(manual>.073&&manual<.077);
    assert(!audio_demod_set_am_agc(d,true));audio_demod_reset(d);
    signal(iq,n,fs,2048,.6,1000,-750,false);decode(d,iq,n,pcm,raw,false);
    double strong=amplitude(raw,72000,91200,1000,NULL);assert(fabs(strong-amp)<.008);
    // Unmodulated carrier must not generate FM audio even with frequency offset.
    audio_demod_reset(d);signal(iq,n,fs,2048,0,1000,1000,false);decode(d,iq,n,pcm,raw,false);
    double energy=0;for(size_t k=72000;k<91200;++k)energy+=(double)raw[k]*raw[k];assert(sqrt(energy/19200)<.001);
    printf("PASS AM %u (%.6f actual): %zu audio samples, depth %.6f, SNR %.2f dB, manual %.6f, chunk/reset exact\n",rate,fs,count,amp,snr,manual);
    audio_demod_destroy(d);audio_demod_destroy(split);free(iq);free(pcm);free(splitpcm);free(raw);free(splitraw);
}
static void filters_and_errors(void)
{
    const unsigned rate=400000;size_t n=rate*2;
    iq16_t* iq=malloc(n*sizeof(*iq));int16_t* pcm=malloc(97000*sizeof(*pcm));float* raw=malloc(97000*sizeof(*raw));assert(iq&&pcm&&raw);
    audio_demod_config_t c={rate,48000,0,8000};audio_demod_t* d=audio_demod_create(AUDIO_DEMOD_AM,&c);assert(d);
    double amps[3];const double hz[]={100,1000,8000};
    for(unsigned j=0;j<3;++j){audio_demod_reset(d);signal(iq,n,rate,512,.6,hz[j],0,false);decode(d,iq,n,pcm,raw,false);amps[j]=amplitude(raw,72000,91200,hz[j],NULL);}
    assert(amps[0]/amps[1]<.13&&amps[2]/amps[1]<.03);
    audio_demod_reset(d);signal(iq,n,rate,512,.6,1000,0,true);decode(d,iq,n,pcm,raw,false);
    double wanted=amplitude(raw,72000,91200,1000,NULL),unwanted=amplitude(raw,72000,91200,2300,NULL);
    assert(fabs(wanted-amps[1])<.008&&unwanted/wanted<.0032);
    audio_demod_reset(d);signal(iq,n,rate,2048,.8,1000,0,false);
    memset(iq,0,(rate/2)*sizeof(*iq));size_t burst=decode(d,iq,n,pcm,raw,false);
    for(size_t k=0;k<burst;++k)assert(pcm[k]!=32767 && pcm[k]!=-32768);
    audio_demod_result_t r=audio_demod_process(d,iq,10,pcm,0,0);assert(!r.error&&!r.consumed&&!r.produced);
    assert(audio_demod_process(d,NULL,1,pcm,1,0).error==-EINVAL);
    assert(audio_demod_process(d,iq,1,NULL,1,0).error==-EINVAL);
    assert(audio_demod_process(d,iq,1,pcm,1,NAN).error==-EINVAL);
    assert(audio_demod_process(d,iq,1,pcm,1,INFINITY).error==-EINVAL);
    assert(audio_demod_process(d,iq,1,pcm,1,.00051).error==-EINVAL);
    assert(audio_demod_set_audio(d,NAN,8000)==-EINVAL&&audio_demod_set_audio(d,0,INFINITY)==-EINVAL);
    for(int direction=-1;direction<=1;direction+=2){
        audio_demod_reset(d);size_t in=0,out=0;double correction=direction*.0005;
        while(in<n){audio_demod_result_t q=audio_demod_process(d,iq+in,n-in,pcm+out,512,correction);assert(!q.error&&q.consumed);in+=q.consumed;out+=q.produced;}
        assert(fabs((double)out-96000*(1+correction))<2);
    }
    c.rf_rate=700000;assert(!audio_demod_create(AUDIO_DEMOD_AM,&c));
    c.rf_rate=rate;c.pcm_gain=NAN;assert(!audio_demod_create(AUDIO_DEMOD_AM,&c));
    assert(audio_demod_set_am_agc(NULL,true)==-EINVAL);
    printf("PASS AM filtering: 100 Hz %.2f dB, 8 kHz %.2f dB, adjacent 2.3 kHz leakage %.2f dBc; invalid input/capacity checks\n",20*log10(amps[0]/amps[1]),20*log10(amps[2]/amps[1]),20*log10(unwanted/wanted));
    audio_demod_destroy(d);free(iq);free(pcm);free(raw);
}
int main(void)
{
    const unsigned rates[]={4000000,2000000,1333333,1000000,800000,666667,500000,400000};
    for(unsigned k=0;k<8;++k)rate_case(rates[k]);
    filters_and_errors();puts("PASS: narrow voice AM envelope, AGC/manual gain, all native clocks, channel/audio filtering and contracts");
}
