#include "tx_pipeline.h"
#include "radio_rates.h"
#include "math_compat.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static void check_rate(unsigned label)
{
    unsigned d=radio_rate_divider(label);double fs=4000000.0/d;
    nbfm_cfg_t cfg={48000,label,2500,0,12000,1};
    nbfm_mod_t *fm=nbfm_create(&cfg),*split=nbfm_create(&cfg);assert(fm&&split);
    float a[480];iq16_t *iq=calloc(40000,sizeof(*iq)),*other=calloc(40000,sizeof(*other));
    rf10_frame_t *frame=calloc(1,sizeof(*frame));assert(iq&&other&&frame);
    tx_writer_ctrl_st tx={.fm=fm,.a48k=a,.iq_rf=iq,.rf_rate_hz=label,.frame_samples=(40000+d-1)/d};
    uint64_t total=0, checked=0;double square_error=0; iq16_t prev={0};
    for(unsigned block=0;block<300;++block){
        for(unsigned j=0;j<480;++j)a[j]=(float)(.4*sin(2*M_PI*1000*(block*480+j)/48000.0));
        assert(!tx_generate_frame(&tx,frame));
        size_t n=frame->sample_count;
        assert(n==((uint64_t)(block+1)*40000/d-(uint64_t)block*40000/d));
        assert(frame->tx_sequence==block+1 && n<=tx.frame_samples);
        // Independent split input/output calls must produce exactly the same IQ.
        for(unsigned j=0;j<480;j+=17){size_t k=480-j;if(k>17)k=17;
            nbfm_result_t r=nbfm_process(split,a+j,k,NULL,0);assert(!r.error&&r.consumed==k);
        }
        size_t off=0;
        while(off<n){size_t k=n-off;if(k>113)k=113;
            nbfm_result_t r=nbfm_process(split,NULL,0,other+off,k);
            assert(!r.error&&r.produced==k&&!r.held_audio);off+=k;
        }
        assert(!memcmp(iq,other,n*sizeof(*iq)));
        assert(nbfm_buffered_audio(fm)<=1 && nbfm_buffered_audio(split)<=1);
        for(size_t j=0;j<n;++j){
            assert(frame->data[j].i==(int16_t)(iq[j].i|1)&&frame->data[j].q==iq[j].q);
            if(total>fs*.002){
                double angle=atan2((double)iq[j].q*prev.i-(double)iq[j].i*prev.q,
                                   (double)iq[j].i*prev.i+(double)iq[j].q*prev.q);
                double ticks=(total+1)*48000.0/fs;double tick=floor(ticks),frac=ticks-tick;
                double x0=.4*sin(2*M_PI*1000*(tick-2)/48000.0);
                double x1=.4*sin(2*M_PI*1000*(tick-1)/48000.0);
                double expected=2500*(x0+(x1-x0)*frac);
                double error=angle*fs/(2*M_PI)-expected;square_error+=error*error;++checked;
            }
            prev=iq[j];++total;
        }
    }
    assert(total==UINT64_C(12000000)/d && tx.rf_frame_phase==0);
    double rms=sqrt(square_error/checked);assert(rms<fs/50000); // int16 phase quantization
    nbfm_reset(fm);nbfm_reset(split);tx.rf_frame_phase=0;
    assert(!tx_generate_frame(&tx,frame));
    nbfm_result_t r=nbfm_process(split,a,480,other,frame->sample_count);
    assert(!r.error&&!r.held_audio&&!memcmp(iq,other,frame->sample_count*sizeof(*iq)));
    printf("PASS TX %u (%.6f actual): 300 fractional frames, %llu pairs, deviation residual %.4f Hz, split/reset exact\n",label,fs,(unsigned long long)total,rms);
    nbfm_destroy(fm);nbfm_destroy(split);free(iq);free(other);free(frame);
}
int main(void)
{
    const unsigned rates[]={4000000,2000000,1333333,1000000,800000,666667,500000,400000};
    for(unsigned k=0;k<8;++k)check_rate(rates[k]);
    puts("PASS: all native TX clocks, fractional frame counts, continuous audio, IQ markers and modulation");
}
