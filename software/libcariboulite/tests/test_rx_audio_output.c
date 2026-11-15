#include "rx_audio_output.h"
#include "radio_rates.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static void controls(void)
{
    assert(rx_audio_volume_valid(-120) && rx_audio_volume_valid(-60) && rx_audio_volume_valid(12));
    assert(!rx_audio_volume_valid(NAN) && !rx_audio_volume_valid(INFINITY));
    assert(!rx_audio_volume_valid(-61) && !rx_audio_volume_valid(12.1f));
    rx_audio_output_t s={0}; uint64_t levels=0;
    rx_audio_set_gain(&s,10000);
    int16_t previous=0;
    for(unsigned k=0;k<960;k++) {
        int16_t y=rx_audio_output_sample(&s,1,1,&levels);
        assert(y>=previous && y-previous<=12);previous=y;
    }
    assert(previous==10000 && s.gain==10000);
    rx_audio_set_gain(&s,0);
    for(unsigned k=0;k<960;k++) {
        int16_t y=rx_audio_output_sample(&s,1,1,&levels);
        assert(y<=previous && previous-y<=12);previous=y;
    }
    assert(previous==0);
    for(unsigned k=0;k<9600;k++) assert(!rx_audio_output_sample(&s,100,1,&levels));
    assert(levels==(UINT64_C(1)<<63)); // valid silence at the -120 dBFS floor
    rx_audio_set_gain(&s,100000);
    for(unsigned k=0;k<960;k++)rx_audio_output_sample(&s,1,0,&levels);
    assert(s.clipped==0); // closed gate must not clip or accumulate noise
    assert(rx_audio_output_sample(&s,1,1,&levels)==32767 && s.clipped==1);
    assert(rx_audio_output_sample(&s,-1,1,&levels)==-32768 && s.clipped==2);
    puts("PASS volume bounds, 20 ms ramps, mute, gate, silence meter and clipping counts");
}
static void reference(audio_demod_mode_t mode,unsigned rate,float tau,float deviation)
{
    audio_demod_config_t config={rate,48000,tau,1000000};
    audio_demod_t* dsp=audio_demod_create(mode,&config);assert(dsp);
    if(mode==AUDIO_DEMOD_NBFM)assert(!audio_demod_set_nbfm_deviation(dsp,deviation));
    double fs=radio_rate_exact(rate),energy=0,peak=0,window_power=0,last_rms=0,window_peak=0,last_peak=0;
    const unsigned total=(unsigned)llround(fs*1.2);
    rx_audio_output_t stage={0};uint64_t levels=0;unsigned outputs=0,measured=0,legacy_clips=0;
    rx_audio_set_gain(&stage,rx_audio_calibration(mode,tau));
    iq16_t iq[4096];int16_t pcm[480];float filtered[480],raw[480];
    for(unsigned base=0;base<total;) {
        unsigned n=total-base;if(n>4096)n=4096;
        for(unsigned j=0;j<n;j++) {
            double t=(base+j)/fs,a=2048,phase;
            if(mode==AUDIO_DEMOD_AM){a*=1+.5*sin(2*M_PI*1000*t);phase=2*M_PI*500*t;}
            else phase=.5*(mode==AUDIO_DEMOD_NBFM?deviation:75000)/1000*sin(2*M_PI*1000*t);
            iq[j]=(iq16_t){(int16_t)lrint(a*cos(phase)),(int16_t)lrint(a*sin(phase))};
        }
        unsigned off=0;
        while(off<n) {
            audio_demod_result_t r=audio_demod_process_with_taps(dsp,iq+off,n-off,pcm,raw,filtered,480,0);
            assert(!r.error && r.consumed);off+=r.consumed;
            for(unsigned j=0;j<r.produced;j++) {
                int16_t y=rx_audio_output_sample(&stage,filtered[j],1,&levels);
                window_power+=(y/32768.)*(y/32768.); window_peak=fmax(window_peak,fabs(y/32768.));
                if ((outputs+1)%4800==0) {last_rms=10*log10(fmax(window_power/4800,1e-12));last_peak=20*log10(fmax(window_peak,1e-6));window_power=window_peak=0;}
                if(outputs++>=48000){double v=y/32768.;energy+=v*v;peak=fmax(peak,fabs(v));measured++;}
                if(pcm[j]==32767 || pcm[j]==-32768)legacy_clips++;
            }
        }
        base+=n;
    }
    double rms=10*log10(energy/measured);
    assert(fabs(rms+12)<.1 && stage.clipped==0 && legacy_clips>100);
    assert(levels>>63);
    double meter=((int)(levels&0x1ffff)-120000)/1000.;assert(fabs(meter-last_rms)<.002);
    double metered_peak=((int)((levels>>17)&0x1ffff)-120000)/1000.;
    assert(fabs(metered_peak-last_peak)<.002);
    printf("PASS %s %u tau %.0f us deviation %.0f: RMS %.3f dBFS, peak %.3f dBFS; pre-PCM tap avoids legacy clipping\n",audio_demod_mode_name(mode),rate,tau*1e6,deviation,rms,20*log10(peak));
    audio_demod_destroy(dsp);
}
int main(void)
{
    controls();
    const unsigned rates[]={4000000,2000000,1333333,1000000,800000,666667,500000,400000};
    for(unsigned m=0;m<3;m++) {
        for(unsigned r=0;r<8;r++)reference(m,rates[r],50e-6f,2500);
        reference(m,1000000,75e-6f,5000);
        reference(m,1000000,0,2500);
    }
    puts("PASS calibrated output across all modes/native rates, de-emphasis settings and NBFM deviation");
}
