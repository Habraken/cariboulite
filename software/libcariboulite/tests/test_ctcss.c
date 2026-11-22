#include "ctcss.h"
#include "audio_mod.h"
#include "audio_demod.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static const float tones[]={67,69.3,71.9,74.4,77,79.7,82.5,85.4,88.5,91.5,94.8,97.4,100,
103.5,107.2,110.9,114.8,118.8,123,127.3,131.8,136.5,141.3,146.2,151.4,156.7,159.8,
162.2,165.5,167.9,171.3,173.8,177.3,179.9,183.5,186.2,189.9,192.8,196.6,199.5,
203.5,206.5,210.7,218.1,225.7,229.1,233.6,241.8,250.3,254.1};
static void feed(ctcss_rx_t *s,float hz,float amplitude,unsigned n,bool voice)
{
    for(unsigned i=0;i<n;i++) {
        float x=amplitude*sinf(6.283185307179586*hz*i/48000.0+1.37);
        if(voice) x+=.6f*sinf(6.283185307179586*1000*i/48000.0)+.3f*sinf(6.283185307179586*1700*i/48000.0);
        ctcss_rx_sample(s,x+.15f); // discriminator DC offset
    }
}
static void detector(void)
{
    for(unsigned t=0;t<sizeof(tones)/sizeof(*tones);t++) {
        ctcss_rx_t s;
        ctcss_rx_reset(&s,tones[t]);
        feed(&s,tones[t],.2f,48000,true);
        if(!s.open) fprintf(stderr,"failed tone %.1f coherence=%f peak=%f\n",tones[t],s.coherence,s.peak);
        assert(s.valid && s.open);
        feed(&s,0,0,33600,false); assert(!s.open);
        // Nearest lower/upper tones must never open, even briefly.
        for(int delta=-1;delta<=1;delta+=2) {
            int j=(int)t+delta;
            if(j<0 || j>=(int)(sizeof(tones)/sizeof(*tones)))continue;
            ctcss_rx_reset(&s,tones[t]);
            for(unsigned i=0;i<48000;i++) {
                float x=.2f*sinf(6.283185307179586*tones[j]*i/48000.0);
                assert(!ctcss_rx_sample(&s,x));
            }
        }
        // +/-0.5 Hz source tolerance, independent of RF rate.
        for(int d=-1;d<=1;d+=2) {
            ctcss_rx_reset(&s,tones[t]);feed(&s,tones[t]+d*.5f,.2f,48000,true);
            assert(s.open);
        }
    }
    ctcss_rx_t s; ctcss_rx_reset(&s,100);
    unsigned seed=1;
    for(unsigned i=0;i<480000;i++) {
        seed=1664525u*seed+1013904223u;
        assert(!ctcss_rx_sample(&s,((int)(seed>>16)-32768)/32768.f));
    }
    feed(&s,100,.2f,48000,false);assert(s.open);
    uint32_t bits=0x7fc00000;float invalid;memcpy(&invalid,&bits,4);
    assert(!ctcss_rx_sample(&s,invalid));assert(!s.valid);
    ctcss_rx_reset(&s,0);assert(ctcss_rx_sample(&s,0));
    assert(!ctcss_frequency_valid(invalid));assert(!ctcss_frequency_valid(66.9f));
    assert(!ctcss_frequency_valid(254.2f));assert(!ctcss_deviation_valid(49));
}
static void playback_filter(void)
{
    for(unsigned t=0;t<sizeof(tones)/sizeof(*tones);t++) {
        for(int d=-1;d<=1;d++) {
            ctcss_voice_t hp={0};ctcss_notch_t notch;ctcss_notch_reset(&notch,tones[t]);
            double energy=0;
            for(unsigned i=0;i<96000;i++) {
                float x=sinf(6.283185307179586*(tones[t]+.5*d)*i/48000.);
                float y=ctcss_voice_sample(&hp,ctcss_notch_sample(&notch,x));
                if(i>=48000)energy+=y*y;
            }
            assert(sqrt(energy/24000)<.025); // >32 dB tone suppression including offset
        }
    }
}
static void modulator(void)
{
    const unsigned rates[]={400000,500000,666667,800000,1000000,1333333,2000000,4000000};
    for(unsigned r=0;r<8;r++) {
        nbfm_cfg_t cfg={48000,rates[r],2500,0,12000,1};
        audio_mod_t *m=audio_mod_create(AUDIO_MOD_NBFM,&cfg,.8f);assert(m);
        assert(!audio_mod_set_ctcss(m,100,500));
        unsigned divider=(unsigned)lround(4000000.0/rates[r]);
        size_t cap=40000/divider+1;
        iq16_t *iq=calloc(cap,sizeof(*iq));assert(iq);
        audio_demod_config_t dc={rates[r],48000,50e-6f,8000};
        audio_demod_t *demod=audio_demod_create(AUDIO_DEMOD_NBFM,&dc);assert(demod);
        ctcss_rx_t decoder;ctcss_rx_reset(&decoder,100);
        float raw[480];int16_t pcm[480];
        float audio[480]={0};double prev_i=0,prev_q=0,si=0,sq=0,energy=0,peak=0;
        unsigned rfphase=0;size_t tick=0,used=0;

        for(unsigned frame=0;frame<120;frame++) {
            unsigned phase=rfphase+40000;size_t n=phase/divider;rfphase=phase%divider;
            nbfm_result_t result=nbfm_process(m,audio,480,iq,n);
            assert(!result.error && result.consumed==480 && result.produced==n && !result.held_audio);
            size_t offset=0;
            while(offset<n) {
                audio_demod_result_t decoded=audio_demod_process_with_raw(demod,iq+offset,n-offset,pcm,raw,480,0);
                assert(!decoded.error && decoded.consumed);
                offset+=decoded.consumed;
                for(size_t k=0;k<decoded.produced;k++)ctcss_rx_sample(&decoder,raw[k]);
            }
            for(size_t k=0;k<n;k++,tick++) {
                double hz=atan2(prev_i*iq[k].q-prev_q*iq[k].i,prev_i*iq[k].i+prev_q*iq[k].q)*4000000/divider/6.283185307179586;
                prev_i=iq[k].i;prev_q=iq[k].q;
                if(frame>10) {
                    double w=6.283185307179586*100*tick*divider/4000000.;
                    si+=hz*sin(w);sq+=hz*cos(w);energy+=hz*hz;used++;
                    if(fabs(hz)>peak)peak=fabs(hz);
                }
            }
        }
        assert(decoder.valid && decoder.open);audio_demod_destroy(demod);
        double amp=2*hypot(si,sq)/used;
        assert(amp>495 && amp<505 && peak<500+2*4000000.0/divider/12000/6.283185307179586 && energy>0);
        // Full voice amplitude must still respect the total deviation budget.
        for(unsigned k=0;k<480;k++)audio[k]=sinf(6.283185307179586*1000*k/48000.);
        for(unsigned f=0;f<10;f++) {
            unsigned phase=rfphase+40000;size_t n=phase/divider;rfphase=phase%divider;
            nbfm_result_t result=nbfm_process(m,audio,480,iq,n);assert(!result.error);
            for(size_t k=1;k<n;k++) {
                double hz=atan2((double)iq[k-1].i*iq[k].q-(double)iq[k-1].q*iq[k].i,
                    (double)iq[k-1].i*iq[k].i+(double)iq[k-1].q*iq[k].q)*4000000/divider/6.283185307179586;
                assert(fabs(hz)<2500+2*4000000.0/divider/12000/6.283185307179586);
            }
        }
        // Reset repeats tone/voice state; final padding removes the sub-tone.
        audio_mod_reset_ctcss(m);audio_mod_mute_ctcss(m,true);memset(audio,0,sizeof(audio));
        for(unsigned f=0;f<3;f++)nbfm_process(m,audio,480,iq,40000/divider);
        for(size_t k=1;k<40000/divider;k++) assert(iq[k].i==iq[k-1].i && iq[k].q==iq[k-1].q);
        assert(audio_mod_set_ctcss(m,100,2500));
        free(iq);nbfm_destroy(m);
    }
    // Off is bit-identical and other modulation modes bypass saved CTCSS.
    for(int mode=AUDIO_MOD_NBFM;mode<=AUDIO_MOD_LSB;mode++) {
        nbfm_cfg_t c={48000,400000,2500,0,12000,1};
        audio_mod_t *a=audio_mod_create(mode,&c,.8f),*b=audio_mod_create(mode,&c,.8f);
        assert(a && b);assert(!audio_mod_set_ctcss(b,mode ? 100:0,500));
        float audio[480];for(unsigned i=0;i<480;i++)audio[i]=.2f*sinf(i*.1f);
        iq16_t x[4000],y[4000];
        nbfm_process(a,audio,480,x,4000);nbfm_process(b,audio,480,y,4000);
        assert(!memcmp(x,y,sizeof(x)));nbfm_destroy(a);nbfm_destroy(b);
    }
}
int main(void) {detector();playback_filter();modulator();puts("CTCSS: all 50 tones, adjacent rejection, tolerance, noise, release, reset, eight TX rates, deviation budget, tail and bypass passed");}
