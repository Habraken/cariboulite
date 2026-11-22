#include "cdcss.h"
#include "audio_mod.h"
#include "audio_demod.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned random_state=12345;
static float noise(void){random_state=random_state*1664525u+1013904223u;return ((int)(random_state>>16)-32768)/32768.f;}
static void words(void)
{
    // Published normal-polarity vectors; these pin wire order, not just a
    // round-trip between our own encoder and decoder (references in docs).
    assert(cdcss_codeword(023)==0x763813);assert(cdcss_codeword(025)==0x6b7815);
    assert(cdcss_codeword(047)==0x0fd827);
    cdcss_config_t c;char name[5];
    assert(cdcss_parse("023",&c)&&c.enabled&&!c.inverted&&c.code==023);
    assert(cdcss_parse("023i",&c)&&c.inverted);cdcss_format(c,name);assert(!strcmp(name,"023I"));
    assert(cdcss_parse("000N",&c)&&c.enabled&&c.code==0);assert(cdcss_parse("off",&c)&&!c.enabled);
    const char*bad[]={"23","0023","D023","089","023B","023N ","-23","nan","on",""};
    for(unsigned k=0;k<sizeof(bad)/sizeof(*bad);k++)assert(!cdcss_parse(bad[k],&c));
    for(unsigned code=0;code<512;code++) {
        uint32_t w=cdcss_codeword(code);assert((w&0xfff)==(0x800|code));
        uint32_t r=cdcss_reverse(w,23);
        for(int k=22;k>=11;k--)if(r&(1u<<k))r^=0xae3u<<(k-11);
        assert(r==0);
    }
    cdcss_rx_t rx;cdcss_rx_reset(&rx,(cdcss_config_t){true,false,023});
    for(unsigned a=0;a<23;a++)for(unsigned b=a+1;b<23;b++)for(unsigned c=b+1;c<23;c++)
        assert(cdcss_match(&rx,0x763813^(1u<<a)^(1u<<b)^(1u<<c))<=3);
    assert(cdcss_match(&rx,cdcss_codeword(025))>=7);
    assert(cdcss_match(&rx,cdcss_codeword(023)^CDCSS_WORD_MASK)>=7);
    // Inverted aliases are inherent to CDCSS: 023N is the same stream as 047I.
    assert(cdcss_match(&rx,cdcss_codeword(047)^CDCSS_WORD_MASK)==0);
    cdcss_tx_t tx;cdcss_tx_reset(&tx,(cdcss_config_t){true,false,023});
    for(unsigned k=0;k<240000;k++)assert(fabsf(cdcss_tx_sample(&tx))<=1);
    assert(tx.phase==0&&tx.bit==5); // exactly 672 bits in five seconds
}
static void waveform(void)
{
    const unsigned codes[]={000,023,025,047,071,125,245,331,431,523,627,754,777};
    for(unsigned k=0;k<sizeof(codes)/sizeof(*codes);k++)for(unsigned inv=0;inv<2;inv++) {
        cdcss_config_t c={true,inv,codes[k]};cdcss_tx_t tx;cdcss_rx_t rx;cdcss_tx_reset(&tx,c);cdcss_rx_reset(&rx,c);
        unsigned acquired=0;
        for(unsigned n=0;n<48000;n++) {
            float x=.1f*cdcss_tx_sample(&tx)+.2f+.4f*sinf(n*.1308996939f)+.002f*noise();
            bool open=cdcss_rx_sample(&rx,x);
            if(open&&!acquired)acquired=n;
            if(n>40000)assert(open);
        }
        if(!rx.open)fprintf(stderr,"waveform code %03o %u failed\n",codes[k],inv);
        assert(rx.valid&&rx.open&&acquired<40000);
        cdcss_tx_tail(&tx,true);
        for(unsigned n=0;n<10000;n++)cdcss_rx_sample(&rx,.1f*cdcss_tx_sample(&tx)+.2f);
        assert(!rx.open&&rx.turnoff);
        cdcss_tx_reset(&tx,c);cdcss_rx_reset(&rx,c);
        for(unsigned n=0;n<48000;n++)cdcss_rx_sample(&rx,.1f*cdcss_tx_sample(&tx));
        assert(rx.open);
        for(unsigned n=0;n<24000;n++)cdcss_rx_sample(&rx,0);
        assert(!rx.open);
    }
    // Independently clocked and shaped reference bitstream: golden 023N word.
    for(int skew=-2;skew<=2;skew++)for(unsigned inv=0;inv<2;inv++) {
        cdcss_rx_t rx;cdcss_rx_reset(&rx,(cdcss_config_t){true,inv,023});
        double lp1=0,lp2=0,rate=134.4*(1+skew*.005),start=.37;
        for(unsigned n=0;n<72000;n++) {
            unsigned bit=(unsigned)((n/48000.0+start)*rate)%23;
            double x=((0x763813>>bit)&1 ? 1:-1)*(inv?-1:1);
            lp1+=.03*(x-lp1);lp2+=.03*(lp1-lp2);
            cdcss_rx_sample(&rx,.1f*lp2+.2f+.3f*sinf(n*.1308996939f));
        }
        assert(rx.open);
    }
    // A source between clock-bank rates must hand over sampling phase without
    // audible gate holes once acquired, rather than only passing exact rates.
    for(int sign=-1;sign<=1;sign+=2) {
        cdcss_rx_t tracked;cdcss_rx_reset(&tracked,(cdcss_config_t){true,false,023});
        double lp1=0,lp2=0,rate=134.4*(1+sign*.0025);
        for(unsigned n=0;n<48000*12;n++) {
            unsigned bit=(unsigned)((n/48000.0+.37)*rate)%23;
            double x=(0x763813>>bit)&1 ? 1:-1;
            lp1+=.03*(x-lp1);lp2+=.03*(lp1-lp2);
            bool open=cdcss_rx_sample(&tracked,.1f*lp2+.2f);
            if(n>96000)assert(open);
        }
    }
    cdcss_rx_t rx;cdcss_rx_reset(&rx,(cdcss_config_t){true,false,023});
    cdcss_tx_t tx;cdcss_tx_reset(&tx,(cdcss_config_t){true,false,025});
    for(unsigned n=0;n<96000;n++)assert(!cdcss_rx_sample(&rx,.1f*cdcss_tx_sample(&tx)));
    cdcss_tx_reset(&tx,(cdcss_config_t){true,true,023});cdcss_rx_reset(&rx,(cdcss_config_t){true,false,023});
    for(unsigned n=0;n<96000;n++)assert(!cdcss_rx_sample(&rx,.1f*cdcss_tx_sample(&tx)));
    cdcss_rx_reset(&rx,(cdcss_config_t){true,false,023});
    for(unsigned n=0;n<96000;n++)assert(!cdcss_rx_sample(&rx,.2f*sinf(n*6.283185307179586*88.5/48000)));
    for(unsigned n=0;n<1440000;n++)assert(!cdcss_rx_sample(&rx,noise()));
    uint32_t bad=0x7fc00000;float nan;memcpy(&nan,&bad,4);assert(!cdcss_rx_sample(&rx,nan)&&!rx.valid);
}
static void playback_filter(void)
{
    cdcss_tx_t tx;cdcss_tx_reset(&tx,(cdcss_config_t){true,false,023});ctcss_voice_t hp={0};
    double before=0,after=0;
    for(unsigned k=0;k<96000;k++){
        float x=cdcss_tx_sample(&tx),y=ctcss_voice_sample(&hp,x);
        if(k>=48000){before+=x*x;after+=y*y;}
    }
    assert(sqrt(after/before)<.04); // at least 28 dB sub-audio RMS rejection
}
static void modem(void)
{
    const unsigned rates[]={400000,500000,666667,800000,1000000,1333333,2000000,4000000};
    for(unsigned r=0;r<8;r++) {
        unsigned divider=(unsigned)lround(4000000.0/rates[r]),rf_phase=0;
        nbfm_cfg_t cfg={48000,rates[r],2500,0,12000,1};
        audio_mod_t*tx=audio_mod_create(AUDIO_MOD_NBFM,&cfg,.8);assert(tx);
        cdcss_config_t code={true,false,023};assert(!audio_mod_set_cdcss(tx,code,250));
        assert(audio_mod_set_ctcss(tx,88.5,200)); // explicit exclusion
        audio_demod_config_t dc={rates[r],48000,50e-6f,8000};
        audio_demod_t*rx=audio_demod_create(AUDIO_DEMOD_NBFM,&dc);assert(rx);
        cdcss_rx_t detector;cdcss_rx_reset(&detector,code);
        iq16_t*iq=calloc(40000/divider+1,sizeof(*iq));assert(iq);
        float audio[480],raw[480];int16_t pcm[480];unsigned tick=0;
        for(unsigned frame=0;frame<150;frame++) {
            if(frame==120)audio_mod_cdcss_tail(tx,true);
            for(unsigned k=0;k<480;k++,tick++)audio[k]=frame<120?(frame<90?.4f:1.f)*sinf(6.283185307179586*1000*tick/48000.):0;
            unsigned phase=rf_phase+40000;size_t n=phase/divider;rf_phase=phase%divider;
            nbfm_result_t mod=nbfm_process(tx,audio,480,iq,n);
            assert(!mod.error&&mod.consumed==480&&mod.produced==n&&!mod.held_audio);
            for(size_t k=1;k<n;k++) {
                double hz=atan2((double)iq[k-1].i*iq[k].q-(double)iq[k-1].q*iq[k].i,
                    (double)iq[k-1].i*iq[k].i+(double)iq[k-1].q*iq[k].q)*4000000/divider/6.283185307179586;
                assert(fabs(hz)<2500+2*4000000.0/divider/12000/6.283185307179586);
            }
            size_t offset=0;
            while(offset<n) {
                audio_demod_result_t d=audio_demod_process_with_raw(rx,iq+offset,n-offset,pcm,raw,480,0);
                assert(!d.error&&d.consumed);offset+=d.consumed;
                for(size_t k=0;k<d.produced;k++)cdcss_rx_sample(&detector,raw[k]);
            }
            if(frame==119)assert(detector.valid&&detector.open);
        }
        assert(!detector.open&&detector.turnoff);free(iq);nbfm_destroy(tx);audio_demod_destroy(rx);
    }
    // Disabled CDCSS and saved CDCSS in non-NBFM modes preserve exact old samples.
    for(int mode=AUDIO_MOD_NBFM;mode<=AUDIO_MOD_LSB;mode++) {
        nbfm_cfg_t c={48000,400000,2500,0,12000,1};
        audio_mod_t*a=audio_mod_create(mode,&c,.8),*b=audio_mod_create(mode,&c,.8);assert(a&&b);
        assert(!audio_mod_set_cdcss(b,(cdcss_config_t){mode!=AUDIO_MOD_NBFM,false,023},250));
        float audio[480];for(unsigned k=0;k<480;k++)audio[k]=.2f*sinf(k*.1f);
        iq16_t x[4000],y[4000];nbfm_process(a,audio,480,x,4000);nbfm_process(b,audio,480,y,4000);
        assert(!memcmp(x,y,sizeof(x)));nbfm_destroy(a);nbfm_destroy(b);
    }
}
int main(void){words();waveform();playback_filter();modem();puts("CDCSS: wire vectors, all 512 codewords, polarity/alias, 3-bit correction, clock offsets, wrong code, noise, turnoff, eight RF rates, budget and bypass passed");}
