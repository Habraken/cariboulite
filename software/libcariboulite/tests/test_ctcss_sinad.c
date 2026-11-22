#include "audio_mod.h"
#include "audio_demod.h"
#include "sinad.h"
#include "ctcss.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

int main(int argc,char **argv)
{
    assert(argc==2);
    const float tones[]={88.5,254.1}, amplitudes[]={.4,.95}, deviations[]={0,250,500};
    puts("ctcss_hz,amplitude,tone_deviation_hz,raw_sinad_db,filtered_sinad_db,file");
    for(unsigned t=0;t<2;t++)for(unsigned a=0;a<2;a++)for(unsigned d=0;d<3;d++) {
        nbfm_cfg_t cfg={48000,1000000,2500,0,12000,1};
        audio_mod_t *tx=audio_mod_create(AUDIO_MOD_NBFM,&cfg,.8);assert(tx);
        assert(!audio_mod_set_ctcss(tx,deviations[d] ? tones[t]:0,deviations[d] ? deviations[d]:500));
        audio_demod_config_t dc={1000000,48000,50e-6,8000};
        audio_demod_t *rx=audio_demod_create(AUDIO_DEMOD_NBFM,&dc);assert(rx);
        sinad_t raw_meter,filtered_meter;sinad_reset(&raw_meter);sinad_reset(&filtered_meter);
        ctcss_voice_t hp={0};ctcss_notch_t notch;ctcss_notch_reset(&notch,tones[t]);
        char name[1024];snprintf(name,sizeof(name),"%s/t%u_a%u_d%u.f32",argv[1],t,a,d);
        FILE *out=fopen(name,"wb");assert(out);
        size_t audio_tick=0,received=0;
        float audio[480],raw[480];int16_t pcm[480];iq16_t iq[10000];
        for(unsigned frame=0;frame<300;frame++) {
            for(unsigned k=0;k<480;k++,audio_tick++)audio[k]=amplitudes[a]*sinf(6.283185307179586*1000*audio_tick/48000.);
            nbfm_result_t m=nbfm_process(tx,audio,480,iq,10000);
            assert(!m.error && m.consumed==480 && m.produced==10000 && !m.held_audio);
            size_t pos=0;
            while(pos<10000) {
                audio_demod_result_t r=audio_demod_process_with_raw(rx,iq+pos,10000-pos,pcm,raw,480,0);
                assert(!r.error && r.consumed);pos+=r.consumed;
                for(size_t k=0;k<r.produced;k++,received++) {
                    // Identical filtering in every case, independent of squelch.
                    float post=ctcss_voice_sample(&hp,ctcss_notch_sample(&notch,raw[k]));
                    sinad_process(&raw_meter,raw[k]);sinad_process(&filtered_meter,post);
                    if(received>=48000) {
                        float pair[]={raw[k],post};assert(fwrite(pair,sizeof(float),2,out)==2);
                    }
                }
            }
        }
        fclose(out);assert(raw_meter.valid && filtered_meter.valid);
        // At the normal 0.4 amplitude, residual CTCSS depresses the raw meter,
        // while the identically filtered measurement stays clean.
        if(a==0 || d==0) assert(filtered_meter.db>55);
        if(a==0 && d>0) assert(filtered_meter.db-raw_meter.db>20);
        // At 0.95 amplitude the voice limiter clips with either CTCSS budget.
        // This is real harmonic distortion, not the sub-tone bias above.
        if(a==1 && d>0) assert(filtered_meter.db<50 && filtered_meter.db>20);
        printf("%.1f,%.2f,%.0f,%.3f,%.3f,%s\n",tones[t],amplitudes[a],deviations[d],raw_meter.db,filtered_meter.db,name);
        nbfm_destroy(tx);audio_demod_destroy(rx);
    }
}
