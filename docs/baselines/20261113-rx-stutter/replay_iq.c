#include "fm_demod_internal.h"
#include "noise_squelch.h"
#include "sinad.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
int main(int argc,char **argv)
{
    if(argc!=4){fprintf(stderr,"usage: replay_iq IQ_FILE RF_RATE OUT_PREFIX\n");return 2;}
    unsigned rate=(unsigned)strtoul(argv[2],NULL,10);
    nbfm_demod_config_t cfg={.rf_rate=rate,.audio_rate=48000,.deemph_tau=50e-6f,.pcm_gain=8000};
    nbfm_demod_t *dsp=nb_demod_create(&cfg);if(!dsp)return 2;
    FILE *in=fopen(argv[1],"rb");if(!in){perror(argv[1]);return 2;}
    char path[4096];snprintf(path,sizeof(path),"%s.raw.f32",argv[3]);FILE *rawfile=fopen(path,"wb");
    snprintf(path,sizeof(path),"%s.pcm.s16",argv[3]);FILE *pcmfile=fopen(path,"wb");
    snprintf(path,sizeof(path),"%s.metrics.jsonl",argv[3]);FILE *metrics=fopen(path,"w");
    if(!rawfile||!pcmfile||!metrics)return 2;
    iq16_t *iq=malloc((rate/100)*sizeof(*iq));float raw[1024];int16_t pcm[1024];
    sinad_t sinad;sinad_reset(&sinad);noise_squelch_t noise;noise_squelch_reset(&noise);
    size_t count;unsigned long long n=0;double power=0;float peak=0;
    while((count=fread(iq,sizeof(*iq),rate/100,in))){
        size_t offset=0;
        while(offset<count){
            nbfm_demod_result_t r=nb_demod_process_with_raw(dsp,iq+offset,count-offset,pcm,raw,1024,0);
            assert(!r.error&&r.consumed);offset+=r.consumed;
            fwrite(raw,sizeof(float),r.produced,rawfile);fwrite(pcm,sizeof(*pcm),r.produced,pcmfile);
            for(size_t j=0;j<r.produced;++j){
                sinad_process(&sinad,raw[j]);noise_squelch_process(&noise,raw[j]);power+=raw[j]*raw[j];peak=fmaxf(peak,fabsf(raw[j]));++n;
                if(n%4800==0){
                    fprintf(metrics,"{\"seconds\":%.3f,\"raw_rms\":%.6f,\"raw_peak\":%.6f,\"noise_rms\":%.6f,\"sinad_db\":",n/48000.0,sqrt(power/4800),peak,sqrtf(noise.power));
                    if(sinad.valid)fprintf(metrics,"%.6f",sinad.db);else fputs("null",metrics);
                    fputs("}\n",metrics);power=0;peak=0;
                }
            }
        }
    }
    fclose(in);fclose(rawfile);fclose(pcmfile);fclose(metrics);free(iq);free(dsp);
    fprintf(stderr,"Replayed %llu audio samples at fixed correction=0, no hardware or worker threads\n",n);
    return 0;
}
