#define _GNU_SOURCE
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "audio_mod.h"
#include "tx_pipeline.h"
int main(int argc,char**argv){assert(argc==4);float amp=atof(argv[1]),back=atof(argv[2]);nbfm_cfg_t cfg={48000,1000000,2500,0,4000*powf(10,-back/20),1};audio_mod_t*m=audio_mod_create(AUDIO_MOD_AM,&cfg,.8);assert(m);float audio[480];iq16_t iq[10000];rf10_frame_t*frame=calloc(1,sizeof(*frame));tx_writer_ctrl_st tx={.fm=m,.a48k=audio,.iq_rf=iq,.rf_rate_hz=1000000,.frame_samples=10000};FILE*f=fopen(argv[3],"wb");assert(f);for(int block=0;block<120;block++){for(int j=0;j<480;j++)audio[j]=amp*sin(2*M_PI*1000*(block*480+j)/48000.);assert(!tx_generate_frame(&tx,frame));if(block>=20)for(int j=0;j<10000;j++){assert(iq[j].q==0&&frame->data[j].q==0);int16_t pair[2]={iq[j].i,frame->data[j].i};assert(fwrite(pair,sizeof(pair),1,f)==1);}}fclose(f);free(frame);nbfm_destroy(m);}
