#include "radio_rates.h"
#include "audio_mod.h"
#include "tx_pipeline.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void run(unsigned rate,float amplitude) {
 unsigned d=radio_rate_divider(rate);double fs=4000000./d;nbfm_cfg_t cfg={48000,rate,2500,0,4000,1};audio_mod_t*m=audio_mod_create(AUDIO_MOD_AM,&cfg,amplitude>1?1:.8),*split=audio_mod_create(AUDIO_MOD_AM,&cfg,amplitude>1?1:.8);assert(m&&split);assert(!audio_mod_set_am_dac_shaping(m,true));assert(!audio_mod_set_am_dac_shaping(split,true));
 float audio[480];iq16_t*iq=calloc(40000,sizeof(*iq)),*other=calloc(40000,sizeof(*other));rf10_frame_t*frame=calloc(1,sizeof(*frame));iq16_t*first=calloc(40000,sizeof(*first));assert(iq&&other&&frame&&first);tx_writer_ctrl_st tx={.fm=m,.a48k=audio,.iq_rf=iq,.rf_rate_hz=rate,.frame_samples=(40000+d-1)/d};size_t first_count=0;double total=0,ss=0,cc=0,count=0,sine=0,cosine=1,w=2*M_PI*1000/fs,sw=sin(w),cw=cos(w);
 for(unsigned block=0;block<70;block++) {
  for(unsigned j=0;j<480;j++)audio[j]=amplitude*sin(2*M_PI*1000*(block*480+j)/48000.);
  assert(!tx_generate_frame(&tx,frame));size_t n=frame->sample_count;
  for(unsigned j=0;j<480;j+=17){size_t k=480-j;if(k>17)k=17;nbfm_result_t r=nbfm_process(split,audio+j,k,NULL,0);assert(!r.error&&r.consumed==k);}
  for(size_t off=0;off<n;){size_t k=n-off;if(k>113)k=113;nbfm_result_t r=nbfm_process(split,NULL,0,other+off,k);assert(!r.error&&!r.held_audio&&r.produced==k);off+=k;}
  assert(!memcmp(iq,other,n*sizeof(*iq)));if(!block){first_count=n;memcpy(first,iq,n*sizeof(*iq));}
  for(size_t j=0;j<n;j++){assert(iq[j].q==0&&iq[j].i>=0&&iq[j].i<=4032&&!(iq[j].i%64));assert(frame->data[j].i==iq[j].i);if(block>=10){double y=iq[j].i/2000.-1;total+=y;ss+=y*sine;cc+=y*cosine;count++;}double ns=sine*cw+cosine*sw;cosine=cosine*cw-sine*sw;sine=ns;}
 }
 double modulation=2*hypot(ss,cc)/count;
 if(amplitude<=1 && fabs(total/count)>=.00002)fprintf(stderr,"DC error rate=%u amplitude=%g mean=%.9f\n",rate,amplitude,total/count);
 if(amplitude<=1){assert(fabs(total/count)<.00002);assert(fabs(modulation-amplitude*.7956)<.00005+amplitude*.004);}
 nbfm_reset(m);tx.rf_frame_phase=0;tx.next_sequence=0;for(unsigned j=0;j<480;j++)audio[j]=amplitude*sin(2*M_PI*1000*j/48000.);assert(!tx_generate_frame(&tx,frame));assert(frame->sample_count==first_count&&!memcmp(iq,first,first_count*sizeof(*iq)));
 printf("PASS rate=%u amplitude=%g modulation=%.6f mean=%.7f; grid/bounds, chunking and reset exact\n",rate,amplitude,modulation,total/count);free(iq);free(other);free(frame);free(first);nbfm_destroy(m);nbfm_destroy(split);
}
static void disabled_is_raw(void) {
 nbfm_cfg_t c={48000,1000000,2500,0,4000,1};
 audio_mod_t*a=audio_mod_create(AUDIO_MOD_AM,&c,.8),*b=audio_mod_create(AUDIO_MOD_AM,&c,.8);
 assert(a&&b&&!audio_mod_set_am_dac_shaping(b,true)&&!audio_mod_set_am_dac_shaping(b,false));
 float audio[480];iq16_t x[10000],y[10000];
 for(unsigned j=0;j<480;j++)audio[j]=.01f*sin(2*M_PI*1000*j/48000.);
 nbfm_result_t u=nbfm_process(a,audio,480,x,10000),v=nbfm_process(b,audio,480,y,10000);
 assert(!u.error&&!v.error&&!u.held_audio&&!v.held_audio&&u.produced==10000&&v.produced==10000&&!memcmp(x,y,sizeof(x)));
 nbfm_destroy(a);nbfm_destroy(b);
}

int main(void){
 setvbuf(stdout,NULL,_IONBF,0);
 disabled_is_raw();
 assert(audio_mod_set_am_dac_shaping(NULL,true)==-1);
 nbfm_cfg_t invalid_scale={48000,1000000,2500,0,4096,1};audio_mod_t* invalid=audio_mod_create(AUDIO_MOD_AM,&invalid_scale,.8);assert(invalid && audio_mod_set_am_dac_shaping(invalid,true)==-1);nbfm_destroy(invalid);
 nbfm_cfg_t normal={48000,1000000,2500,0,4000,1};invalid=audio_mod_create(AUDIO_MOD_NBFM,&normal,.8);assert(invalid && audio_mod_set_am_dac_shaping(invalid,true)==-1);nbfm_destroy(invalid);
unsigned rates[]={4000000,2000000,1333333,1000000,800000,666667,500000,400000};for(unsigned r=0;r<8;r++)for(unsigned a=0;a<6;a++)run(rates[r],(float[]){0,.001,.01,.1,1,100}[a]);}
