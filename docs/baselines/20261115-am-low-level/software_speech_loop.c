#include <stdio.h>
#include <assert.h>
#include "audio_mod.h"
#include "audio_demod.h"
int main(void){
 nbfm_cfg_t c={48000,1000000,2500,0,2004.7489f,1};
 nbfm_mod_t*m=audio_mod_create(AUDIO_MOD_AM,&c,.8);
 audio_demod_config_t d={1000000,48000,0,16384};
 audio_demod_t*r=audio_demod_create(AUDIO_DEMOD_AM,&d);
 assert(m&&r);float a[480];iq16_t iq[10000];int16_t out[512];size_t n;
 while((n=fread(a,sizeof(float),480,stdin))==480){
  nbfm_result_t v=nbfm_process(m,a,n,iq,10000);assert(!v.error&&v.consumed==n&&v.produced==10000&&v.held_audio==0);
  audio_demod_result_t z=audio_demod_process(r,iq,10000,out,512,0);assert(!z.error&&z.consumed==10000);
  fwrite(out,sizeof(int16_t),z.produced,stdout);
 }
 audio_demod_destroy(r);nbfm_destroy(m);return 0;
}
