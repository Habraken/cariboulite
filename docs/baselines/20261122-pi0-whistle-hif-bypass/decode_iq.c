#include "audio_demod.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
/* Replay captured IQ through the production NBFM DSP without live clocks or
 * playback. Save interleaved raw-discriminator / filtered-audio float32. */
int main(int argc,char **argv)
{
    assert(argc==4);unsigned rate=strtoul(argv[3],0,10);
    FILE *in=fopen(argv[1],"rb"),*out=fopen(argv[2],"wb");assert(in&&out);
    audio_demod_config_t cfg={rate,48000,50e-6f,8000};
    audio_demod_t*d=audio_demod_create(AUDIO_DEMOD_NBFM,&cfg);assert(d);
    iq16_t iq[10000];float raw[480],filtered[480];int16_t pcm[480];size_t n;
    while((n=fread(iq,sizeof(*iq),10000,in))) {
        size_t offset=0;
        while(offset<n) {
            audio_demod_result_t r=audio_demod_process_with_taps(d,iq+offset,n-offset,pcm,raw,filtered,480,0);
            assert(!r.error&&r.consumed);offset+=r.consumed;
            for(size_t k=0;k<r.produced;k++){float pair[]={raw[k],filtered[k]};assert(fwrite(pair,4,2,out)==2);}
        }
    }
    assert(!ferror(in));fclose(in);fclose(out);audio_demod_destroy(d);
}
