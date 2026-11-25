#include "audio_demod.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Offline production DSP replay. Optional CFO removal rotates decoded IQ
 * before the unchanged channel filter. No hardware, clocks or playback. */
int main(int argc,char **argv)
{
    assert(argc==4||argc==5);unsigned rate=(unsigned)strtoul(argv[3],0,10);
    double cfo=argc==5?strtod(argv[4],0):0;
    double step=6.283185307179586*cfo/rate,cs=cos(step),sn=sin(step),c=1,s=0;
    unsigned rotation_count=0;
    FILE *in=fopen(argv[1],"rb"),*out=fopen(argv[2],"wb");assert(in&&out);
    audio_demod_config_t cfg={rate,48000,50e-6f,8000};
    audio_demod_t*d=audio_demod_create(AUDIO_DEMOD_NBFM,&cfg);assert(d);
    iq16_t iq[10000];float raw[480],filtered[480];int16_t pcm[480];size_t n;
    while((n=fread(iq,sizeof(*iq),10000,in))){
        if(cfo!=0)for(size_t k=0;k<n;k++){
            double i=iq[k].i,q=iq[k].q;
            double ri=i*c+q*s,rq=q*c-i*s;
            assert(fabs(ri)<32767&&fabs(rq)<32767);
            iq[k].i=(int16_t)lround(ri);iq[k].q=(int16_t)lround(rq);
            double nc=c*cs-s*sn;s=s*cs+c*sn;c=nc;
            if(++rotation_count==8192){double scale=1/hypot(c,s);c*=scale;s*=scale;rotation_count=0;}
        }
        size_t offset=0;
        while(offset<n){
            audio_demod_result_t r=audio_demod_process_with_taps(d,iq+offset,n-offset,pcm,raw,filtered,480,0);
            assert(!r.error&&r.consumed);offset+=r.consumed;
            for(size_t k=0;k<r.produced;k++){float pair[]={raw[k],filtered[k]};assert(fwrite(pair,4,2,out)==2);}
        }
    }
    assert(!ferror(in));fclose(in);assert(!fclose(out));audio_demod_destroy(d);
}
