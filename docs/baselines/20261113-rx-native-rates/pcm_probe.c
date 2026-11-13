#define _GNU_SOURCE
#include <alsa/asoundlib.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
static FILE *logfile,*pcmfile;static unsigned channels;
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
int snd_pcm_hw_params_set_buffer_size_near(snd_pcm_t *p,snd_pcm_hw_params_t *h,snd_pcm_uframes_t *n){
 int(*fn)(snd_pcm_t*,snd_pcm_hw_params_t*,snd_pcm_uframes_t*)=dlsym(RTLD_NEXT,"snd_pcm_hw_params_set_buffer_size_near");
 const char *e=getenv("PROBE_BUFFER_FRAMES");if(e)*n=strtoul(e,0,10);return fn(p,h,n);
}
snd_pcm_sframes_t snd_pcm_writei(snd_pcm_t *p,const void *b,snd_pcm_uframes_t n){
 snd_pcm_sframes_t(*fn)(snd_pcm_t*,const void*,snd_pcm_uframes_t)=dlsym(RTLD_NEXT,"snd_pcm_writei");
 double start=now();snd_pcm_sframes_t r=fn(p,b,n);
 if(!logfile){const char *path=getenv("PROBE_LOG");if(path)logfile=fopen(path,"w");}
 if(r>0){
   if(!pcmfile){const char *path=getenv("PROBE_PCM");if(path){pcmfile=fopen(path,"wb");snd_pcm_hw_params_t *h;snd_pcm_hw_params_malloc(&h);snd_pcm_hw_params_current(p,h);snd_pcm_hw_params_get_channels(h,&channels);snd_pcm_hw_params_free(h);}}
   if(pcmfile)fwrite(b,channels*sizeof(short),(size_t)r,pcmfile);
 }
 if(logfile)fprintf(logfile,"%.9f %.9f %lu %ld\n",start,now(),n,r);
 return r;
}
