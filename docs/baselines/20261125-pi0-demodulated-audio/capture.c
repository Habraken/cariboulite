#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "audio_demod.h"
#include <alsa/asoundlib.h>
#include <dlfcn.h>
#include <limits.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/* Diagnostic-only link wrappers. Forward every call unchanged, prefault RAM,
 * arm once with SIGUSR1, and save only after the backend joins its workers. */
static int (*real_iq)(void *,void *,void *,size_t);
static snd_pcm_sframes_t (*real_pcm)(snd_pcm_t *,const void *,snd_pcm_uframes_t);
static atomic_bool armed;
static int16_t *iq_buffer,*pcm_buffer;
static float *raw_buffer,*filtered_buffer;
typedef struct {uint64_t first_sample,callback_ns;double correction;uint32_t frames;} chunk_t;
static chunk_t *chunks;
static size_t iq_count,pcm_count,raw_count,filtered_count,chunk_count;
static size_t iq_capacity,audio_capacity;
static unsigned channels;
static uint64_t iq_start_ns,pcm_start_ns,raw_start_ns;
static char prefix[PATH_MAX-64];
static uint64_t now_ns(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000ULL+t.tv_nsec;}
static void arm(int sig){(void)sig;atomic_store_explicit(&armed,true,memory_order_relaxed);}
static void *buffer(size_t n){void*p=mmap(0,n,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);if(p==MAP_FAILED)_exit(98);for(size_t k=0;k<n;k+=4096)((volatile char*)p)[k]=0;return p;}
__attribute__((constructor)) static void init(void)
{
    _Static_assert(ATOMIC_BOOL_LOCK_FREE==2,"Signal handler requires lock-free atomic");
    *(void**)(&real_iq)=dlsym(RTLD_NEXT,"cariboulite_radio_read_samples");
    *(void**)(&real_pcm)=dlsym(RTLD_NEXT,"snd_pcm_writei");
    const char*p=getenv("CL_CAPTURE_PREFIX"),*n=getenv("CL_CAPTURE_IQ_SAMPLES"),*a=getenv("CL_CAPTURE_AUDIO_FRAMES");
    if(!p)return; /* TX diagnostics need no recording buffers. */
    if(!real_iq||!real_pcm||snprintf(prefix,sizeof(prefix),"%s",p)>=(int)sizeof(prefix)-24)_exit(98);
    iq_capacity=n?strtoull(n,0,10):8000000;audio_capacity=a?strtoull(a,0,10):384000;
    if(!iq_capacity||iq_capacity>16000000||!audio_capacity||audio_capacity>960000)_exit(98);
    iq_buffer=buffer(iq_capacity*4);pcm_buffer=buffer(audio_capacity*2*2);
    raw_buffer=buffer(audio_capacity*4);filtered_buffer=buffer(audio_capacity*4);
    chunks=buffer(audio_capacity*sizeof(*chunks));
    atomic_init(&armed,false);
    struct sigaction sa={0};sa.sa_handler=arm;sigemptyset(&sa.sa_mask);sa.sa_flags=SA_RESTART;
    if(sigaction(SIGUSR1,&sa,0))_exit(98);
    fprintf(stderr,"capture ready: IQ=%zu audio=%zu; IQ/raw/filtered/PCM forwarded unchanged\n",iq_capacity,audio_capacity);
}
int cariboulite_radio_read_samples(void*r,void*b,void*m,size_t n)
{
    if(!real_iq)*(void**)(&real_iq)=dlsym(RTLD_NEXT,"cariboulite_radio_read_samples");
    int got=real_iq(r,b,m,n);
    if(got>0&&atomic_load_explicit(&armed,memory_order_relaxed)&&iq_count<iq_capacity){
        if(!iq_count)iq_start_ns=now_ns();
        size_t take=(size_t)got;if(take>iq_capacity-iq_count)take=iq_capacity-iq_count;
        memcpy(iq_buffer+2*iq_count,b,4*take);iq_count+=take;
    }
    return got;
}
audio_demod_result_t __real_audio_demod_process_with_taps(audio_demod_t*,const iq16_t*,size_t,audio_s16_t*,float*,float*,size_t,double);
audio_demod_result_t __wrap_audio_demod_process_with_taps(audio_demod_t*d,const iq16_t*input,size_t n,audio_s16_t*out,float*raw,float*filtered,size_t cap,double correction)
{
    audio_demod_result_t result=__real_audio_demod_process_with_taps(d,input,n,out,raw,filtered,cap,correction);
    if(result.produced&&atomic_load_explicit(&armed,memory_order_relaxed)){
        if(raw&&raw_count<audio_capacity){
            size_t take=result.produced;if(take>audio_capacity-raw_count)take=audio_capacity-raw_count;
            uint64_t ns=now_ns();if(!raw_count)raw_start_ns=ns;
            chunks[chunk_count++]=(chunk_t){raw_count,ns,correction,(uint32_t)take};
            memcpy(raw_buffer+raw_count,raw,4*take);raw_count+=take;
        }
        if(filtered&&filtered_count<audio_capacity){
            size_t take=result.produced;if(take>audio_capacity-filtered_count)take=audio_capacity-filtered_count;
            memcpy(filtered_buffer+filtered_count,filtered,4*take);filtered_count+=take;
        }
    }
    return result;
}
snd_pcm_sframes_t snd_pcm_writei(snd_pcm_t*p,const void*b,snd_pcm_uframes_t n)
{
    if(!real_pcm)*(void**)(&real_pcm)=dlsym(RTLD_NEXT,"snd_pcm_writei");
    snd_pcm_sframes_t got=real_pcm(p,b,n);
    if(prefix[0]&&!channels){snd_pcm_hw_params_t*params;snd_pcm_hw_params_alloca(&params);
        if(snd_pcm_hw_params_current(p,params)||snd_pcm_hw_params_get_channels(params,&channels)||!channels||channels>2)_exit(98);}
    if(got>0&&atomic_load_explicit(&armed,memory_order_relaxed)&&pcm_count<audio_capacity){
        if(!pcm_count)pcm_start_ns=now_ns();
        size_t take=(size_t)got;if(take>audio_capacity-pcm_count)take=audio_capacity-pcm_count;
        memcpy(pcm_buffer+channels*pcm_count,b,2*channels*take);pcm_count+=take;
    }
    return got;
}
static void save(const char*s,const void*p,size_t n){char path[PATH_MAX];snprintf(path,sizeof(path),"%s%s",prefix,s);FILE*f=fopen(path,"wb");if(!f){perror(path);return;}if(fwrite(p,1,n,f)!=n)perror("write capture");fclose(f);}
__attribute__((destructor)) static void finish(void)
{
    if(!prefix[0]||!atomic_load(&armed))return;
    save(".iq",iq_buffer,iq_count*4);save(".pcm",pcm_buffer,pcm_count*channels*2);
    save(".raw.f32",raw_buffer,raw_count*4);save(".filtered.f32",filtered_buffer,filtered_count*4);
    char path[PATH_MAX];snprintf(path,sizeof(path),"%s.clock.jsonl",prefix);FILE*f=fopen(path,"w");
    if(f){for(size_t k=0;k<chunk_count;k++)fprintf(f,"{\"first_sample\":%llu,\"callback_ns\":%llu,\"correction\":%.17g,\"frames\":%u}\n",(unsigned long long)chunks[k].first_sample,(unsigned long long)chunks[k].callback_ns,chunks[k].correction,chunks[k].frames);fclose(f);}
    char json[1500];int n=snprintf(json,sizeof(json),"{\"iq_samples\":%zu,\"raw_frames\":%zu,\"filtered_frames\":%zu,\"pcm_frames\":%zu,\"channels\":%u,\"clock_chunks\":%zu,\"iq_first_callback_ns\":%llu,\"raw_first_callback_ns\":%llu,\"pcm_first_callback_ns\":%llu,\"iq_layout\":\"ci16_le\",\"raw_layout\":\"f32_le_mono_pre_volume_pre_deemphasis\",\"filtered_layout\":\"f32_le_mono_post_filter_pre_gain\",\"pcm_layout\":\"s16_le_interleaved_accepted_ALSA_frames\",\"audio_rate_hz\":48000}\n",iq_count,raw_count,filtered_count,pcm_count,channels,chunk_count,(unsigned long long)iq_start_ns,(unsigned long long)raw_start_ns,(unsigned long long)pcm_start_ns);
    if(n>0&&(size_t)n<sizeof(json))save(".json",json,(size_t)n);
}
