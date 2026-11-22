#define _GNU_SOURCE
#include <alsa/asoundlib.h>
#include <dlfcn.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <limits.h>
/* Bounded diagnostic capture. One RX reader and one ALSA writer own their
 * respective buffers. No disk IO during acquisition; save after worker join.
 * SIGUSR1 arms both taps. RF samples and playback are forwarded unchanged. */
static int (*real_iq)(void*,void*,void*,size_t);
static snd_pcm_sframes_t (*real_pcm)(snd_pcm_t*,const void*,snd_pcm_uframes_t);
static atomic_bool armed;
static int16_t *iq_buffer,*pcm_buffer;
static size_t iq_count,pcm_count,iq_capacity,pcm_capacity;
static unsigned channels;
static uint64_t iq_start_ns,pcm_start_ns;
static char prefix[PATH_MAX];
static uint64_t now_ns(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000+t.tv_nsec;}
static void arm(int sig){(void)sig;atomic_store_explicit(&armed,true,memory_order_relaxed);}
static void *buffer(size_t n){void*p=mmap(0,n,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);if(p==MAP_FAILED)_exit(98);for(size_t k=0;k<n;k+=4096)((volatile char*)p)[k]=0;return p;}
__attribute__((constructor)) static void init(void)
{
    _Static_assert(ATOMIC_BOOL_LOCK_FREE==2,"Signal handler requires lock-free atomic");
    *(void**)(&real_iq)=dlsym(RTLD_NEXT,"cariboulite_radio_read_samples");
    *(void**)(&real_pcm)=dlsym(RTLD_NEXT,"snd_pcm_writei");
    const char*p=getenv("WHISTLE_CAPTURE_PREFIX"),*n=getenv("WHISTLE_CAPTURE_IQ_SAMPLES");
    if(!real_iq||!real_pcm||!p||snprintf(prefix,sizeof(prefix),"%s",p)>=(int)sizeof(prefix)-20)_exit(98);
    iq_capacity=n?strtoull(n,0,10):4000000;pcm_capacity=48000*6;
    if(!iq_capacity||iq_capacity>16000000)_exit(98);
    iq_buffer=buffer(iq_capacity*4);pcm_buffer=buffer(pcm_capacity*2*2);
    atomic_init(&armed,false);
    struct sigaction a={0};a.sa_handler=arm;sigemptyset(&a.sa_mask);a.sa_flags=SA_RESTART;
    if(sigaction(SIGUSR1,&a,0))_exit(98);
    fprintf(stderr,"whistle capture ready: IQ capacity %zu, PCM frames %zu\n",iq_capacity,pcm_capacity);
}
int cariboulite_radio_read_samples(void*r,void*b,void*m,size_t n)
{
    int got=real_iq(r,b,m,n);
    if(got>0&&atomic_load_explicit(&armed,memory_order_relaxed)&&iq_count<iq_capacity){
        if(!iq_count)iq_start_ns=now_ns();size_t take=(size_t)got;
        if(take>iq_capacity-iq_count)take=iq_capacity-iq_count;
        memcpy(iq_buffer+2*iq_count,b,4*take);iq_count+=take;
    }
    return got;
}
snd_pcm_sframes_t snd_pcm_writei(snd_pcm_t*p,const void*b,snd_pcm_uframes_t n)
{
    snd_pcm_sframes_t got=real_pcm(p,b,n);
    if(!channels){snd_pcm_hw_params_t*params;snd_pcm_hw_params_alloca(&params);
        if(snd_pcm_hw_params_current(p,params)||snd_pcm_hw_params_get_channels(params,&channels)||channels>2)_exit(98);}
    if(got>0&&atomic_load_explicit(&armed,memory_order_relaxed)&&pcm_count<pcm_capacity){
        if(!pcm_count)pcm_start_ns=now_ns();size_t take=(size_t)got;
        if(take>pcm_capacity-pcm_count)take=pcm_capacity-pcm_count;
        memcpy(pcm_buffer+channels*pcm_count,b,2*channels*take);pcm_count+=take;
    }
    return got;
}
static void save(const char*s,const void*p,size_t n){char path[PATH_MAX];snprintf(path,sizeof(path),"%s%s",prefix,s);FILE*f=fopen(path,"wb");if(!f){perror(path);return;}if(fwrite(p,1,n,f)!=n)perror("write capture");fclose(f);}
__attribute__((destructor)) static void finish(void)
{
    if(!atomic_load(&armed))return;
    save(".iq",iq_buffer,iq_count*4);save(".pcm",pcm_buffer,pcm_count*channels*2);
    char json[1024];int n=snprintf(json,sizeof(json),"{\"iq_samples\":%zu,\"pcm_frames\":%zu,\"channels\":%u,\"iq_first_callback_ns\":%llu,\"pcm_first_callback_ns\":%llu,\"iq_layout\":\"ci16_le\",\"pcm_layout\":\"s16_le_interleaved\",\"pcm_rate_hz\":48000}\n",iq_count,pcm_count,channels,(unsigned long long)iq_start_ns,(unsigned long long)pcm_start_ns);
    if(n>0)save(".json",json,(size_t)n);
}
