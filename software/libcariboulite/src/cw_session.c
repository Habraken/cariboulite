#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "cw_session.h"
#include "alsa_sink.h"
#include "math_compat.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/ioctl.h>

#define CW_RING 262144u /* >5 seconds; overflow disarms rather than losing edges */
#define RX_RING 16384u
#define BLOCK 240u
struct cw_session {
    audio_sink_t virtual_sink;
    audio_sink_t* output;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    bool created,exit,simulated,receiving;
    char* input;
    int fd;
    unsigned pending_contacts;
    cw_keyer_t keyer;
    cw_envelope_t envelope;
    cw_status_t status;
    float pitch,amplitude,volume;
    double phase;
    float rf[CW_RING];
    int16_t rx[RX_RING];
    unsigned rx_read,rx_write,rx_count;
    uint64_t write,read,last_mark;
};
static void disarm(cw_session_t* s,int error)
{
    if(s->fd>=0){ioctl(s->fd,EVIOCGRAB,0);close(s->fd);s->fd=-1;}
    s->status.enabled=s->status.connected=s->status.requested=false;
    s->pending_contacts=0;s->status.contacts=0;s->status.error=error;cw_keyer_release(&s->keyer);
    s->envelope.position=0;s->read=s->write;
    pthread_cond_broadcast(&s->changed);
}
static void unlock_session(void* mutex){pthread_mutex_unlock(mutex);}
static audio_sink_result_t receive_audio(audio_sink_t* sink,const int16_t* pcm,volatile size_t n)
{
    cw_session_t* s=(cw_session_t*)sink;
    if(n>1920)return (audio_sink_result_t){0,AUDIO_SINK_ERROR,-EMSGSIZE};
    pthread_mutex_lock(&s->mutex);
    pthread_cleanup_push(unlock_session,&s->mutex);
    while(s->receiving && !s->exit && s->rx_count+n>1920)
        pthread_cond_wait(&s->changed,&s->mutex);
    if(s->receiving&&!s->exit){
        for(size_t i=0;i<n;++i){s->rx[s->rx_write]=pcm[i];s->rx_write=(s->rx_write+1)%RX_RING;}
        s->rx_count+=(unsigned)n;
    }
    pthread_cleanup_pop(1);
    return (audio_sink_result_t){n,AUDIO_SINK_OK,0};
}

static void borrowed_destroy(audio_sink_t* sink){(void)sink;}
static const char* borrowed_state(audio_sink_t* sink){(void)sink;return "CW_MIXER";}
static bool borrowed_stats(audio_sink_t* sink,audio_sink_stats_t* out)
{
    cw_session_t* s=(cw_session_t*)sink;
    return audio_sink_get_stats(s->output,out);
}
static const audio_sink_ops_t sink_ops={.write=receive_audio,.destroy=borrowed_destroy,.state=borrowed_state,.get_stats=borrowed_stats};
static void input_events(cw_session_t* s)
{
    if(s->fd<0)return;
    struct input_event events[32];
    for(;;){
        ssize_t n=read(s->fd,events,sizeof(events));
        if(n<0&&(errno==EAGAIN||errno==EINTR))return;
        if(n<=0||n%(ssize_t)sizeof(events[0])){disarm(s,n<0?-errno:-ENODEV);return;}
        for(size_t i=0;i<(size_t)n/sizeof(events[0]);++i){
            struct input_event* e=&events[i];
            if(e->type==EV_SYN&&e->code==SYN_DROPPED){disarm(s,-EOVERFLOW);return;}
            if(e->type==EV_SYN&&e->code==SYN_REPORT){
                s->status.contacts=s->pending_contacts;
                cw_keyer_contacts(&s->keyer,s->pending_contacts);continue;
            }
            if(e->type!=EV_KEY||e->value==2)continue;
            unsigned bit=e->code==KEY_LEFTBRACE ? 1 : e->code==KEY_RIGHTBRACE ? 2 : 0;
            if(bit){
                /* Retain raw contact state separately from configurable swap. */
                unsigned raw=s->pending_contacts;
                raw=e->value ? raw|bit : raw&~bit;
                s->pending_contacts=raw;
            }
        }
    }
}
static void* audio_thread(void* arg)
{
    cw_session_t* s=arg;
    if(!s->simulated){
        struct sched_param priority={.sched_priority=46};
        int rc=pthread_setschedparam(pthread_self(),SCHED_FIFO,&priority);
        if(rc)fprintf(stderr,"[cw] sidetone FIFO/46 unavailable: %s\n",strerror(rc));
    }
    struct timespec next;clock_gettime(CLOCK_MONOTONIC,&next);
    while(true){
        int16_t pcm[BLOCK];
        pthread_mutex_lock(&s->mutex);
        if(s->exit){pthread_mutex_unlock(&s->mutex);break;}
        input_events(s);
        float gain=s->volume<=-120?0:powf(10,s->volume/20);
        for(unsigned i=0;i<BLOCK;++i){
            bool mark=s->status.enabled&&cw_keyer_tick(&s->keyer);
            if(mark&&!s->status.practice&&!s->status.requested){s->read=s->write;s->status.requested=true;}
            float envelope=cw_envelope_tick(&s->envelope,mark,s->status.ramp_ms);
            if(envelope>0)s->last_mark=s->write;
            if(s->status.requested&&s->write-s->read>=CW_RING){disarm(s,-ENOBUFS);envelope=0;}
            s->rf[s->write%CW_RING]=envelope;
            ++s->write;
            float voice=0;
            if(s->receiving&&s->rx_count){voice=s->rx[s->rx_read];s->rx_read=(s->rx_read+1)%RX_RING;--s->rx_count;}
            float tone=envelope*s->amplitude*32767*gain*sin(s->phase);
            s->phase+=2*M_PI*s->pitch/48000;if(s->phase>=2*M_PI)s->phase-=2*M_PI;
            pcm[i]=(int16_t)lrintf(fmaxf(-32768,fminf(32767,voice+tone)));
        }
        s->status.audio_samples=s->write;
        s->status.busy=cw_keyer_busy(&s->keyer)||s->envelope.position>0||s->status.requested;
        pthread_cond_broadcast(&s->changed);
        pthread_mutex_unlock(&s->mutex);
        if(s->output){
            size_t offset=0;
            while(offset<BLOCK){
                audio_sink_result_t result=audio_sink_write(s->output,pcm+offset,BLOCK-offset);
                if(result.status==AUDIO_SINK_ERROR){pthread_mutex_lock(&s->mutex);disarm(s,result.error);s->exit=true;pthread_mutex_unlock(&s->mutex);return NULL;}
                offset+=result.frames;
                if(!result.frames){struct timespec delay={.tv_nsec=1000000};nanosleep(&delay,NULL);}
            }
        } else {
            next.tv_nsec+=5000000;
            if(next.tv_nsec>=1000000000){next.tv_nsec-=1000000000;++next.tv_sec;}
            clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next,NULL);
        }
    }
    return NULL;
}
cw_session_t* cw_session_create(const char* input,const char* playback,bool simulated)
{
    cw_session_t* s=calloc(1,sizeof(*s));if(!s)return NULL;
    s->fd=-1;s->simulated=simulated;s->input=strdup(input?input:"");
    s->virtual_sink=(audio_sink_t){48000,&sink_ops};
    s->status=(cw_status_t){.wpm=12,.hang_ms=250,.ramp_ms=5,.type=CW_STRAIGHT};
    s->pitch=600;s->amplitude=.4;
    cw_keyer_configure(&s->keyer,CW_STRAIGHT,12,false);
    pthread_mutex_init(&s->mutex,NULL);
    pthread_condattr_t attr;pthread_condattr_init(&attr);pthread_condattr_setclock(&attr,CLOCK_MONOTONIC);
    pthread_cond_init(&s->changed,&attr);pthread_condattr_destroy(&attr);
    if(!s->input)goto fail;
    if(!simulated){s->output=alsa_sink_open_low_latency(playback);if(!s->output)goto fail;}
    int thread_error=pthread_create(&s->thread,NULL,audio_thread,s);
    if(thread_error){errno=thread_error;goto fail;}
    s->created=true;return s;
fail:
    cw_session_destroy(s);return NULL;
}
void cw_session_destroy(cw_session_t* s)
{
    if(!s)return;
    pthread_mutex_lock(&s->mutex);s->exit=true;disarm(s,0);pthread_mutex_unlock(&s->mutex);
    if(s->created)pthread_join(s->thread,NULL);
    audio_sink_destroy(s->output);pthread_cond_destroy(&s->changed);pthread_mutex_destroy(&s->mutex);free(s->input);free(s);
}
int cw_session_enable(cw_session_t* s,bool enabled)
{
    if(!s)return -EINVAL;
    pthread_mutex_lock(&s->mutex);
    if(!enabled){disarm(s,0);pthread_mutex_unlock(&s->mutex);return 0;}
    if(s->exit){pthread_mutex_unlock(&s->mutex);return -EIO;}
    if(s->status.enabled){pthread_mutex_unlock(&s->mutex);return 0;}
    int fd=-1,error=0;
    if(!s->simulated){
        fd=open(s->input,O_RDONLY|O_NONBLOCK|O_CLOEXEC);
        unsigned long keys[(KEY_MAX+8*sizeof(long))/(8*sizeof(long))];memset(keys,0,sizeof(keys));
        if(fd<0)error=-errno;
        else if(ioctl(fd,EVIOCGRAB,1)<0)error=-errno;
        else if(ioctl(fd,EVIOCGKEY(sizeof(keys)),keys)<0)error=-errno;
        else if((keys[KEY_LEFTBRACE/(8*sizeof(long))]&(1ul<<(KEY_LEFTBRACE%(8*sizeof(long)))))||
                (keys[KEY_RIGHTBRACE/(8*sizeof(long))]&(1ul<<(KEY_RIGHTBRACE%(8*sizeof(long))))))error=-EBUSY;
    }
    if(error){if(fd>=0)close(fd);s->status.error=error;pthread_mutex_unlock(&s->mutex);return error;}
    s->fd=fd;s->status.enabled=s->status.connected=true;s->status.error=0;s->status.contacts=0;s->pending_contacts=0;
    cw_keyer_release(&s->keyer);
    pthread_mutex_unlock(&s->mutex);return 0;
}
int cw_session_configure(cw_session_t* s,cw_key_type_t type,unsigned speed,bool swap,bool practice,unsigned hang,float ramp)
{
    uint32_t bits;memcpy(&bits,&ramp,sizeof(bits));
    if(!s||hang>5000||(bits&0x7f800000u)==0x7f800000u||ramp<1||ramp>20)return -EINVAL;
    pthread_mutex_lock(&s->mutex);
    if(s->status.requested||s->envelope.position>0||!cw_keyer_configure(&s->keyer,type,speed,swap)){
        pthread_mutex_unlock(&s->mutex);return -EBUSY;
    }
    s->status.type=type;s->status.wpm=speed;s->status.swap=swap;s->status.practice=practice;
    s->status.hang_ms=hang;s->status.ramp_ms=ramp;
    pthread_mutex_unlock(&s->mutex);return 0;
}
void cw_session_set_audio(cw_session_t* s,float pitch,float amp,float db)
{
    if(!s)return;
    pthread_mutex_lock(&s->mutex);s->pitch=pitch;s->amplitude=amp;s->volume=db;pthread_mutex_unlock(&s->mutex);
}
void cw_session_status(cw_session_t* s,cw_status_t* status)
{
    *status=(cw_status_t){0};if(!s)return;
    pthread_mutex_lock(&s->mutex);*status=s->status;pthread_mutex_unlock(&s->mutex);
}
void cw_session_cancel(cw_session_t* s)
{ if(s){pthread_mutex_lock(&s->mutex);disarm(s,s->status.error);pthread_mutex_unlock(&s->mutex);} }
int cw_session_render(cw_session_t* s,float* out,volatile size_t n)
{
    int error;
    pthread_mutex_lock(&s->mutex);
    pthread_cleanup_push(unlock_session,&s->mutex);
    for(size_t i=0;i<n;++i){
        struct timespec deadline;clock_gettime(CLOCK_MONOTONIC,&deadline);deadline.tv_nsec+=100000000;
        if(deadline.tv_nsec>=1000000000){deadline.tv_nsec-=1000000000;++deadline.tv_sec;}
        while(s->status.requested&&s->read>=s->write&&!s->exit){
            if(pthread_cond_timedwait(&s->changed,&s->mutex,&deadline)){disarm(s,-ETIMEDOUT);break;}
        }
        out[i]=s->status.requested?s->rf[s->read++%CW_RING]:0;
        if(s->status.requested&&!s->keyer.contacts&&!s->keyer.mark&&s->envelope.position<=0&&
           s->read>=s->last_mark+1+(uint64_t)s->status.hang_ms*48){s->status.requested=false;}
    }
    s->status.rf_samples=s->read;
    error=s->status.error;
    pthread_cleanup_pop(1);return error;
}
audio_sink_t* cw_session_rx_sink(cw_session_t* s){return s?&s->virtual_sink:NULL;}
void cw_session_receive(cw_session_t* s,bool on)
{
    if(!s)return;
    pthread_mutex_lock(&s->mutex);if(s->receiving!=on){s->receiving=on;s->rx_count=s->rx_read=s->rx_write=0;pthread_cond_broadcast(&s->changed);}pthread_mutex_unlock(&s->mutex);
}
int cw_session_test_contacts(cw_session_t* s,unsigned contacts)
{
    if(!s||!s->simulated||contacts>3)return -EINVAL;
    pthread_mutex_lock(&s->mutex);
    if(!s->status.enabled){pthread_mutex_unlock(&s->mutex);return -EPERM;}
    s->status.contacts=contacts;cw_keyer_contacts(&s->keyer,contacts);
    pthread_mutex_unlock(&s->mutex);return 0;
}
