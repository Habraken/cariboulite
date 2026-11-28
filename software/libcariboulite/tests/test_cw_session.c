#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "cw_session.h"
#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

static cw_status_t status(cw_session_t* s){cw_status_t out;cw_session_status(s,&out);return out;}
static void* blocked_sink(void* arg)
{
    int16_t pcm[480]={0};audio_sink_t* sink=arg;
    for(;;)assert(audio_sink_write(sink,pcm,480).status==AUDIO_SINK_OK);
    return NULL;
}
int main(void)
{
    cw_session_t* s=cw_session_create(NULL,NULL,true);assert(s);
    assert(!cw_session_enable(s,true));
    assert(!cw_session_test_contacts(s,1));usleep(30000);
    assert(!cw_session_test_contacts(s,0));usleep(100000);
    /* Simulate a slow RX->TX transition. The already released first mark
     * must be retained rather than sampling only the current contact level. */
    assert(status(s).requested);
    assert(cw_session_configure(s,CW_IAMBIC,12,false,false,250,5)<0);
    float envelope[480];double sum=0;unsigned frames=0;
    while(status(s).requested){assert(!cw_session_render(s,envelope,480));for(unsigned i=0;i<480;++i)sum+=envelope[i];assert(++frames<100);}
    assert(sum>900&&sum<2200);assert(frames>=27&&frames<=36);
    assert(!cw_session_configure(s,CW_IAMBIC,5,false,false,0,5));
    assert(!cw_session_test_contacts(s,1));usleep(30000);assert(!cw_session_test_contacts(s,0));
    frames=0;
    while(status(s).requested){assert(!cw_session_render(s,envelope,480));assert(++frames<30);}
    cw_session_cancel(s);assert(!cw_session_enable(s,true));
    assert(!cw_session_configure(s,CW_STRAIGHT,12,false,true,250,5));
    assert(!cw_session_test_contacts(s,2));usleep(100000);assert(!status(s).requested&&status(s).busy);
    assert(!cw_session_test_contacts(s,0));usleep(30000);
    assert(!cw_session_configure(s,CW_COOTIE,12,false,false,250,5));
    assert(!cw_session_test_contacts(s,3));usleep(20000);
    for(unsigned i=0;i<60;++i){assert(!cw_session_render(s,envelope,480));assert(status(s).requested);}
    cw_session_cancel(s);assert(!status(s).requested&&!status(s).enabled&&!status(s).contacts);
    assert(cw_session_test_contacts(s,1)<0);
    cw_session_receive(s,true);pthread_t writer;assert(!pthread_create(&writer,NULL,blocked_sink,cw_session_rx_sink(s)));
    usleep(20000);pthread_cancel(writer);pthread_join(writer,NULL);
    assert(!status(s).enabled); /* cancellation must release mixer mutex */
    cw_session_receive(s,false);
    assert(!cw_session_enable(s,true));cw_session_cancel(s);
    cw_session_destroy(s);
    puts("CW session: first mark buffered, practice RF inhibit, held key, release, mixer cancellation PASS");
}
