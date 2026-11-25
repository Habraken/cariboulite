#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
/* Isolated TX-process override. Return the real modem frequency so the
 * unmodified SDK computes the matching mixer LO and retains requested RF. */
int64_t at86rf215_setup_channel(void *dev,int channel,uint64_t frequency)
{
    static int64_t (*next)(void*,int,uint64_t);
    if(!next)*(void**)(&next)=dlsym(RTLD_NEXT,"at86rf215_setup_channel");
    if(!next)_exit(98);
    const char *value=getenv("DEMOD_TX_IF_HZ");
    if(value&&strcmp(value,"2385000000"))_exit(98);
    uint64_t selected=value&&channel==1&&frequency==2495000000ULL?2385000000ULL:frequency;
    int64_t result=next(dev,channel,selected);
    if(selected!=frequency)fprintf(stderr,"TX_IF_OVERRIDE channel=%d requested=%llu selected=%llu actual=%lld\n",channel,(unsigned long long)frequency,(unsigned long long)selected,(long long)result);
    return result;
}
