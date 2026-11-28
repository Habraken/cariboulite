#include "headset_ptt.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static size_t load(const char* path,uint8_t* data)
{
    FILE* f=fopen(path,"r");assert(f);unsigned value;size_t n=0;
    while(fscanf(f,"%2x",&value)==1) { assert(n<HEADSET_REPORT_MAX);data[n++]=(uint8_t)value; }
    fclose(f);return n;
}
static void feed(headset_ptt_t* s,unsigned id,unsigned bits,double now)
{ uint8_t r[]={id,bits,0};headset_ptt_feed(s,r,id==8?2:3,now); }
int main(int argc,char** argv)
{
    assert(argc==3);headset_ptt_t s;uint8_t data[HEADSET_REPORT_MAX];
    size_t n=load(argv[1],data);assert(!headset_ptt_init(&s,data,n,0x046d,0x0a46));
    assert(s.profile==HEADSET_H650E&&!s.feedback);
    feed(&s,8,2,0);feed(&s,8,3,.001);assert(headset_ptt_poll(&s,.1)==HEADSET_NONE);
    feed(&s,8,1,1);feed(&s,8,3,1.001);assert(headset_ptt_poll(&s,1.005)==HEADSET_NONE);
    assert(headset_ptt_poll(&s,1.02)==HEADSET_TOGGLE);
    feed(&s,8,1,1.1);assert(headset_ptt_poll(&s,1.12)==HEADSET_NONE);
    feed(&s,8,1,2);assert(headset_ptt_poll(&s,2.02)==HEADSET_TOGGLE);
    /* Real Jabra traces: first device feedback is a baseline, not TX. */
    n=load(argv[2],data);assert(!headset_ptt_init(&s,data,n,0x0b0e,0x0420));
    assert(s.profile==HEADSET_JABRA510&&s.feedback&&s.feedback_id==3);
    uint8_t initial[]={3,3,0};headset_ptt_prime(&s,initial,3);
    feed(&s,3,3,0);assert(headset_ptt_poll(&s,.1)==HEADSET_NONE);
    feed(&s,3,0x23,1);feed(&s,3,3,1.001);assert(headset_ptt_poll(&s,1.02)==HEADSET_TOGGLE);
    feed(&s,3,0x23,2);feed(&s,3,3,2.001);assert(headset_ptt_poll(&s,2.02)==HEADSET_TOGGLE);
    feed(&s,3,3,3);feed(&s,3,2,3.001);assert(headset_ptt_poll(&s,3.02)==HEADSET_OFF);
    feed(&s,3,3,4);assert(headset_ptt_poll(&s,4.5)==HEADSET_NONE);
    feed(&s,3,2,4.501);assert(headset_ptt_poll(&s,4.52)==HEADSET_OFF);
    feed(&s,3,0x22,5);feed(&s,3,2,5.001);assert(headset_ptt_poll(&s,5.02)==HEADSET_TOGGLE);
    feed(&s,3,0x12,6);feed(&s,3,2,6.001);assert(headset_ptt_poll(&s,6.02)==HEADSET_NONE);
    feed(&s,4,2,7);assert(headset_ptt_poll(&s,7.02)==HEADSET_NONE);
    /* Unlisted device: relative call + absolute disconnect, arbitrary IDs,
     * padding and a preserved Off-Hook output bit at offset seven. */
    const uint8_t generic[]={0x05,0x0b,0x09,0x05,0xa1,1,0x85,5,0x15,0,0x25,1,
        0x75,1,0x95,3,0x81,1,0x09,0x20,0x95,1,0x81,6,
        0x09,0x26,0x81,2,0x09,0x2f,0x81,6,0x95,2,0x81,1,
        0x85,9,0x05,8,0x95,7,0x91,1,0x09,0x17,0x95,1,0x91,2,0xc0};
    assert(!headset_ptt_init(&s,generic,sizeof(generic),0x1234,0x5678));
    assert(s.profile==HEADSET_GENERIC&&s.feedback_id==9&&s.feedback_offset==7);
    uint8_t output[]={9,0x55};assert(!headset_ptt_feedback(&s,output,2,true)&&output[1]==0xd5);
    assert(!headset_ptt_feedback(&s,output,2,false)&&output[1]==0x55);
    const uint8_t call[]={5,8},idle[]={5,0},mute[]={5,32},stop[]={5,16},both[]={5,24};
    headset_ptt_feed(&s,call,2,0);headset_ptt_feed(&s,idle,2,.001);
    assert(headset_ptt_poll(&s,.02)==HEADSET_TOGGLE);
    headset_ptt_feed(&s,mute,2,1);assert(headset_ptt_poll(&s,1.02)==HEADSET_NONE);
    headset_ptt_feed(&s,stop,2,2);assert(headset_ptt_poll(&s,2.02)==HEADSET_OFF);
    headset_ptt_feed(&s,both,2,3);assert(headset_ptt_poll(&s,3.02)==HEADSET_OFF);
    /* Absolute switches are primed before arming; no initial-state TX. */
    uint8_t absolute[sizeof(generic)];memcpy(absolute,generic,sizeof(generic));absolute[23]=2;
    assert(!headset_ptt_init(&s,absolute,sizeof(absolute),0,0));
    headset_ptt_feed(&s,call,2,0);assert(headset_ptt_poll(&s,.02)==HEADSET_NONE);
    headset_ptt_prime(&s,idle,2);headset_ptt_feed(&s,call,2,1);
    assert(headset_ptt_poll(&s,1.02)==HEADSET_TOGGLE);
    /* Usage ranges and Global Push/Pop retain non-byte-aligned offsets. */
    const uint8_t ranged[]={0x05,0x0b,0x09,5,0xa1,1,0x15,0,0x25,1,0x75,1,
        0x85,7,0x19,0x20,0x29,0x21,0x95,2,0x81,2,0xa4,
        0x05,9,0x75,8,0x95,1,0x81,1,0xb4,0x09,0x26,0x95,1,0x81,2,
        0x95,5,0x81,1,0xc0};
    assert(!headset_ptt_init(&s,ranged,sizeof(ranged),0,0));
    assert(s.input_bits[7]==16&&s.fields[2].offset==10);
    uint8_t zero[]={7,0,0},hook[]={7,1,0},drop[]={7,1,4},flash[]={7,2,0};
    headset_ptt_prime(&s,zero,3);headset_ptt_feed(&s,hook,3,0);
    assert(headset_ptt_poll(&s,.02)==HEADSET_TOGGLE);
    headset_ptt_feed(&s,drop,3,1);assert(headset_ptt_poll(&s,1.02)==HEADSET_OFF);
    headset_ptt_feed(&s,flash,3,2);assert(headset_ptt_poll(&s,2.02)==HEADSET_TOGGLE);
    const uint8_t unnumbered[]={0x05,0x0b,0x09,5,0xa1,1,0x15,0,0x25,1,0x75,1,
        0x95,1,0x09,0x20,0x81,6,0x09,0x26,0x81,2,0x95,6,0x81,1,0xc0};
    assert(!headset_ptt_init(&s,unnumbered,sizeof(unnumbered),0,0)&&!s.numbered);
    uint8_t ucall[]={1},ustop[]={2};headset_ptt_feed(&s,ucall,1,0);
    assert(headset_ptt_poll(&s,.02)==HEADSET_TOGGLE);
    headset_ptt_feed(&s,ustop,1,1);assert(headset_ptt_poll(&s,1.02)==HEADSET_OFF);
    /* Truncation, bad stacks/counts, irrelevant reports and malformed input
     * must remain bounded. Sanitizer run checks randomized descriptor bytes. */
    assert(headset_ptt_init(&s,generic,sizeof(generic)-1,0,0));
    const uint8_t pop[]={0xb4};assert(headset_ptt_init(&s,pop,sizeof(pop),0,0));
    for(unsigned trial=0;trial<5000;++trial) {
        size_t length=rand()%256;for(size_t i=0;i<length;++i)data[i]=(uint8_t)rand();
        if(!headset_ptt_init(&s,data,length,0,0)) {
            headset_ptt_feed(&s,data,length,1);(void)headset_ptt_poll(&s,2);
        }
    }
    puts("HID decoder: captured H650e/Jabra gestures, generic capabilities, startup, disconnect precedence, feedback preservation and malformed descriptors PASS");
    return 0;
}
