#include "headset_ptt.h"
#include <errno.h>
#include <limits.h>
#include <string.h>

#define HOOK 0x000b0020u
#define FLASH 0x000b0021u
#define DROP 0x000b0026u
#define MUTE 0x000b002fu
#define OFFHOOK_LED 0x00080017u
typedef struct { unsigned page, size, count, id; int32_t minimum,maximum; } globals_t;
static uint32_t usage(unsigned page, uint32_t value, unsigned bytes)
{ return bytes==4 ? value : (page<<16)|value; }
static bool eligible(const headset_ptt_t* s, const headset_field_t* f)
{ return !(f->flags&1) || (s->profile==HEADSET_JABRA510 && f->id==3); }

int headset_ptt_init(headset_ptt_t* s, const uint8_t* data, size_t bytes,
                     unsigned vendor, unsigned product)
{
    memset(s,0,sizeof(*s));
    if (!bytes || bytes>HEADSET_REPORT_MAX) return -EINVAL;
    globals_t g={0},stack[8]; unsigned depth=0,collections=0;
    uint32_t usages[64],lo=0,hi=0; unsigned n=0; bool range=false;
    for (size_t p=0;p<bytes;) {
        unsigned head=data[p++],len=head&3,type=(head>>2)&3,tag=head>>4;
        if (len==3) len=4;
        if (head==0xfe || len>bytes-p) return -EINVAL;
        uint32_t value=0; for(unsigned i=0;i<len;++i)value|=(uint32_t)data[p++]<<(8*i);
        if (type==1) {
            if(tag==0) { if(value>65535)return -EINVAL;g.page=value; }
            else if(tag==1||tag==2) {
                int32_t signed_value=(int32_t)value;
                if(len&&len<4 && (value&(1u<<(len*8-1))))signed_value=(int32_t)(value|(~0u<<(len*8)));
                if(tag==1)g.minimum=signed_value;else g.maximum=g.minimum<0 ? signed_value : (int32_t)value;
            }
            else if(tag==7)g.size=value;
            else if(tag==8) { if(!value||value>255)return -EINVAL;g.id=value;s->numbered=true; }
            else if(tag==9)g.count=value;
            else if(tag==10) { if(depth==8)return -EINVAL;stack[depth++]=g; }
            else if(tag==11) { if(!depth)return -EINVAL;g=stack[--depth]; }
        } else if(type==2) {
            if(tag==0) { if(n==64)return -EINVAL;usages[n++]=usage(g.page,value,len); }
            else if(tag==1) { lo=usage(g.page,value,len);range=true; }
            else if(tag==2)hi=usage(g.page,value,len);
            else return -ENOTSUP;
        } else if(type==0) {
            if(tag==10) { if(++collections>32)return -EINVAL; }
            if(tag==12) { if(!collections)return -EINVAL;--collections; }
            if(tag==8 || tag==9) {
                if(!g.size||!g.count||g.size>32768 || g.count>32768/g.size)return -EINVAL;
                unsigned* total=tag==8 ? s->input_bits : s->output_bits;
                unsigned start=total[g.id],width=g.size*g.count;
                if(start>32768-width)return -EINVAL;
                total[g.id]+=width;
                if(range && hi<lo)return -EINVAL;
                for(unsigned i=0;i<g.count;++i) {
                    uint32_t u=i<n ? usages[i] : range && i-n<=hi-lo ? lo+i-n : n ? usages[n-1] : 0;
                    if(!(value&2))continue;
                    if(tag==8 && (u==HOOK||u==FLASH||u==DROP||u==MUTE)) {
                        if(g.size>32||g.minimum!=0||g.maximum!=1||s->count==HEADSET_FIELDS_MAX)return -ENOTSUP;
                        s->fields[s->count++]=(headset_field_t){.usage=u,.id=g.id,
                            .offset=start+i*g.size,.bits=g.size,.flags=value};
                    } else if(tag==9 && u==OFFHOOK_LED && !(value&1)) {
                        if(g.size!=1 || s->feedback)return -ENOTSUP;
                        s->feedback=true;s->feedback_id=g.id;s->feedback_offset=start+i*g.size;
                    }
                }
            }
            n=0;range=false;lo=hi=0;
        }
    }
    if(depth || collections || (s->numbered && s->input_bits[0]))return -EINVAL;
    bool hook8=false,mute8=false,hook3=false,flash3=false;
    for(unsigned i=0;i<s->count;++i) {
        headset_field_t* f=&s->fields[i];
        if(f->bits!=1)continue;
        hook8|=f->usage==HOOK&&f->id==8&&f->offset==1;
        mute8|=f->usage==MUTE&&f->id==8&&f->offset==0;
        hook3|=f->usage==HOOK&&f->id==3&&f->offset==0;
        flash3|=f->usage==FLASH&&f->id==3&&f->offset==5;
    }
    if(vendor==0x046d&&product==0x0a46) {
        if(!hook8||!mute8||s->input_bits[8]!=8)return -ENOTSUP;
        s->profile=HEADSET_H650E;s->feedback=false;
    } else if(vendor==0x0b0e&&product==0x0420) {
        if(!hook3||!flash3||s->input_bits[3]!=16||!s->feedback||s->feedback_id!=3)return -ENOTSUP;
        s->profile=HEADSET_JABRA510;
    }
    bool call=s->profile==HEADSET_H650E;
    for(unsigned i=0;i<s->count;++i)
        call|=eligible(s,&s->fields[i]) && (s->fields[i].usage==HOOK||s->fields[i].usage==FLASH);
    return call ? 0 : -ENOTSUP;
}
const char* headset_ptt_profile_name(const headset_ptt_t* s)
{ return s->profile==HEADSET_H650E ? "H650e call pulse" :
    s->profile==HEADSET_JABRA510 ? "Jabra SPEAK 510 telephony" : "USB HID Telephony"; }
static bool input(const headset_ptt_t* s, const uint8_t** data, size_t bytes, unsigned* id)
{
    *id=0;
    if(s->numbered) { if(!bytes)return false;*id=(*data)[0];++*data;--bytes; }
    return s->input_bits[*id] && bytes==(s->input_bits[*id]+7)/8;
}
static uint32_t get(const uint8_t* data, const headset_field_t* f)
{
    uint32_t value=0;
    for(unsigned bit=0;bit<f->bits;++bit)
        value|=(uint32_t)((data[(f->offset+bit)/8]>>((f->offset+bit)%8))&1)<<bit;
    return value;
}
void headset_ptt_prime(headset_ptt_t* s, const uint8_t* data, size_t bytes)
{
    unsigned id;if(!input(s,&data,bytes,&id))return;
    for(unsigned i=0;i<s->count;++i)if(s->fields[i].id==id) {
        s->fields[i].last=get(data,&s->fields[i]);s->fields[i].seen=true;
    }
}
void headset_ptt_feed(headset_ptt_t* s, const uint8_t* data, size_t bytes, double now)
{
    unsigned id;const uint8_t* original=data;
    if(!input(s,&data,bytes,&id))return;
    bool event=false;
    if(s->profile==HEADSET_H650E) {
        if(bytes==2&&original[0]==8&&original[1]==1)s->pending_flash=event=true;
    } else for(unsigned i=0;i<s->count;++i) {
        headset_field_t* f=&s->fields[i];
        if(f->id!=id||!eligible(s,f))continue;
        uint32_t value=get(data,f),previous=f->last;bool seen=f->seen;
        f->seen=true;f->last=value;
        if(value>1)continue;
        if(f->usage==DROP && value) { s->pending_drop=event=true; }
        else if(f->usage==FLASH && value && ((f->flags&4)||!seen||!previous)) {
            s->pending_flash=event=true;
        } else if(f->usage==HOOK) {
            if(f->flags&4) { if(value)s->pending_rise=event=true; }
            else if(seen && value!=previous) {
                if(value) {
                    /* This Jabra sends Hook=1 before EVERY red press, even
                     * when already disconnected. Its green button is Flash
                     * while our call-control session is active. Never turn
                     * that red pre-state into TX, regardless of USB timing. */
                    if(s->profile!=HEADSET_JABRA510)s->pending_rise=event=true;
                } else s->pending_fall=event=true;
            }
        }
    }
    /* Combine simultaneous scalar controls; explicit Drop has precedence. */
    if(event)s->deadline=now+.010;
}
headset_action_t headset_ptt_poll(headset_ptt_t* s, double now)
{
    if(!s->deadline||now<s->deadline)return HEADSET_NONE;
    headset_action_t action=s->pending_drop ? HEADSET_OFF :
        s->pending_flash ? HEADSET_TOGGLE : s->pending_fall ? HEADSET_OFF :
        s->pending_rise ? HEADSET_TOGGLE : HEADSET_NONE;
    s->pending_drop=s->pending_flash=s->pending_fall=s->pending_rise=false;s->deadline=0;
    if(action==HEADSET_TOGGLE) {
        if(s->have_toggle && now-s->last_toggle<.25)return HEADSET_NONE;
        s->have_toggle=true;s->last_toggle=now;
    }
    return action;
}
int headset_ptt_wait_ms(const headset_ptt_t* s,double now)
{
    if(!s->deadline)return -1;
    double ms=(s->deadline-now)*1000;
    return ms<=0 ? 0 : ms>=INT_MAX-1 ? INT_MAX : (int)ms+1;
}
size_t headset_ptt_feedback_size(const headset_ptt_t* s)
{ return s->feedback ? 1+(s->output_bits[s->feedback_id]+7)/8 : 0; }
int headset_ptt_feedback(const headset_ptt_t* s,uint8_t* report,size_t bytes,bool active)
{
    if(!s->feedback||bytes!=headset_ptt_feedback_size(s)||report[0]!=s->feedback_id)return -EINVAL;
    unsigned byte=1+s->feedback_offset/8,mask=1u<<(s->feedback_offset%8);
    if(active)report[byte]|=mask;else report[byte]&=(uint8_t)~mask;
    return 0;
}
