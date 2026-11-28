#include "cw_keyer.h"
#include "math_compat.h"
#include <math.h>
#include <string.h>

bool cw_keyer_busy(const cw_keyer_t* k)
{ return k && (k->contacts || k->mark || k->gap); }
bool cw_keyer_configure(cw_keyer_t* k,cw_key_type_t type,unsigned wpm,bool swap)
{
    if(!k||cw_keyer_busy(k)||type<CW_STRAIGHT||type>CW_COOTIE||wpm<5||wpm>30)return false;
    *k=(cw_keyer_t){.type=type,.wpm=wpm,.swap=swap};return true;
}
void cw_keyer_contacts(cw_keyer_t* k,unsigned contacts)
{
    contacts&=3;
    if(k->swap)contacts=((contacts&1)<<1)|((contacts&2)>>1);
    unsigned pressed=contacts&~k->contacts;
    if(pressed)k->priority=pressed==3 ? (k->priority?k->priority:1) : pressed;
    k->contacts=contacts;
}
void cw_keyer_release(cw_keyer_t* k)
{
    k->contacts=k->priority=k->element=k->previous=0;
    k->mark=k->gap=false;k->boundary=0;k->clock=0;
}
static unsigned choose(cw_keyer_t* k)
{
    if(k->contacts!=3)return k->contacts;
    if(k->type==CW_ULTIMATIC)return k->priority?k->priority:1;
    return k->previous==1 ? 2 : 1;
}
bool cw_keyer_tick(cw_keyer_t* k)
{
    if(k->type==CW_STRAIGHT||k->type==CW_COOTIE)return k->contacts!=0;
    if(!k->wpm)return false;
    const double dit=57600.0/k->wpm; /* PARIS: 1200/WPM ms */
    if(k->mark && (double)k->clock>=k->boundary){
        k->mark=false;k->gap=true;k->boundary+=dit;
    }
    if(k->gap && (double)k->clock>=k->boundary)k->gap=false;
    if(!k->mark&&!k->gap){
        unsigned element=choose(k);
        if(element){
            /* Retain the fractional boundary across a continuous element
             * sequence, but start a manually spaced character at this tick. */
            if(k->boundary<(double)k->clock-1)k->boundary=(double)k->clock;
            k->boundary+=(element==2?3:1)*dit;
            k->element=k->previous=element;k->mark=true;
        }
    }
    ++k->clock;return k->mark;
}
float cw_envelope_tick(cw_envelope_t* e,bool down,float ramp_ms)
{
    double step=1.0/(48.0*fmaxf(.1f,ramp_ms));
    e->position=fmax(0,fmin(1,e->position+(down?step:-step)));
    return (float)(.5-.5*cos(M_PI*e->position));
}
