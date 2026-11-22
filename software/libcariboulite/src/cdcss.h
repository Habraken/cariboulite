#pragma once
/* Allocation-free CDCSS at 48 kHz. On-air bit 0 is sent first; logical one is
 * positive RF deviation for normal polarity. See docs/cdcss.md for references. */
#include "ctcss.h"
#include <stddef.h>
#include <limits.h>
#define CDCSS_WORD_MASK UINT32_C(0x7fffff)
#define CDCSS_DEFAULT_DEVIATION_HZ 250.0f

typedef struct { bool enabled, inverted; unsigned code; } cdcss_config_t;
static inline bool cdcss_config_valid(cdcss_config_t c) { return c.code<512; }
static inline bool cdcss_equal(cdcss_config_t a,cdcss_config_t b)
{ return a.enabled==b.enabled && (!a.enabled || (a.code==b.code && a.inverted==b.inverted)); }
static inline bool cdcss_parse(const char *text,cdcss_config_t *out)
{
    if (!text || !out) return false;
    if (!strcmp(text,"off")) { *out=(cdcss_config_t){0}; return true; }
    size_t n=strlen(text);
    if (n!=3 && n!=4) return false;
    unsigned code=0;
    for (unsigned k=0;k<3;k++) { if (text[k]<'0'||text[k]>'7') return false; code=8*code+(unsigned)(text[k]-'0'); }
    char p=n==4 ? text[3] : 'N';
    if (p!='N' && p!='n' && p!='I' && p!='i') return false;
    *out=(cdcss_config_t){.enabled=true,.inverted=p=='I'||p=='i',.code=code}; return true;
}
static inline void cdcss_format(cdcss_config_t c,char text[5])
{
    if (!c.enabled) { memcpy(text,"off",4); return; }
    text[0]='0'+((c.code>>6)&7);text[1]='0'+((c.code>>3)&7);text[2]='0'+(c.code&7);
    text[3]=c.inverted?'I':'N';text[4]=0;
}
static inline uint32_t cdcss_reverse(uint32_t x,unsigned n)
{ uint32_t y=0;for(unsigned k=0;k<n;k++){y=2*y+(x&1);x>>=1;}return y; }
static inline uint32_t cdcss_codeword(unsigned code)
{
    uint32_t data=0x800u|(code&511u),remainder=cdcss_reverse(data,12)<<11;
    /* Golay generator x^11+x^9+x^7+x^6+x^5+x+1. Polynomial division is
     * performed in transmit order, then parity is put back into LSB order. */
    for(int bit=22;bit>=11;bit--) if(remainder&(1u<<bit))remainder^=0xae3u<<(bit-11);
    return data|(cdcss_reverse(remainder,11)<<12);
}
static inline unsigned cdcss_distance(uint32_t a,uint32_t b)
{ return (unsigned)__builtin_popcount((a^b)&CDCSS_WORD_MASK); }

typedef struct {
    cdcss_config_t config;
    uint32_t word;
    unsigned phase,bit,stop_left;
    bool stopping;
    double stop_phase,lp[4];
} cdcss_tx_t;
static inline void cdcss_tx_reset(cdcss_tx_t *s,cdcss_config_t c)
{ memset(s,0,sizeof(*s));s->config=c;s->word=cdcss_codeword(c.code)^(c.inverted?CDCSS_WORD_MASK:0); }
static inline void cdcss_tx_tail(cdcss_tx_t *s,bool tail)
{ if(tail && !s->stopping){s->stopping=true;s->stop_left=8640;s->stop_phase=0;} }
static inline float cdcss_tx_sample(cdcss_tx_t *s)
{
    if(!s->config.enabled)return 0;
    double x=0;
    if(s->stopping) {
        if(s->stop_left){x=sin(s->stop_phase);s->stop_phase+=6.2831853071795864769*134.4/48000;
            if(s->stop_phase>=6.2831853071795864769)s->stop_phase-=6.2831853071795864769;
            --s->stop_left;
        }
    } else {
        x=(s->word>>s->bit)&1 ? 1 : -1;
        s->phase+=14; // 134.4 / 48000 = 14 / 5000 exactly
        if(s->phase>=5000){s->phase-=5000;s->bit=(s->bit+1)%23;}
    }
    // Monotone cascaded poles: bounded NRZ shaping without overshoot.
    const double a=1-exp(-6.2831853071795864769*180/48000);
    for(unsigned k=0;k<4;k++){s->lp[k]+=a*(x-s->lp[k]);x=s->lp[k];}
    return (float)x;
}

enum { CDCSS_CLOCKS=5,CDCSS_PHASES=10,CDCSS_EOT_WINDOW=80 };
typedef struct { uint32_t word;unsigned bits,wait,hits; } cdcss_bank_t;
typedef struct {
    cdcss_config_t config;
    uint32_t rotations[23];
    double lp[4],dc,power;
    unsigned clock_phase[CDCSS_CLOCKS],clock_sample[CDCSS_CLOCKS];
    cdcss_bank_t bank[CDCSS_CLOCKS][CDCSS_PHASES];
    uint64_t samples,last_match;
    bool valid,open,turnoff;
    unsigned errors;
    double eot_i[CDCSS_EOT_WINDOW],eot_q[CDCSS_EOT_WINDOW],eot_e[CDCSS_EOT_WINDOW],eot_x[CDCSS_EOT_WINDOW];
    double sum_i,sum_q,sum_e,sum_x;
    unsigned eot_head,eot_count,eot_phase,eot_hop,eot_hits;
} cdcss_rx_t;
static inline void cdcss_rx_reset(cdcss_rx_t *s,cdcss_config_t c)
{
    memset(s,0,sizeof(*s));s->config=c;s->errors=UINT_MAX;
    uint32_t w=cdcss_codeword(c.code)^(c.inverted?CDCSS_WORD_MASK:0);
    for(unsigned k=0;k<23;k++)s->rotations[k]=((w>>k)|(w<<(23-k)))&CDCSS_WORD_MASK;
}
static inline unsigned cdcss_match(const cdcss_rx_t *s,uint32_t w)
{
    unsigned best=23;
    for(unsigned k=0;k<23;k++){unsigned n=cdcss_distance(w,s->rotations[k]);if(n<best)best=n;}
    return best;
}
static inline bool cdcss_eot_sample(cdcss_rx_t *s,double x)
{
    /* Center clock emits 1344 samples/s: 134.4 Hz turn-off has 10 samples/cycle.
     * Eight cycles reject ordinary NRZ; two successive estimates qualify it. */
    static const double co[10]={1,.8090169943749475,.30901699437494745,-.30901699437494734,-.8090169943749473,-1,-.8090169943749476,-.30901699437494756,.30901699437494723,.8090169943749473};
    static const double si[10]={0,.5877852522924731,.9510565162951535,.9510565162951536,.5877852522924732,0,-.587785252292473,-.9510565162951535,-.9510565162951536,-.5877852522924734};
    unsigned h=s->eot_head,p=s->eot_phase;
    double i=x*co[p],q=x*si[p],e=x*x;
    s->sum_i+=i-s->eot_i[h];s->sum_q+=q-s->eot_q[h];s->sum_e+=e-s->eot_e[h];
    s->sum_x+=x-s->eot_x[h];s->eot_x[h]=x;
    s->eot_i[h]=i;s->eot_q[h]=q;s->eot_e[h]=e;
    s->eot_head=(h+1)%CDCSS_EOT_WINDOW;s->eot_phase=(p+1)%10;
    if(s->eot_count<CDCSS_EOT_WINDOW)++s->eot_count;
    if(++s->eot_hop<10)return false;
    s->eot_hop=0;
    if(s->eot_count<CDCSS_EOT_WINDOW)return false;
    double tone=2*(s->sum_i*s->sum_i+s->sum_q*s->sum_q)/CDCSS_EOT_WINDOW;
    double ac=s->sum_e-s->sum_x*s->sum_x/CDCSS_EOT_WINDOW;
    bool hit=ac>1e-12 && tone>.8*ac && 2*tone/CDCSS_EOT_WINDOW>.000025;
    if(hit){if(++s->eot_hits>=2){s->eot_hits=2;return true;}}else s->eot_hits=0;
    return false;
}
static inline bool cdcss_rx_sample(cdcss_rx_t *s,float input)
{
    if(!s->config.enabled)return true;
    if(!ctcss_finite(input)){cdcss_rx_reset(s,s->config);return false;}
    ++s->samples;
    if(s->open && s->samples-s->last_match>16800)s->open=false; // 350 ms hold
    double x=fmax(-16,fmin(16,input));
    const double a=1-exp(-6.2831853071795864769*300/48000);
    for(unsigned k=0;k<4;k++){s->lp[k]+=a*(x-s->lp[k]);x=s->lp[k];}
    s->dc+=(1-exp(-6.2831853071795864769*2/48000))*(x-s->dc);x-=s->dc;
    s->power+=(x*x-s->power)/4800;
    static const unsigned step[CDCSS_CLOCKS]={13860,13930,14000,14070,14140};
    for(unsigned c=0;c<CDCSS_CLOCKS;c++) {
        s->clock_phase[c]+=step[c];if(s->clock_phase[c]<500000)continue;s->clock_phase[c]-=500000;
        if(c==2 && cdcss_eot_sample(s,x)) {
            s->open=false;s->turnoff=true;
            for(unsigned j=0;j<CDCSS_CLOCKS;j++)for(unsigned k=0;k<CDCSS_PHASES;k++)s->bank[j][k].hits=0;
        }
        unsigned p=s->clock_sample[c];s->clock_sample[c]=(p+1)%CDCSS_PHASES;
        cdcss_bank_t *b=&s->bank[c][p];
        b->word=(b->word>>1)|((uint32_t)(x>=0)<<22);
        if(b->bits<23){++b->bits;if(b->bits<23)continue;}
        s->valid=true;
        if(b->wait){--b->wait;if(b->wait)continue;}
        unsigned errors=cdcss_match(s,b->word);
        if(errors<=3 && s->power>.000036) {
            b->wait=23;if(b->hits<3)++b->hits;
            if(b->hits>=3){s->open=true;s->turnoff=false;s->last_match=s->samples;s->errors=errors;}
        } else {b->hits=0;b->wait=0;}
    }
    return s->open;
}
