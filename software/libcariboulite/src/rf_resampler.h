#pragma once
#include "math_compat.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
/* Allocation-free polyphase RF frontend for native 4 MHz/divider clocks.
 * Existing integer decimators remain the fast paths. Coefficients are built
 * off the worker thread. Output is exactly 4 MHz/target_divider, independent
 * of integer Hz labels or caller block sizes. */
#define RF_RESAMPLE_PHASES 6
#define RF_RESAMPLE_HISTORY 256
#define RF_RESAMPLE_MAX_TAPS 641
typedef struct {
    unsigned up,down,phase,pos,length;
    float coeff[RF_RESAMPLE_PHASES][RF_RESAMPLE_HISTORY];
    float i[2*RF_RESAMPLE_HISTORY],q[2*RF_RESAMPLE_HISTORY];
} rf_resampler_t;
static inline unsigned rf_gcd(unsigned a,unsigned b){while(b){unsigned r=a%b;a=b;b=r;}return a;}
static inline void rf_resampler_reset(rf_resampler_t *s)
{
    s->phase=s->pos=0;memset(s->i,0,sizeof(s->i));memset(s->q,0,sizeof(s->q));
}
static inline bool rf_resampler_init(rf_resampler_t *s,unsigned divider,unsigned target,float cutoff)
{
    memset(s,0,sizeof(*s));
    if(!divider||!target)return false;
    unsigned g=rf_gcd(divider,target);if(!g)return false;
    s->up=divider/g;s->down=target/g;
    unsigned fs_up=4000000/g,n=fs_up/6250+1;
    if(!s->up||s->up>=RF_RESAMPLE_PHASES||s->up>s->down)return false;
    s->length=(n+s->up-1)/s->up;
    if(s->length>RF_RESAMPLE_HISTORY||n>RF_RESAMPLE_MAX_TAPS)return false;
    for(unsigned p=0;p<=s->up;++p){
        double sum=0;
        for(unsigned j=0;j<s->length;++j){
            unsigned k=p+j*s->up;if(k>=n)continue;
            int t=(int)k-(int)n/2;
            double v=t?sin(2*M_PI*cutoff*t/fs_up)/(M_PI*t):2*cutoff/fs_up;
            v*=.42-.5*cos(2*M_PI*k/(n-1))+.08*cos(4*M_PI*k/(n-1));
            s->coeff[p][j]=(float)v;sum+=v;
        }
        for(unsigned j=0;j<s->length;++j)s->coeff[p][j]/=(float)sum;
    }
    return true;
}
static inline bool rf_resampler_push(rf_resampler_t *s,float i,float q,float *oi,float *oq)
{
    unsigned p=s->pos,n=s->length;
    s->i[p]=s->i[p+n]=i;s->q[p]=s->q[p+n]=q;
    s->pos=p+1==n?0:p+1;
    s->phase+=s->up;if(s->phase<s->down)return false;s->phase-=s->down;
    const float *h=s->coeff[s->up-s->phase],*hi=s->i+s->pos+n-1,*hq=s->q+s->pos+n-1;
    float yi=0,yq=0;
    for(unsigned k=0;k<n;++k){yi+=h[k]*hi[-(int)k];yq+=h[k]*hq[-(int)k];}
    *oi=yi;*oq=yq;return true;
}
