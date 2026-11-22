#include "tx_dynamics.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static float settle(tx_dynamics_config_t c, float input)
{
    tx_dynamics_t s; assert(tx_dynamics_init(&s,c));
    float frame[480];
    for (int j=0;j<400;++j) {
        for (int i=0;i<480;++i) frame[i]=input;
        tx_dynamics_process(&s,frame,480);
    }
    return frame[479];
}
int main(void)
{
    tx_dynamics_config_t c=tx_dynamics_defaults();
    assert(c.enabled);
    c.enabled=false; // explicit bypass must remain bit-exact
    tx_dynamics_t s; assert(tx_dynamics_init(&s,c));
    float bypass[]={0,.01f,-.3f,1.5f,-2}, original[5];
    memcpy(original,bypass,sizeof(bypass));
    tx_dynamics_process(&s,bypass,5);assert(!memcmp(original,bypass,sizeof(bypass)));
    assert(!tx_dynamics_set(&c,"ratio",0));assert(c.ratio==3);
    assert(!tx_dynamics_set(&c,"attack",0));assert(!tx_dynamics_set(&c,"enabled",2));
    assert(!tx_dynamics_set(&c,"unknown",0));
    uint32_t bits=UINT32_C(0x7fc00000);float nan;memcpy(&nan,&bits,4);
    assert(!tx_dynamics_set(&c,"makeup",nan));
    assert(tx_dynamics_set(&c,"enabled",1));
    double quiet=settle(c,.01f),loud=settle(c,.316227766f);
    assert(fabs(20*log10(quiet/.01)-6)<.01); /* quiet speech lifted */
    assert(fabs(20*log10(loud)-(-24+14./3+6))<.02);
    double a=settle(c,.1f),b=settle(c,.2f);
    assert(fabs(20*log10(b/a)-20*log10(2)/3)<.02); /* 3:1 slope */
    c.ratio=1;c.makeup_db=0;c.threshold_db=0;c.ceiling_db=-1;
    assert(tx_dynamics_init(&s,c));
    float impulse[100]={0};impulse[0]=20;
    tx_dynamics_process(&s,impulse,100);
    for (int i=0;i<100;++i) {
        assert(fabs(impulse[i])<=pow(10.,-1./20)+1e-6);
        if (i!=48) assert(impulse[i]==0);
    }
    assert(impulse[48]>.1); /* exactly 48 samples of lookahead */
    c.input_db=24;c.makeup_db=24;
    assert(tx_dynamics_init(&s,c));
    float step[480];
    for (int j=0;j<20;++j) {
        for (int i=0;i<480;++i)step[i]=(i%17==0 ? -1.f : .5f);
        tx_dynamics_process(&s,step,480);
        for (int i=0;i<480;++i)assert(fabs(step[i])<=s.ceiling+1e-6);
    }
    tx_dynamics_reset(&s);memset(step,0,sizeof(step));tx_dynamics_process(&s,step,480);
    for (int i=0;i<480;++i)assert(step[i]==0); /* no speech leaks across PTT */
    step[0]=nan;bits=UINT32_C(0x7f800000);memcpy(&step[1],&bits,4);
    step[2]=3.4e38f;tx_dynamics_process(&s,step,480);
    for (int i=0;i<480;++i)assert(tx_dynamics_finite(step[i]) && fabs(step[i])<=s.ceiling+1e-6);
    /* Identical state and samples independent of caller frame partitioning. */
    enum {N=6000};float whole[N],split[N];
    for(int i=0;i<N;++i)whole[i]=split[i]=.4f*sinf(i*.113f);
    c=tx_dynamics_defaults();c.enabled=true;
    tx_dynamics_t t;assert(tx_dynamics_init(&s,c));assert(tx_dynamics_init(&t,c));
    tx_dynamics_process(&s,whole,N);
    for(int i=0;i<N;) {int n=(i%113)+1;if(n>N-i)n=N-i;tx_dynamics_process(&t,split+i,n);i+=n;}
    assert(!memcmp(whole,split,sizeof(whole)));
    /* Release returns toward unity on silence; the finite-gain design never
     * learns background noise into a rising auto-makeup gain. */
    memset(step,0,sizeof(step));double before=t.reduction_db;
    for(int j=0;j<200;++j) {memset(step,0,sizeof(step));tx_dynamics_process(&t,step,480);}
    assert(t.reduction_db<before && t.reduction_db<.01);
    puts("TX dynamics: bypass, curve, gain, peak ceiling, lookahead, reset, finite handling, partitioning and release passed");
    return 0;
}
