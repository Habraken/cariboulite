#pragma once
/* Application-owned 48 kHz microphone conditioner. No allocation or locks in
 * process(); the parked TX lifecycle owner configures/resets it. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

typedef struct {
    bool enabled;
    float input_db, threshold_db, ratio, knee_db, attack_ms, release_ms;
    float makeup_db, ceiling_db;
} tx_dynamics_config_t;

static inline tx_dynamics_config_t tx_dynamics_defaults(void)
{
    return (tx_dynamics_config_t){.enabled=true, .threshold_db=-24, .ratio=3, .knee_db=6,
        .attack_ms=5, .release_ms=200, .makeup_db=6, .ceiling_db=-1};
}

/* Bit tests remain valid in the production -ffast-math build. */
static inline bool tx_dynamics_finite(float value)
{
    uint32_t bits; memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}
static inline bool tx_dynamics_range(float x, float lo, float hi)
{
    return tx_dynamics_finite(x) && x>=lo && x<=hi;
}
static inline bool tx_dynamics_valid(const tx_dynamics_config_t *c)
{
    return c && tx_dynamics_range(c->input_db,-24,24) &&
        tx_dynamics_range(c->threshold_db,-60,0) && tx_dynamics_range(c->ratio,1,10) &&
        tx_dynamics_range(c->knee_db,0,24) && tx_dynamics_range(c->attack_ms,.1f,100) &&
        tx_dynamics_range(c->release_ms,10,2000) && tx_dynamics_range(c->makeup_db,0,24) &&
        tx_dynamics_range(c->ceiling_db,-12,0);
}

static inline bool tx_dynamics_equal(tx_dynamics_config_t a, tx_dynamics_config_t b)
{
    return a.enabled==b.enabled && a.input_db==b.input_db && a.threshold_db==b.threshold_db &&
        a.ratio==b.ratio && a.knee_db==b.knee_db && a.attack_ms==b.attack_ms &&
        a.release_ms==b.release_ms && a.makeup_db==b.makeup_db && a.ceiling_db==b.ceiling_db;
}

/* Stable parameter names shared by CLI, wire protocol and controller. */
static inline bool tx_dynamics_set(tx_dynamics_config_t *c, const char *name, float value)
{
    if (!c || !name || !tx_dynamics_finite(value)) return false;
    tx_dynamics_config_t next=*c;
    if (!strcmp(name,"enabled")) {
        if (value!=0 && value!=1) return false;
        next.enabled=value!=0;
    } else if (!strcmp(name,"input")) next.input_db=value;
    else if (!strcmp(name,"threshold")) next.threshold_db=value;
    else if (!strcmp(name,"ratio")) next.ratio=value;
    else if (!strcmp(name,"knee")) next.knee_db=value;
    else if (!strcmp(name,"attack")) next.attack_ms=value;
    else if (!strcmp(name,"release")) next.release_ms=value;
    else if (!strcmp(name,"makeup")) next.makeup_db=value;
    else if (!strcmp(name,"ceiling")) next.ceiling_db=value;
    else return false;
    if (!tx_dynamics_valid(&next)) return false;
    *c=next; return true;
}

#define TX_DYNAMICS_LOOKAHEAD 48 /* 1 ms at the fixed 48 kHz audio rate */
typedef struct {
    tx_dynamics_config_t config;
    double rms_squared, reduction_db, limiter_gain;
    double input_gain, makeup_gain, ceiling;
    double rms_coeff, attack_coeff, release_coeff, limiter_release_coeff;
    float delay[TX_DYNAMICS_LOOKAHEAD];
    unsigned position;
} tx_dynamics_t;

static inline void tx_dynamics_reset(tx_dynamics_t *s)
{
    s->rms_squared=0; s->reduction_db=0; s->limiter_gain=1; s->position=0;
    memset(s->delay,0,sizeof(s->delay));
}
static inline bool tx_dynamics_init(tx_dynamics_t *s, tx_dynamics_config_t c)
{
    if (!s || !tx_dynamics_valid(&c)) return false;
    memset(s,0,sizeof(*s)); s->config=c;
    s->input_gain=pow(10.,c.input_db/20.); s->makeup_gain=pow(10.,c.makeup_db/20.);
    s->ceiling=pow(10.,c.ceiling_db/20.);
    s->rms_coeff=exp(-1./(48000*.010));
    s->attack_coeff=exp(-1./(48*c.attack_ms));
    s->release_coeff=exp(-1./(48*c.release_ms));
    s->limiter_release_coeff=exp(-1./(48000*.050));
    tx_dynamics_reset(s); return true;
}
static inline void tx_dynamics_process(tx_dynamics_t *s, float *audio, size_t count)
{
    if (!s->config.enabled) return; /* exact bypass, including no added delay */
    for (size_t i=0;i<count;++i) {
        /* Invalid source samples become silence; bound pathological finite
         * inputs before squaring so detector state can never overflow. */
        double x=tx_dynamics_finite(audio[i]) ? audio[i]*s->input_gain : 0;
        x=fmax(-64.,fmin(64.,x));
        s->rms_squared=s->rms_coeff*s->rms_squared+(1-s->rms_coeff)*x*x;
        if (s->rms_squared<1e-20) s->rms_squared=0;
        double level=10*log10(fmax(s->rms_squared,1e-20));
        double over=level-s->config.threshold_db, knee=s->config.knee_db;
        double slope=1.-1./s->config.ratio, reduction;
        if (knee>0 && over>-knee/2 && over<knee/2)
            reduction=slope*(over+knee/2)*(over+knee/2)/(2*knee);
        else reduction=over>0 ? slope*over : 0;
        double coeff=reduction>s->reduction_db ? s->attack_coeff : s->release_coeff;
        s->reduction_db=coeff*s->reduction_db+(1-coeff)*reduction;
        float value=(float)(x*s->makeup_gain*pow(10.,-s->reduction_db/20.));
        float delayed=s->delay[s->position];
        s->delay[s->position]=value;
        s->position=(s->position+1)%TX_DYNAMICS_LOOKAHEAD;
        double peak=fabs(delayed);
        for (unsigned j=0;j<TX_DYNAMICS_LOOKAHEAD;++j) peak=fmax(peak,fabs(s->delay[j]));
        double target=peak>s->ceiling ? s->ceiling/peak : 1;
        /* Attack immediately with lookahead; release slowly, never past the
         * gain needed to bound every pending sample. */
        s->limiter_gain=fmin(target,s->limiter_release_coeff*s->limiter_gain+
                                  (1-s->limiter_release_coeff));
        audio[i]=(float)(delayed*s->limiter_gain);
    }
}
