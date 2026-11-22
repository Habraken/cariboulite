#pragma once
/* CTCSS DSP uses 48 kHz mono discriminator audio. No allocation or locks.
 * A zero frequency bypasses processing; settings are immutable while running. */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

static inline bool ctcss_finite(float x)
{
    uint32_t bits; memcpy(&bits, &x, sizeof(bits));
    /* Preserve bit classification in inlined fast-math callers too. */
    volatile uint32_t checked=bits;
    return (checked & 0x7f800000u) != 0x7f800000u;
}
static inline bool ctcss_frequency_valid(float hz)
{ return ctcss_finite(hz) && (hz == 0 || (hz >= 67 && hz <= 254.1f)); }
static inline bool ctcss_deviation_valid(float hz)
{ return ctcss_finite(hz) && hz >= 50 && hz <= 1000; }

/* Fourth-order Butterworth 300 Hz high-pass for voice only. */
typedef struct { double x1[2], x2[2], y1[2], y2[2]; } ctcss_voice_t;
static inline float ctcss_voice_sample(ctcss_voice_t *s, float x)
{
    const double q[2] = {0.541196100146197, 1.306562964876377};
    const double w = 6.2831853071795864769 * 300 / 48000;
    for (unsigned k=0; k<2; ++k) {
        double a=sin(w)/(2*q[k]), d=1+a, b=(1+cos(w))/(2*d);
        double y=b*(x-2*s->x1[k]+s->x2[k])+2*cos(w)/d*s->y1[k]-(1-a)/d*s->y2[k];
        s->x2[k]=s->x1[k]; s->x1[k]=x;
        s->y2[k]=s->y1[k]; s->y1[k]=y; x=(float)y;
    }
    return x;
}

/* Playback-only notch removes the selected tone, including high CTCSS tones
 * near the voice high-pass corner. Reset together with the RX detector. */
typedef struct { double b0,b1,b2,a1,a2,z1,z2; } ctcss_notch_t;
static inline void ctcss_notch_reset(ctcss_notch_t *s,float hz)
{
    double w=6.2831853071795864769*hz/48000, a=sin(w)/16, d=1+a;
    *s=(ctcss_notch_t){.b0=1/d,.b1=-2*cos(w)/d,.b2=1/d,.a1=-2*cos(w)/d,.a2=(1-a)/d};
}
static inline float ctcss_notch_sample(ctcss_notch_t *s,float x)
{
    double y=s->b0*x+s->z1;
    s->z1=s->b1*x-s->a1*y+s->z2; s->z2=s->b2*x-s->a2*y;
    return (float)y;
}

/* Coherent 400 ms rectangular window, evaluated every 100 ms at 1 kHz.
 * Two consecutive hits acquire; two misses release. The energy ratio rejects
 * adjacent tones and speech. A 4-pole 300 Hz LPF precedes decimation; a DC
 * blocker rejects discriminator offset. No dependence on RX gain/volume. */
enum { CTCSS_WINDOW=400, CTCSS_HOP=100 };
typedef struct {
    float frequency_hz;
    double phase, lp[4], dc, sum_i, sum_q, energy;
    double history_i[CTCSS_WINDOW], history_q[CTCSS_WINDOW], history_e[CTCSS_WINDOW];
    unsigned decimation, head, count, hop, hits, misses;
    bool valid, open;
    float coherence, peak;
} ctcss_rx_t;
static inline void ctcss_rx_reset(ctcss_rx_t *s, float hz)
{ memset(s,0,sizeof(*s)); s->frequency_hz=hz; }
static inline bool ctcss_rx_sample(ctcss_rx_t *s, float input)
{
    if (!s->frequency_hz) return true;
    if (!ctcss_finite(input)) { ctcss_rx_reset(s,s->frequency_hz); return false; }
    /* Identical low-pass poles give steep rejection before 48:1 decimation. */
    double x=input;
    const double a=1-exp(-6.2831853071795864769*300/48000);
    for (unsigned k=0;k<4;k++) { s->lp[k]+=a*(x-s->lp[k]); x=s->lp[k]; }
    if (++s->decimation<48) return s->open;
    s->decimation=0;
    s->dc+=(1-exp(-6.2831853071795864769*20/1000))*(x-s->dc);
    x-=s->dc;
    double i=x*cos(s->phase), q=x*sin(s->phase), e=x*x;
    s->phase+=6.2831853071795864769*s->frequency_hz/1000;
    if (s->phase>=6.2831853071795864769) s->phase-=6.2831853071795864769;
    unsigned h=s->head;
    s->sum_i+=i-s->history_i[h]; s->sum_q+=q-s->history_q[h];
    s->energy+=e-s->history_e[h];
    s->history_i[h]=i; s->history_q[h]=q; s->history_e[h]=e;
    s->head=(h+1)%CTCSS_WINDOW;
    if (s->count<CTCSS_WINDOW) ++s->count;
    if (++s->hop<CTCSS_HOP) return s->open;
    s->hop=0;
    if (s->count<CTCSS_WINDOW) return false;
    s->valid=true;
    double tone=2*(s->sum_i*s->sum_i+s->sum_q*s->sum_q)/CTCSS_WINDOW;
    s->coherence=(float)(s->energy>1e-15 ? fmax(0,fmin(1,tone/s->energy)) : 0);
    s->peak=(float)sqrt(fmax(0,2*tone/CTCSS_WINDOW));
    bool hit=s->coherence>=(s->open ? .50f : .70f) && s->peak>=.008f;
    if (hit) { s->misses=0; if (++s->hits>=2) { s->open=true; s->hits=2; } }
    else { s->hits=0; if (++s->misses>=2) { s->open=false; s->misses=2; } }
    return s->open;
}
