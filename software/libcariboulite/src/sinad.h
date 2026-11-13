#pragma once
/* Streaming 48 kHz NBFM SINAD estimator. Independent measurement branch:
 * Butterworth HP 300 Hz -> Butterworth LP 3000 Hz -> 1000 Hz notch (Q=30).
 * The notch removes some noise too: this is a link-quality estimate, not a
 * calibrated standards-compliant audio analyzer. No allocation or sample FFT.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct { double b0,b1,b2,a1,a2,z1,z2; } sinad_biquad_t;
typedef struct {
    sinad_biquad_t hp, lp, notch;
    double total, residual;
    unsigned warmup, count;
    bool valid;
    double db;
} sinad_t;

static inline void sinad_filter_init(sinad_biquad_t *b, double hz, double q, int kind)
{
    double w = 6.283185307179586 * hz / 48000.0;
    double c = cos(w), a = sin(w)/(2*q), d = 1+a;
    b->b0 = (kind == 0 ? (1+c)/2 : kind == 1 ? (1-c)/2 : 1)/d;
    b->b1 = (kind == 0 ? -(1+c) : kind == 1 ? 1-c : -2*c)/d;
    b->b2 = b->b0;
    b->a1 = -2*c/d; b->a2 = (1-a)/d;
}
static inline double sinad_filter(sinad_biquad_t *b, double x)
{
    double y = b->b0*x+b->z1;
    b->z1 = b->b1*x-b->a1*y+b->z2;
    b->z2 = b->b2*x-b->a2*y;
    return y;
}
static inline void sinad_reset(sinad_t *s)
{
    memset(s, 0, sizeof(*s));
    sinad_filter_init(&s->hp, 300, 0.7071067811865476, 0);
    sinad_filter_init(&s->lp, 3000, 0.7071067811865476, 1);
    sinad_filter_init(&s->notch, 1000, 30, 2);
    s->warmup = 9600; /* 200 ms settling; first result after 700 ms. */
}
/* true when a new 500 ms result is ready. Non-finite input resets closed.
 * Use bit validation because production builds enable -ffast-math. */
static inline bool sinad_process(sinad_t *s, float sample)
{
    uint32_t bits; memcpy(&bits, &sample, sizeof(bits));
    if ((bits & 0x7f800000u) == 0x7f800000u) {
        sinad_reset(s); return true;
    }
    double x = sinad_filter(&s->lp, sinad_filter(&s->hp, sample));
    double r = sinad_filter(&s->notch, x);
    if (s->warmup) { --s->warmup; return false; }
    s->total += x*x; s->residual += r*r;
    if (++s->count < 24000) return false;
    /* Suppress silence and readings without a detectable fundamental.
     * The 1 dB criterion is a reporting threshold, not a carrier detector. */
    s->valid = s->total/24000 > 1e-12 && s->residual < s->total/1.2589254117941673;
    s->db = s->valid ? fmin(80.0, 10*log10(s->total/fmax(s->residual, s->total*1e-8))) : 0;
    s->total = s->residual = 0; s->count = 0;
    return true;
}
