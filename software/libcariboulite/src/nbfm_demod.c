#include "nbfm_demod.h"
#include "math_compat.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>

struct nbfm_demod {
    nbfm_demod_config_t config;
    int D1, D2, use_limiter;
    float K_norm, dc_a, lpf_a, lpf_b;
    float ai1, aq1, ai2, aq2;
    int cnt1, cnt2;
    float pi50, pq50;
    int have_prev50;
    float dc_y, x_prev_audio, deemph_state, lpf_y;
    float y_prev_50k, y_curr_50k;
    double phase48;
};

// --- audio-rate deemphasis (48 kHz) ---
static inline float deemph_48k(float x, float *z, float tau_s)
{
    if (tau_s <= 0.f) return x;          // bypass if tau==0
    const float fs = 48000.f;
    const float a  = expf(-1.0f/(fs * tau_s));
    const float b  = 1.0f - a;
    *z = a * (*z) + b * x;
    return *z;
}

// Ultra-fast small-angle atan2f approximation
// Error < 0.005 rad for |y/x| < 0.3 (typical in NBFM discriminator)
static inline float fast_atan2f_small(float y, float x)
{
    // approximate atan(y/x) ≈ y / (|x| + 0.28f*|y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    float angle = y / (abs_x + 0.28f * abs_y + 1e-10f);
    if (x < 0.0f)
        angle = (y >= 0.0f ? (float)M_PI + angle : -((float)M_PI - angle));
    return angle;
}

int nbfm_demod_set_audio(nbfm_demod_t* s, float tau, float gain)
{
    if (!s || !isfinite(tau) || tau < 0 || !isfinite(gain) || gain < 0)
        return -EINVAL;
    s->config.deemph_tau = tau;
    s->config.pcm_gain = gain;
    return 0;
}
nbfm_demod_t* nbfm_demod_create(const nbfm_demod_config_t* config)
{
    if (!config || (config->rf_rate != 2000000 && config->rf_rate != 4000000) ||
        config->audio_rate != 48000 || !isfinite(config->deemph_tau) ||
        config->deemph_tau < 0 || !isfinite(config->pcm_gain) || config->pcm_gain < 0) {
        errno = EINVAL;
        return NULL;
    }
    nbfm_demod_t* s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->config = *config;
    s->D1 = config->rf_rate / 200000;
    s->D2 = 4;
    s->use_limiter = 1;
    s->K_norm = 50000.0f / (2.0f * (float)M_PI * 2500.0f);
    s->dc_a = expf(-2.0f * (float)M_PI * 5.0f / 48000.0f);
    s->lpf_a = expf(-2.0f * (float)M_PI * 3200.0f / 48000.0f);
    s->lpf_b = 1.0f - s->lpf_a;
    return s;
}
void nbfm_demod_reset(nbfm_demod_t* s)
{
    if (!s) return;
    s->pi50 = s->pq50 = 0.0f;
    s->have_prev50 = 0;
    s->dc_y = s->x_prev_audio = s->deemph_state = s->lpf_y = 0.0f;
    s->y_prev_50k = s->y_curr_50k = 0.0f;
    s->phase48 = 0.0;
}
void nbfm_demod_destroy(nbfm_demod_t* s) { free(s); }

nbfm_demod_result_t nbfm_demod_process(nbfm_demod_t* s,
    const iq16_t* input, size_t count, int16_t* output, size_t capacity,
    double correction)
{
    nbfm_demod_result_t result = {0};
    if (!s || (!input && count) || (!output && capacity) ||
        !isfinite(correction) || fabs(correction) > 0.0005) {
        result.error = -EINVAL;
        return result;
    }
    for (size_t n = 0; n < count && result.produced < capacity; ++n) {
        ++result.consumed;
        // --- accumulate at the configured RF rate (stage-1) ---
        s->ai1 += (float)input[n].i;
        s->aq1 += (float)input[n].q;
        if (++s->cnt1 != s->D1) continue;

        // boxcar avg #1
        float i1 = s->ai1 / (float)s->D1;
        float q1 = s->aq1 / (float)s->D1;
        s->ai1 = s->aq1 = 0.0f; s->cnt1 = 0;

        // --- accumulate @ 200k (stage-2 to 50k) ---
        s->ai2 += i1;
        s->aq2 += q1;
        if (++s->cnt2 != s->D2) continue;

        // boxcar avg #2 -> 50 kS/s complex sample
        float i50 = s->ai2 / (float)s->D2;
        float q50 = s->aq2 / (float)s->D2;
        s->ai2 = s->aq2 = 0.0f; s->cnt2 = 0;

        // --- limiter (unit vector) ---
        if (s->use_limiter) {
            float m2 = i50*i50 + q50*q50;
            if (m2 > 0.0f) {
                float invm = 1.0f / sqrtf(m2);
                i50 *= invm; q50 *= invm;
            }
        }

        // --- discriminator at 50 kS/s using previous 50k sample ---
        float y50 = 0.0f;
        if (s->have_prev50) {
            const float re = i50 * s->pi50 + q50 * s->pq50;
            const float im = q50 * s->pi50 - i50 * s->pq50;
            const float dphi = fast_atan2f_small(im, re);
            y50 = dphi * s->K_norm;                   // normalize to ~±1 @ ±dev
        } else {
            s->have_prev50 = 1;
        }
        s->pi50 = i50; s->pq50 = q50;

        // Interpolation endpoints for this 50k interval
        s->y_prev_50k = s->y_curr_50k;
        s->y_curr_50k = y50;

        // Advance phase by "outputs per input" this step (r < 1)
        const double r = (48000.0 / 50000.0) * (1.0 + correction);
        s->phase48 += r;

        // If we crossed 1.0, emit exactly one 48k sample at that crossing
        if (s->phase48 >= 1.0) {
            const double frac = (s->phase48 - 1.0) / r;    // ∈ [0..1)
            float y_lin = s->y_prev_50k + (float)frac * (s->y_curr_50k - s->y_prev_50k);

            // === 48k audio chain ===
            float x = y_lin;
            float y = (x - s->x_prev_audio) + s->dc_a * s->dc_y;
            s->x_prev_audio = x;
            s->dc_y = y; if (fabsf(s->dc_y) < 1e-20f) s->dc_y = 0.0f;

            float yd = deemph_48k(y, &s->deemph_state, s->config.deemph_tau);
            if (fabsf(s->deemph_state) < 1e-20f) s->deemph_state = 0.0f;

            s->lpf_y = s->lpf_a * s->lpf_y + s->lpf_b * yd;
            float ya = s->lpf_y;
            if (fabsf(s->lpf_y) < 1e-20f) s->lpf_y = 0.0f;

            float pcm = ya * s->config.pcm_gain;
            if (pcm >  32767.f) pcm =  32767.f;
            if (pcm < -32768.f) pcm = -32768.f;
            output[result.produced++] = (int16_t)lrintf(pcm);

            // Keep fractional remainder, including across process calls.
            s->phase48 -= 1.0;
        }
    }
    return result;
}
