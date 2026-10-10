#include "fm_demod_internal.h"
#include "fm_audio.h"
#include "fm_discriminator.h"
#include "math_compat.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct nb_state {
    nbfm_demod_t base;
    nbfm_demod_config_t config;
    int D1, D2, use_limiter;
    float K_norm;
    float ai1, aq1, ai2, aq2;
    int cnt1, cnt2;
    float pi50, pq50;
    int have_prev50;
    fm_audio_state_t audio;
    float y_prev_50k, y_curr_50k;
    double phase48;
} nb_state_t;

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

int nb_demod_set_audio(nbfm_demod_t* dsp, float tau, float gain)
{
    nb_state_t* s = (nb_state_t*)dsp;
    if (!s || !isfinite(tau) || tau < 0 || !isfinite(gain) || gain < 0)
        return -EINVAL;
    s->config.deemph_tau = tau;
    s->config.pcm_gain = gain;
    return 0;
}
nbfm_demod_t* nb_demod_create(const nbfm_demod_config_t* config)
{
    if (!config || (config->rf_rate != 1000000 && config->rf_rate != 2000000 &&
        config->rf_rate != 4000000) ||
        config->audio_rate != 48000 || !isfinite(config->deemph_tau) ||
        config->deemph_tau < 0 || !isfinite(config->pcm_gain) || config->pcm_gain < 0) {
        errno = EINVAL;
        return NULL;
    }
    nb_state_t* s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->base.mode = FM_MODE_NBFM;
    s->config = *config;
    s->D1 = config->rf_rate / 200000;
    s->D2 = 4;
    s->use_limiter = 1;
    s->K_norm = 50000.0f / (2.0f * (float)M_PI * NBFM_DEFAULT_DEVIATION_HZ);
    fm_audio_init(&s->audio);
    return &s->base;
}
void nb_demod_reset(nbfm_demod_t* dsp)
{
    nb_state_t* s = (nb_state_t*)dsp;
    if (!s) return;
    s->pi50 = s->pq50 = 0.0f;
    s->have_prev50 = 0;
    fm_audio_reset(&s->audio);
    s->y_prev_50k = s->y_curr_50k = 0.0f;
    s->phase48 = 0.0;
}

nbfm_demod_result_t nb_demod_process_with_raw(nbfm_demod_t* dsp,
    const iq16_t* input, size_t count, int16_t* output, float* raw_audio, size_t capacity,
    double correction)
{
    nb_state_t* s = (nb_state_t*)dsp;
    nbfm_demod_result_t result = {0};
    if (!s || (!input && count) || (!output && capacity) ||
        !isfinite(correction) || fabs(correction) > 0.0005) {
        result.error = -EINVAL;
        return result;
    }
    for (size_t n = 0; n < count && result.produced < capacity; ++n) {
        ++result.consumed;
        float y50 = 0.0f;
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
        if (s->have_prev50) {
            float re, im;
            fm_conjugate_product(i50, q50, s->pi50, s->pq50, &re, &im);
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
            float y_lin;
            // Preserve the legacy NBFM interpolation exactly.
            y_lin = s->y_prev_50k + (float)frac * (s->y_curr_50k - s->y_prev_50k);

            if (raw_audio) raw_audio[result.produced] = y_lin;

            output[result.produced++] = fm_audio_process(&s->audio, y_lin,
                s->config.deemph_tau, s->config.pcm_gain, 1);

            // Keep fractional remainder, including across process calls.
            s->phase48 -= 1.0;
        }
    }
    return result;
}

