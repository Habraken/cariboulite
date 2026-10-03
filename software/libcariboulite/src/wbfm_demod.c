#include "fm_demod_internal.h"
#include "math_compat.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Symmetric Blackman-windowed FIRs. Mirrored histories avoid modulo in the
// convolution; filtering runs only at decimation output instants.
#define WBFM_RF_TAPS 641
#define WBFM_AUDIO_TAPS 401
#define WBFM_RESAMPLE_TAPS 32
#define WBFM_PHASES 256
typedef struct {
    int rf_taps, rf_pos, rf_count, audio_pos, audio_count;
    float rf_coeff[WBFM_RF_TAPS];
    float ri[2*WBFM_RF_TAPS], rq[2*WBFM_RF_TAPS];
    float audio_coeff[WBFM_AUDIO_TAPS], audio[2*WBFM_AUDIO_TAPS];
    float resample[2*WBFM_RESAMPLE_TAPS];
    float phases[WBFM_PHASES+1][WBFM_RESAMPLE_TAPS];
    int resample_pos;
    float pi, pq;
    int have_prev;
} wbfm_state_t;

typedef struct wb_state {
    nbfm_demod_t base;
    wbfm_state_t wide;
    nbfm_demod_config_t config;
    int D1;
    float K_norm, dc_a, lpf_a, lpf_b;
    float dc_y, x_prev_audio, deemph_state, lpf_y;
    double phase48;
} wb_state_t;

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

int wb_demod_set_audio(nbfm_demod_t* dsp, float tau, float gain)
{
    wb_state_t* s = (wb_state_t*)dsp;
    if (!s || !isfinite(tau) || tau < 0 || !isfinite(gain) || gain < 0)
        return -EINVAL;
    s->config.deemph_tau = tau;
    s->config.pcm_gain = gain;
    return 0;
}
static nbfm_demod_t* allocate_state(const nbfm_demod_config_t* config)
{
    if (!config || (config->rf_rate != 2000000 && config->rf_rate != 4000000) ||
        config->audio_rate != 48000 || !isfinite(config->deemph_tau) ||
        config->deemph_tau < 0 || !isfinite(config->pcm_gain) || config->pcm_gain < 0) {
        errno = EINVAL;
        return NULL;
    }
    wb_state_t* s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->base.mode = FM_MODE_WBFM;
    s->config = *config;
    s->dc_a = expf(-2.0f * (float)M_PI * 5.0f / 48000.0f);
    s->lpf_a = expf(-2.0f * (float)M_PI * 3200.0f / 48000.0f);
    s->lpf_b = 1.0f - s->lpf_a;
    return &s->base;
}
static void fir_design(float* h, int n, float cutoff, float fs)
{
    double sum = 0;
    for (int j = 0; j < n; ++j) {
        int k = j - n/2;
        double v = k ? sin(2*M_PI*cutoff*k/fs)/(M_PI*k) : 2*cutoff/fs;
        v *= 0.42 - 0.5*cos(2*M_PI*j/(n-1)) + 0.08*cos(4*M_PI*j/(n-1));
        h[j] = (float)v;
        sum += v;
    }
    for (int j = 0; j < n; ++j) h[j] /= (float)sum;
}

nbfm_demod_t* wb_demod_create(const nbfm_demod_config_t* config)
{
    wb_state_t* s = (wb_state_t*)allocate_state(config);
    if (!s) return NULL;
    s->D1 = config->rf_rate / 250000;
    s->wide.rf_taps = 40*s->D1 + 1;
    fir_design(s->wide.rf_coeff, s->wide.rf_taps, 100000, config->rf_rate);
    // Preserve mono audio through 15 kHz; reject the 19 kHz pilot and stereo
    // subchannel before reducing the discriminator output from 250 to 50 kHz.
    fir_design(s->wide.audio_coeff, WBFM_AUDIO_TAPS, 17000, 250000);
    // Fractional-delay low-pass bank for 50 -> 48 kHz clock-corrected output.
    // Linear interpolation alone introduces audible images near the top of
    // the broadcast audio band. Coefficients are prepared once, off the worker.
    for (int phase=0; phase<=WBFM_PHASES; ++phase) {
        double sum=0;
        for (int j=0; j<WBFM_RESAMPLE_TAPS; ++j) {
            double x=j-16.0+(double)phase/WBFM_PHASES;
            double v=fabs(x)<1e-10 ? 0.68 : sin(M_PI*0.68*x)/(M_PI*x);
            v *= 0.42-0.5*cos(2*M_PI*j/(WBFM_RESAMPLE_TAPS-1))
                     +0.08*cos(4*M_PI*j/(WBFM_RESAMPLE_TAPS-1));
            s->wide.phases[phase][j]=(float)v;
            sum+=v;
        }
        for (int j=0; j<WBFM_RESAMPLE_TAPS; ++j)
            s->wide.phases[phase][j]/=(float)sum;
    }
    s->K_norm = 250000.0f / (2.0f * (float)M_PI * 75000.0f);
    return &s->base;
}

static float fir_symmetric(const float* h, const float* x, int n)
{
    float y = h[n/2]*x[n/2];
    for (int j = 0; j < n/2; ++j) y += h[j]*(x[j] + x[n-1-j]);
    return y;
}

// Return one filtered discriminator sample at 50 kHz, as in the NBFM path.
static int wbfm_sample(wb_state_t* s, iq16_t sample, float* value)
{
    wbfm_state_t* w = &s->wide;
    int n = w->rf_taps, p = w->rf_pos;
    w->ri[p] = w->ri[p+n] = sample.i;
    w->rq[p] = w->rq[p+n] = sample.q;
    w->rf_pos = (p+1 == n) ? 0 : p+1;
    if (++w->rf_count != s->D1) return 0;
    w->rf_count = 0;
    float i = fir_symmetric(w->rf_coeff, w->ri+w->rf_pos, n);
    float q = fir_symmetric(w->rf_coeff, w->rq+w->rf_pos, n);
    float v = 0;
    if (w->have_prev)
        v = atan2f(q*w->pi-i*w->pq, i*w->pi+q*w->pq)*s->K_norm;
    w->pi = i; w->pq = q; w->have_prev = 1;
    p = w->audio_pos;
    w->audio[p] = w->audio[p+WBFM_AUDIO_TAPS] = v;
    w->audio_pos = (p+1 == WBFM_AUDIO_TAPS) ? 0 : p+1;
    if (++w->audio_count != 5) return 0;
    w->audio_count = 0;
    *value = fir_symmetric(w->audio_coeff, w->audio+w->audio_pos, WBFM_AUDIO_TAPS);
    return 1;
}

void wb_demod_reset(nbfm_demod_t* dsp)
{
    wb_state_t* s = (wb_state_t*)dsp;
    if (!s) return;
    {
        wbfm_state_t* w = &s->wide;
        w->rf_pos = w->rf_count = w->audio_pos = w->audio_count = 0;
        w->pi = w->pq = 0; w->have_prev = 0;
        memset(w->ri, 0, sizeof(w->ri));
        memset(w->rq, 0, sizeof(w->rq));
        memset(w->audio, 0, sizeof(w->audio));
        memset(w->resample, 0, sizeof(w->resample));
        w->resample_pos = 0;
    }
    s->dc_y = s->x_prev_audio = s->deemph_state = s->lpf_y = 0.0f;
    s->phase48 = 0.0;
}

nbfm_demod_result_t wb_demod_process_with_raw(nbfm_demod_t* dsp,
    const iq16_t* input, size_t count, int16_t* output, float* raw_audio, size_t capacity,
    double correction)
{
    wb_state_t* s = (wb_state_t*)dsp;
    nbfm_demod_result_t result = {0};
    if (!s || (!input && count) || (!output && capacity) ||
        !isfinite(correction) || fabs(correction) > 0.0005) {
        result.error = -EINVAL;
        return result;
    }
    for (size_t n = 0; n < count && result.produced < capacity; ++n) {
        ++result.consumed;
        float y50 = 0.0f;
        if (!wbfm_sample(s, input[n], &y50)) continue;

        {
            wbfm_state_t* w = &s->wide;
            int p=w->resample_pos;
            w->resample[p] = w->resample[p+WBFM_RESAMPLE_TAPS] = y50;
            w->resample_pos = (p+1 == WBFM_RESAMPLE_TAPS) ? 0 : p+1;
        }

        // Advance phase by "outputs per input" this step (r < 1)
        const double r = (48000.0 / 50000.0) * (1.0 + correction);
        s->phase48 += r;

        // If we crossed 1.0, emit exactly one 48k sample at that crossing
        if (s->phase48 >= 1.0) {
            const double frac = (s->phase48 - 1.0) / r;    // ∈ [0..1)
            float y_lin;
            {
                wbfm_state_t* w=&s->wide;
                // frac intervals before current input, with a 15-sample delay.
                double phase=frac*WBFM_PHASES;
                int p=(int)phase;
                if (p>=WBFM_PHASES) p=WBFM_PHASES-1;
                float blend=(float)(phase-p);
                const float* x=w->resample+w->resample_pos;
                y_lin=0;
                for (int j=0; j<WBFM_RESAMPLE_TAPS; ++j) {
                    float h=w->phases[p][j]+blend*(w->phases[p+1][j]-w->phases[p][j]);
                    y_lin+=h*x[j];
                }
            }

            if (raw_audio) raw_audio[result.produced] = y_lin;

            // === 48k audio chain ===
            float x = y_lin;
            float y = (x - s->x_prev_audio) + s->dc_a * s->dc_y;
            s->x_prev_audio = x;
            s->dc_y = y; if (fabsf(s->dc_y) < 1e-20f) s->dc_y = 0.0f;

            float yd = deemph_48k(y, &s->deemph_state, s->config.deemph_tau);
            if (fabsf(s->deemph_state) < 1e-20f) s->deemph_state = 0.0f;

            // Retain legacy audio-chain evaluation order for this extraction.
            s->lpf_y = s->lpf_a * s->lpf_y + s->lpf_b * yd;
            float ya = yd;
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

