#pragma once
#include <stddef.h>
#include "iq16.h"
#include "audio_format.h"

// Standalone DSP. No threads, device handles, queues or retained caller buffers.
typedef struct nbfm_demod nbfm_demod_t;
typedef struct {
    unsigned rf_rate;       // exactly 2000000 or 4000000 Hz
    unsigned audio_rate;    // exactly 48000 Hz
    float deemph_tau;       // seconds; zero bypasses de-emphasis
    float pcm_gain;         // scale filtered audio to signed-16-bit PCM
} nbfm_demod_config_t;
typedef struct {
    size_t consumed;        // IQ pairs consumed
    size_t produced;        // mono PCM samples written
    int error;             // 0 or negative errno-style code
} nbfm_demod_result_t;

nbfm_demod_t* nbfm_demod_create(const nbfm_demod_config_t* config);
// Preserve I&D accumulators, as in the previous worker reset; reset the
// discriminator, audio filters and fractional resampler. Recreate for cold reset.
void nbfm_demod_reset(nbfm_demod_t* dsp);
// Update audio controls without resetting filter history.
int nbfm_demod_set_audio(nbfm_demod_t* dsp, float deemph_tau, float pcm_gain);
// correction is fractional output-rate adjustment in [-0.0005, 0.0005].
// No allocations. Stops at input end or output capacity; retry unused input.
// Zero capacity consumes nothing; no hidden pending output. Single owner.
nbfm_demod_result_t nbfm_demod_process(nbfm_demod_t* dsp,
    const iq16_t* input, size_t count, audio_s16_t* output, size_t capacity,
    double correction);
void nbfm_demod_destroy(nbfm_demod_t* dsp);

// Optional parallel 48 kHz discriminator tap, before DC/de-emphasis/low-pass
// and PCM gain. raw_audio has capacity floats and receives exactly produced
// samples; NULL skips the tap. Same progress/lifetime rules as process().
nbfm_demod_result_t nbfm_demod_process_with_raw(nbfm_demod_t* dsp,
    const iq16_t* input, size_t count, audio_s16_t* output, float* raw_audio,
    size_t capacity, double correction);
