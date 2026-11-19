#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "iq16.h"
#include "audio_format.h"
#include "nbfm_defaults.h"

// Standalone DSP. No threads, device handles, queues or retained caller buffers.
// NBFM audio defaults to NBFM_DEFAULT_DEVIATION_HZ normalization.
typedef struct nbfm_demod audio_demod_t;
typedef enum {
    AUDIO_DEMOD_NBFM = 0, AUDIO_DEMOD_WBFM, AUDIO_DEMOD_AM,
    AUDIO_DEMOD_USB, AUDIO_DEMOD_LSB
} audio_demod_mode_t;
static inline bool audio_demod_is_ssb(audio_demod_mode_t mode)
{
    return mode == AUDIO_DEMOD_USB || mode == AUDIO_DEMOD_LSB;
}
static inline bool audio_demod_uses_offset(audio_demod_mode_t mode, bool am_enabled)
{
    return audio_demod_is_ssb(mode) || (mode == AUDIO_DEMOD_AM && am_enabled);
}
static inline const char* audio_demod_mode_name(audio_demod_mode_t mode)
{
    switch (mode) {
    case AUDIO_DEMOD_NBFM: return "nbfm";
    case AUDIO_DEMOD_WBFM: return "wbfm";
    case AUDIO_DEMOD_AM: return "am";
    case AUDIO_DEMOD_USB: return "usb";
    case AUDIO_DEMOD_LSB: return "lsb";
    default: return "unknown";
    }
}
static inline const char* audio_demod_mode_label(audio_demod_mode_t mode)
{
    switch (mode) {
    case AUDIO_DEMOD_NBFM: return "NBFM";
    case AUDIO_DEMOD_WBFM: return "WBFM mono";
    case AUDIO_DEMOD_AM: return "AM voice";
    case AUDIO_DEMOD_USB: return "USB";
    case AUDIO_DEMOD_LSB: return "LSB";
    default: return "Unknown";
    }
}
static inline bool audio_demod_mode_parse(const char* name, audio_demod_mode_t* mode)
{
    for (int candidate = AUDIO_DEMOD_NBFM; candidate <= AUDIO_DEMOD_LSB; ++candidate) {
        audio_demod_mode_t value = (audio_demod_mode_t)candidate;
        if (strcmp(name, audio_demod_mode_name(value)) == 0) {
            *mode = value;
            return true;
        }
    }
    return false;
}
typedef struct {
    unsigned rf_rate;       // native 4 MHz/divider rate label; see radio_rates.h
    unsigned audio_rate;    // exactly 48000 Hz
    float deemph_tau;       // seconds; zero bypasses de-emphasis
    float pcm_gain;         // scale filtered audio to signed-16-bit PCM
} audio_demod_config_t;
typedef struct {
    size_t consumed;        // IQ pairs consumed
    size_t produced;        // mono PCM samples written
    int error;             // 0 or negative errno-style code
} audio_demod_result_t;

// Unsupported modes fail with errno=EINVAL; the selected mode is immutable.
audio_demod_t* audio_demod_create(audio_demod_mode_t mode, const audio_demod_config_t* config);
// Zero for null/unsupported mode. Noise metric means the NBFM raw-tap contract.
#define AUDIO_DEMOD_CAP_NOISE_SQUELCH 1u
unsigned audio_demod_mode_capabilities(audio_demod_mode_t mode);
unsigned audio_demod_capabilities(const audio_demod_t* dsp);
// All modes clear all signal history, decimator phases, filters and resampling.
void audio_demod_reset(audio_demod_t* dsp);
// Update audio controls without resetting filter history.
int audio_demod_set_audio(audio_demod_t* dsp, float deemph_tau, float pcm_gain);
// NBFM only: set expected peak deviation for audio normalization, in Hz
// (finite 1..24000). Channel filters and the fixed raw/noise-squelch scale are
// unchanged. Reset retains the setting. Single owner; invalid values or other
// modes return -EINVAL without changing the DSP.
int audio_demod_set_nbfm_deviation(audio_demod_t* dsp, float deviation_hz);
// correction is fractional output-rate adjustment in [-0.0005, 0.0005].
// No allocations. Stops at input end or output capacity; retry unused input.
// Zero capacity consumes nothing; no hidden pending output. Single owner.
audio_demod_result_t audio_demod_process(audio_demod_t* dsp,
    const iq16_t* input, size_t count, audio_s16_t* output, size_t capacity,
    double correction);
// Optional post-filter, pre-gain floating-point tap, separate from raw metrics.
// The returned samples match PCM output counts; buffer is never retained.
audio_demod_result_t audio_demod_process_with_taps(audio_demod_t* dsp,
    const iq16_t* input, size_t count, audio_s16_t* output, float* raw_audio,
    float* filtered_audio, size_t capacity, double correction);
void audio_demod_destroy(audio_demod_t* dsp);
// AM only: slow carrier normalization (default on). Off uses IQ/4096 scaling;
// pcm_gain remains the manual audio gain. Settings survive reset.
int audio_demod_set_am_agc(audio_demod_t* dsp, bool enabled);
// AM only, configure before processing: digital shift 0 or +/-12500 Hz.
// Reset retains the shift and restarts its exact native-clock phase.
int audio_demod_set_am_offset(audio_demod_t* dsp, int hz);
// SSB only, configure before processing. Offset is in decoded IQ coordinates;
// inverted indicates that positive decoded IQ frequency represents lower RF.
int audio_demod_set_ssb_tuning(audio_demod_t* dsp, int hz, bool inverted);

// Optional parallel 48 kHz discriminator tap, before DC/de-emphasis/low-pass
// and PCM gain in NBFM; WBFM taps follow mono/anti-alias filtering.
// NBFM raw values retain NBFM_DEFAULT_DEVIATION_HZ normalization regardless
// of the configured receive audio deviation.
// raw_audio has capacity floats and receives exactly produced
// samples; NULL skips the tap. Same progress/lifetime rules as process().
audio_demod_result_t audio_demod_process_with_raw(audio_demod_t* dsp,
    const iq16_t* input, size_t count, audio_s16_t* output, float* raw_audio,
    size_t capacity, double correction);

// AM raw tap is the filtered 300–3000 Hz voice signal before PCM gain;
// it does not implement the NBFM noise-RMS or SINAD contract.
