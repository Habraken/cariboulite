#pragma once
#include "nbfm_mod.h"
#include "ctcss.h"
#include "cdcss.h"
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared audio-to-IQ handle; legacy NBFM entry points retain their behavior.
typedef nbfm_mod_t audio_mod_t;
typedef enum {
    AUDIO_MOD_NBFM = 0,
    AUDIO_MOD_AM,
    AUDIO_MOD_USB,
    AUDIO_MOD_LSB,
} audio_mod_mode_t;

static inline bool audio_mod_mode_valid(audio_mod_mode_t mode)
{
    return mode >= AUDIO_MOD_NBFM && mode <= AUDIO_MOD_LSB;
}

static inline const char* audio_mod_mode_name(audio_mod_mode_t mode)
{
    switch (mode) {
    case AUDIO_MOD_NBFM: return "nbfm";
    case AUDIO_MOD_AM: return "am";
    case AUDIO_MOD_USB: return "usb";
    case AUDIO_MOD_LSB: return "lsb";
    }
    return "unknown";
}

static inline bool audio_mod_mode_parse(const char* name, audio_mod_mode_t* mode)
{
    for (int candidate = AUDIO_MOD_NBFM; candidate <= AUDIO_MOD_LSB; ++candidate) {
        if (strcmp(name, audio_mod_mode_name(candidate)) == 0) {
            *mode = candidate;
            return true;
        }
    }
    return false;
}

// USB/LSB: suppressed-carrier SSB, 300–3000 Hz voice, no pre-emphasis.
// AM: conventional carrier-present DSB, 300–3000 Hz voice, no pre-emphasis.
// out_scale is the maximum IQ envelope at 100% modulation; carrier=out_scale/2.
// depth is nominal modulation at unit audio (0..1); default chosen by caller.
// Samples are bounded before/after voice filtering to prevent overmodulation.
audio_mod_t* audio_mod_create(audio_mod_mode_t, const nbfm_cfg_t*, float depth);
// AM only. Configure while stopped, before processing; reset retains selection.
// Experimental 40 kHz DAC error feedback, limited to out_scale <= 4032.
int audio_mod_set_am_dac_shaping(audio_mod_t*, bool enabled);
// Configure while stopped. Only NBFM emits CTCSS; other modes retain bypass.
int audio_mod_set_ctcss(audio_mod_t*, float frequency_hz, float deviation_hz);
// CDCSS and CTCSS are exclusive. Configure while stopped; only NBFM emits CDCSS.
int audio_mod_set_cdcss(audio_mod_t*, cdcss_config_t, float deviation_hz);
void audio_mod_reset_cdcss(audio_mod_t*);
void audio_mod_cdcss_tail(audio_mod_t*, bool tail);
// Single modulator owner only: retained starts reset CTCSS; the stop cue/tail mutes it.
void audio_mod_reset_ctcss(audio_mod_t*);
void audio_mod_mute_ctcss(audio_mod_t*, bool muted);

#ifdef __cplusplus
}
#endif
