#pragma once
#include "nbfm_mod.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shared audio-to-IQ handle; legacy NBFM entry points retain their behavior.
typedef nbfm_mod_t audio_mod_t;
typedef enum { AUDIO_MOD_NBFM=0, AUDIO_MOD_AM=1 } audio_mod_mode_t;
static inline const char* audio_mod_mode_name(audio_mod_mode_t mode)
{
    return mode==AUDIO_MOD_NBFM ? "nbfm" : mode==AUDIO_MOD_AM ? "am" : "unknown";
}
// AM: conventional carrier-present DSB, 300–3000 Hz voice, no pre-emphasis.
// out_scale is the maximum IQ envelope at 100% modulation; carrier=out_scale/2.
// depth is nominal modulation at unit audio (0..1); default chosen by caller.
// Samples are bounded before/after voice filtering to prevent overmodulation.
audio_mod_t* audio_mod_create(audio_mod_mode_t, const nbfm_cfg_t*, float depth);

#ifdef __cplusplus
}
#endif
