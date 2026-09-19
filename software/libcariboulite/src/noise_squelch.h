#pragma once
#include <stdbool.h>
// Single-owner, allocation-free detector for unfiltered 48 kHz discriminator
// audio, normalized to nominal +/-1 at +/-2.5 kHz deviation. No PCM gain.
// Two cascaded 6 kHz high-pass biquads, 20 ms power averaging.
#define NOISE_SQUELCH_OPEN_RMS 0.12f
#define NOISE_SQUELCH_CLOSE_RMS 0.18f
typedef struct {
    float z1[2], z2[2], power;
    unsigned qualify;
    bool open;
} noise_squelch_t;
void noise_squelch_reset(noise_squelch_t* s);
// Initially closed; 30 ms quiet opens, 120 ms noisy closes; hysteresis.
// Invalid samples reset closed. Detector observes audio even while muted.
bool noise_squelch_process(noise_squelch_t* s, float audio);
