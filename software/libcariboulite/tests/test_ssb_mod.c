#include "audio_mod.h"
#include "math_compat.h"
#include "radio_rates.h"
#include "tx_pipeline.h"

#include <assert.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double check_sideband(audio_mod_mode_t mode, unsigned rate, unsigned tone)
{
    const unsigned divider = radio_rate_divider(rate);
    const double fs = 4000000.0 / divider;
    const nbfm_cfg_t config = {48000, rate, 2500, 0, 4000, 1};
    audio_mod_t* modem = audio_mod_create(mode, &config, 0);
    audio_mod_t* split = audio_mod_create(mode, &config, 1);
    iq16_t* iq = malloc(40000 * sizeof(*iq));
    iq16_t* other = malloc(40000 * sizeof(*other));
    rf10_frame_t* frame = malloc(sizeof(*frame));
    assert(modem && split && iq && other && frame);
    assert(audio_mod_set_am_dac_shaping(modem, true) == -1);
    float audio[480];
    tx_writer_ctrl_st tx = {.fm = modem, .a48k = audio, .iq_rf = iq,
                            .rf_rate_hz = rate, .frame_samples = (40000 + divider - 1) / divider};
    double complex carrier = 0, wanted = 0, image = 0;
    unsigned samples = 0, measured = 0;
    const int sign = mode == AUDIO_MOD_USB ? 1 : -1;

    for (unsigned block = 0; block < 30; ++block) {
        for (unsigned j = 0; j < 480; ++j)
            audio[j] = .5 * sin(2 * M_PI * tone * (480 * block + j) / 48000);
        tx.conjugate_iq = block % 2 != 0;
        assert(tx_generate_frame(&tx, frame) == 0);
        nbfm_result_t r = nbfm_process(split, audio, 213, NULL, 0);
        assert(!r.error && r.consumed == 213);
        unsigned count = frame->sample_count;
        r = nbfm_process(split, audio + 213, 267, other, count / 3);
        assert(!r.error && !r.held_audio && r.consumed == 267 && r.produced == count / 3);
        r = nbfm_process(split, NULL, 0, other + count / 3, count - count / 3);
        assert(!r.error && !r.held_audio && r.produced == count - count / 3);
        assert(memcmp(other, iq, count * sizeof(*iq)) == 0);

        for (unsigned j = 0; j < count; ++j, ++samples) {
            assert(frame->data[j].i == iq[j].i);
            assert(frame->data[j].q == (tx.conjugate_iq ? -iq[j].q : iq[j].q));
            if (block < 6) continue;
            double angle = sign * 2 * M_PI * tone * samples / fs;
            double complex oscillator = cos(angle) + I * sin(angle);
            double complex sample = iq[j].i + I * iq[j].q;
            wanted += sample * conj(oscillator);
            image += sample * oscillator;
            carrier += sample;
            ++measured;
        }
    }
    assert(samples == 1200000 / divider);
    assert(cabs(wanted) / measured > 1300);
    assert(cabs(carrier) < cabs(wanted) / 1000);
    double suppression = 20 * log10(cabs(wanted) / fmax(1, cabs(image)));
    assert(suppression > 45);

    nbfm_result_t r = nbfm_process(modem, NULL, 0, iq, 1000);
    assert(!r.error && r.held_audio);
    for (unsigned j = 500; j < 1000; ++j)
        assert(iq[j].i == 0 && iq[j].q == 0);

    nbfm_reset(modem);
    nbfm_reset(split);
    memset(audio, 0, sizeof(audio));
    r = nbfm_process(modem, audio, 480, iq, 40000 / divider);
    assert(!r.error && !r.held_audio);
    for (size_t j = 0; j < r.produced; ++j)
        assert(iq[j].i == 0 && iq[j].q == 0);

    // Refill after reset and overload the input with a square wave.
    for (unsigned j = 0; j < 480; ++j)
        audio[j] = (j / 24) % 2 ? 4 : -4;
    nbfm_reset(modem);
    r = nbfm_process(modem, audio, 480, iq, 40000 / divider);
    assert(!r.error && !r.held_audio);
    nbfm_result_t reference = nbfm_process(split, audio, 480, other, r.produced);
    assert(!reference.error && !reference.held_audio && reference.produced == r.produced);
    assert(!memcmp(iq, other, r.produced * sizeof(*iq)));
    for (size_t j = 0; j < r.produced; ++j)
        assert(hypot(iq[j].i, iq[j].q) <= 4001);

    printf("%s %u S/s %u Hz: image rejection %.1f dB\n",
           audio_mod_mode_name(mode), rate, tone, suppression);
    nbfm_destroy(modem);
    nbfm_destroy(split);
    free(iq);
    free(other);
    free(frame);
    return suppression;
}

int main(void)
{
    const unsigned rates[] = {4000000, 2000000, 1333333, 1000000,
                              800000, 666667, 500000, 400000};
    const unsigned tones[] = {300, 1000, 3000};
    double minimum = INFINITY;
    for (audio_mod_mode_t mode = AUDIO_MOD_USB; mode <= AUDIO_MOD_LSB; ++mode)
        for (unsigned rate = 0; rate < sizeof(rates) / sizeof(*rates); ++rate)
            for (unsigned tone = 0; tone < sizeof(tones) / sizeof(*tones); ++tone)
                minimum = fmin(minimum, check_sideband(mode, rates[rate], tones[tone]));
    printf("Minimum tested SSB image rejection %.1f dB\n", minimum);
}
