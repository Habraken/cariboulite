#include "audio_demod.h"
#include "math_compat.h"
#include "radio_rates.h"

#include <assert.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double amplitude(const int16_t* data, unsigned start, unsigned end, double hz)
{
    double complex projection = 0;
    for (unsigned k = start; k < end; ++k) {
        double phase = 2 * M_PI * hz * k / 48000;
        projection += data[k] * (cos(phase) - I * sin(phase));
    }
    return 2 * cabs(projection) / (end - start);
}

static void check_mode(audio_demod_mode_t mode, unsigned rate, bool inverted, int offset)
{
    const double fs = radio_rate_exact(rate);
    const size_t count = (size_t)(fs * .32);
    const size_t capacity = 16000;
    const int sign = (mode == AUDIO_DEMOD_USB) != inverted ? 1 : -1;
    iq16_t* input = malloc(count * sizeof(*input));
    int16_t* one = malloc(capacity * sizeof(*one));
    int16_t* split = malloc(capacity * sizeof(*split));
    audio_demod_config_t cfg = {rate, 48000, 75e-6f, 24000};
    audio_demod_t* dsp = audio_demod_create(mode, &cfg);
    assert(input && one && split && dsp);
    assert(!audio_demod_capabilities(dsp));
    assert(audio_demod_set_am_agc(dsp, true) < 0);
    assert(audio_demod_set_ssb_tuning(dsp, offset, inverted) == 0);

    // Equal wanted/opposite tones, residual carrier and a strong tuner-DC spur.
    for (size_t k = 0; k < count; ++k) {
        double t = 2 * M_PI * k / fs;
        double a = (sign * 900 - offset) * t;
        double b = (-sign * 1300 - offset) * t;
        double carrier = -offset * t;
        input[k] = (iq16_t){
            lrint(1500 * cos(a) + 1500 * cos(b) + 300 * cos(carrier) + 600),
            lrint(1500 * sin(a) + 1500 * sin(b) + 300 * sin(carrier))
        };
    }

    audio_demod_result_t whole = audio_demod_process(dsp,input,count,one,capacity,.0002);
    assert(!whole.error && whole.consumed == count && whole.produced > 15300);
    audio_demod_reset(dsp);
    size_t used = 0, produced = 0;
    while (used < count) {
        size_t block = count - used;
        if (block > 3713) block = 3713;
        audio_demod_result_t part = audio_demod_process(dsp,input + used,block,
                                                        split + produced,31,.0002);
        assert(!part.error && part.consumed);
        used += part.consumed;
        produced += part.produced;
    }
    assert(produced == whole.produced);
    assert(!memcmp(one, split, produced * sizeof(*one)));

    double wanted = amplitude(one, 5760, 15360, 900 / 1.0002);
    double opposite = amplitude(one, 5760, 15360, 1300 / 1.0002);
    double rejection = 20 * log10(wanted / fmax(.01, opposite));
    assert(wanted > 8000 && wanted < 9400);
    assert(rejection > 45);
    printf("%s %u inverted=%d offset=%d image rejection %.1f dB\n",
           audio_demod_mode_name(mode), rate, inverted, offset, rejection);
    free(input);
    free(one);
    free(split);
    audio_demod_destroy(dsp);
}

int main(void)
{
    const unsigned rates[] = {400000,500000,666667,800000,1000000,1333333,2000000,4000000};
    for (audio_demod_mode_t mode = AUDIO_DEMOD_USB; mode <= AUDIO_DEMOD_LSB; ++mode)
        for (unsigned k = 0; k < sizeof(rates) / sizeof(*rates); ++k)
            check_mode(mode, rates[k], k % 2 != 0, k % 2 ? -12500 : 12500);
    check_mode(AUDIO_DEMOD_USB, 1000000, false, 0);
    check_mode(AUDIO_DEMOD_LSB, 1000000, true, 0);
}
