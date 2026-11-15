#pragma once
#include <stdint.h>
#include <math.h>

/* AT86RF215 AM experiment: second-order error feedback on a 64-code grid.
 * Update at 40 kHz using the exact 4 MHz/divider clock, then hold between
 * updates. Native-rate noise shaping is filtered before the effective modem
 * quantizer and did not correct weak modulation in the hardware comparison.
 */
typedef struct {
    double error1, error2;
    uint32_t phase;
    int16_t held;
} am_dac_shaper_t;

static inline int16_t am_dac_shaper_process(am_dac_shaper_t *s,
                                           double envelope, unsigned divider)
{
    s->phase += 40000U * divider;
    if (s->phase >= 4000000U) {
        s->phase -= 4000000U;
        /* Error is envelope area, not just amplitude: fractional native clocks
         * alternate hold lengths (e.g. 12/13 samples at 500 kS/s). Weight the
         * feedback by this cell's actual duration to preserve the mean. */
        unsigned increment = 40000U * divider;
        unsigned duration = (4000000U - s->phase + increment - 1) / increment;
        double corrected = envelope + (2 * s->error1 - s->error2) / duration;
        double quantized = 64 * nearbyint(corrected / 64);
        if (quantized < 0 || quantized > 4032) {
            /* Bound the AM envelope and prevent feedback windup at a rail. */
            quantized = fmax(0, fmin(4032, quantized));
            s->error1 = s->error2 = 0;
        } else {
            s->error2 = s->error1;
            s->error1 = (corrected - quantized) * duration;
        }
        s->held = (int16_t)quantized;
    }
    return s->held;
}
