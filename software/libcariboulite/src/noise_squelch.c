#include "noise_squelch.h"
#include <math.h>
#include <string.h>
void noise_squelch_reset(noise_squelch_t* s) { memset(s, 0, sizeof(*s)); }
bool noise_squelch_process(noise_squelch_t* s, float x)
{
    if (!isfinite(x)) { noise_squelch_reset(s); return false; }
    for (unsigned k = 0; k < 2; ++k) {
        float y = 0.5690356f * x + s->z1[k];
        s->z1[k] = -1.1380712f * x + 0.9428090f * y + s->z2[k];
        s->z2[k] = 0.5690356f * x - 0.3333333f * y;
        x = y;
    }
    s->power += 0.001041124f * (x*x - s->power); // 1-exp(-1/(48000*.020))
    if (!isfinite(s->power)) { noise_squelch_reset(s); return false; }
    bool transition = s->open ? s->power > NOISE_SQUELCH_CLOSE_RMS * NOISE_SQUELCH_CLOSE_RMS
                              : s->power < NOISE_SQUELCH_OPEN_RMS * NOISE_SQUELCH_OPEN_RMS;
    if (!transition) s->qualify = 0;
    else if (++s->qualify >= (s->open ? 5760u : 1440u)) {
        s->open = !s->open;
        s->qualify = 0;
    }
    return s->open;
}
