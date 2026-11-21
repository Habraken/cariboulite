#include "carrier_squelch.h"
#include <math.h>
void carrier_squelch_reset(carrier_squelch_t* s) { *s = (carrier_squelch_t){0}; }
bool carrier_squelch_process_levels(carrier_squelch_t* s, float rssi, bool valid, uint32_t levels)
{
    if (!valid || !isfinite(rssi) || rssi < CARRIER_SQUELCH_MIN_DBM || rssi > CARRIER_SQUELCH_MAX_DBM) {
        carrier_squelch_reset(s);
        return false;
    }
    if (levels != s->levels) {
        s->levels = levels;
        s->qualify = 0;
    }
    int open, close;
    carrier_squelch_unpack_levels(levels, &open, &close);
    bool transition = s->open ? rssi < close : rssi >= open;
    if (!transition) s->qualify = 0;
    else if (++s->qualify >= (s->open ? 15u : 3u)) {
        s->open = !s->open;
        s->qualify = 0;
    }
    return s->open;
}
