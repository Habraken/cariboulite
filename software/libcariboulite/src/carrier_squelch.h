#pragma once
#include <stdbool.h>
#include <stdint.h>
// Single-owner, allocation-free RSSI detector. Call once per 10 ms RF block.
// Thresholds are modem RSSI dBm, not calibrated external connector power.
#define CARRIER_SQUELCH_OPEN_DBM (-97)
#define CARRIER_SQUELCH_CLOSE_DBM (-102)
#define CARRIER_SQUELCH_MIN_DBM (-127)
#define CARRIER_SQUELCH_MAX_DBM 4

// Pack both integral dBm thresholds for one atomic handoff. Zero is the default.
static inline uint32_t carrier_squelch_pack_levels(int open, int close)
{
    if (close < CARRIER_SQUELCH_MIN_DBM || open > CARRIER_SQUELCH_MAX_DBM || open <= close)
        return 0;
    return ((uint32_t)(open - CARRIER_SQUELCH_MIN_DBM) << 8) |
           (uint32_t)(close - CARRIER_SQUELCH_MIN_DBM);
}

static inline void carrier_squelch_unpack_levels(uint32_t levels, int* open, int* close)
{
    *open = levels ? (int)(levels >> 8) + CARRIER_SQUELCH_MIN_DBM : CARRIER_SQUELCH_OPEN_DBM;
    *close = levels ? (int)(levels & 0xffu) + CARRIER_SQUELCH_MIN_DBM : CARRIER_SQUELCH_CLOSE_DBM;
}

typedef struct { uint32_t levels; unsigned qualify; bool open; } carrier_squelch_t;
void carrier_squelch_reset(carrier_squelch_t* s);
// Three strong blocks open; fifteen weak blocks close. Invalid/read-failed
// measurements immediately close, with no reuse of cached RSSI.
bool carrier_squelch_process_levels(carrier_squelch_t* s, float rssi_dbm, bool valid,
                                    uint32_t levels);
static inline bool carrier_squelch_process(carrier_squelch_t* s, float rssi_dbm, bool valid)
{
    return carrier_squelch_process_levels(s, rssi_dbm, valid, 0);
}
