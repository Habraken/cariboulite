#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HEADSET_REPORT_MAX 4096
#define HEADSET_FIELDS_MAX 32
typedef enum { HEADSET_NONE, HEADSET_TOGGLE, HEADSET_OFF } headset_action_t;
typedef enum { HEADSET_GENERIC, HEADSET_H650E, HEADSET_JABRA510 } headset_profile_t;
typedef struct {
    uint32_t usage, last;
    unsigned id, offset, bits, flags;
    bool seen;
} headset_field_t;
typedef struct {
    headset_profile_t profile;
    headset_field_t fields[HEADSET_FIELDS_MAX];
    unsigned count, input_bits[256], output_bits[256];
    bool numbered, feedback, have_toggle;
    unsigned feedback_id, feedback_offset;
    bool pending_rise, pending_fall, pending_flash, pending_drop;
    double deadline, last_toggle;
} headset_ptt_t;

/* Standard scalar Hook Switch/Flash/Drop controls are discovered from HID
 * descriptors, without a vendor allowlist. The two measured device quirks
 * use adapters; unsupported layouts fail closed. */
int headset_ptt_init(headset_ptt_t*, const uint8_t*, size_t, unsigned vendor, unsigned product);
const char* headset_ptt_profile_name(const headset_ptt_t*);
void headset_ptt_prime(headset_ptt_t*, const uint8_t*, size_t);
void headset_ptt_feed(headset_ptt_t*, const uint8_t*, size_t, double now);
headset_action_t headset_ptt_poll(headset_ptt_t*, double now);
int headset_ptt_wait_ms(const headset_ptt_t*, double now);
/* Output reports include an ID byte (zero for unnumbered devices).
 * Only Off-Hook is modified in a report preserved by the caller. */
size_t headset_ptt_feedback_size(const headset_ptt_t*);
int headset_ptt_feedback(const headset_ptt_t*, uint8_t*, size_t, bool active);
