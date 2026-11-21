#ifndef CARIBOULITE_COMMAND_PROTOCOL_H
#define CARIBOULITE_COMMAND_PROTOCOL_H

#include <stdbool.h>
#include "audio_demod.h"
#include "audio_mod.h"
#include <stddef.h>
#include <stdint.h>
#include "caribou_fpga/caribou_fpga_regs.h"

#define CONTROL_PROTOCOL_MAX_PACKET_BYTES 1024

typedef enum control_command {
    CONTROL_GET,
    CONTROL_RX_VOLUME,
    CONTROL_START,
    CONTROL_STOP,
    CONTROL_FREQUENCY,
    CONTROL_MODE,
    CONTROL_SAMPLE_RATE,
    CONTROL_SQUELCH,
    CONTROL_CARRIER_LEVELS,
    CONTROL_TX_START,
    CONTROL_TX_STOP,
    CONTROL_TX_FREQUENCY,
    CONTROL_TX_POWER,
    CONTROL_TX_SAMPLE_RATE,
    CONTROL_TX_SOURCE,
    CONTROL_TX_MODE,
    CONTROL_TX_AM_DEPTH,
    CONTROL_TX_AM_BACKOFF,
    CONTROL_TX_TONE,
    CONTROL_FPGA_REGS,
    CONTROL_FPGA_LED_GET,
    CONTROL_FPGA_LED_SET,
    CONTROL_FPGA_PROGRAM,
    CONTROL_FPGA_SOFT_RESET,
    CONTROL_FPGA_HARD_RESET,
    CONTROL_BOTH_FREQUENCY,
    CONTROL_BOTH_MODE,
    CONTROL_BOTH_SAMPLE_RATE,
    /* These are transport-generated requests, never valid wire commands. */
    CONTROL_INVALID,
    CONTROL_BUSY
} control_command_t;

typedef struct control_request {
    /* Zero selects v1 when formatting existing callers' requests. */
    unsigned protocol_version;
    uint64_t id;
    control_command_t command;
    double frequency_hz;
    float volume_db;
    unsigned sample_rate_hz;
    audio_demod_mode_t mode;
    bool noise_enabled;
    bool carrier_enabled;
    float noise_open_rms;
    float noise_close_rms;
    int carrier_open_dbm, carrier_close_dbm;
    double tx_frequency_hz;
    int tx_power_dbm;
    unsigned tx_sample_rate_hz;
    bool tx_tone_mode;
    audio_mod_mode_t tx_mode;
    float tx_am_depth;
    float tx_am_backoff_db;
    float tx_tone_hz;
    float tx_tone_amp;
    bool fpga_led0, fpga_led1;
} control_request_t;

typedef struct control_reply {
    unsigned protocol_version;
    uint64_t id;
    bool ok;
    char code[32];
    uint64_t config_generation;
    uint64_t pipeline_generation;
    bool simulated;
    char state[24];
    char channel[8];
    double requested_frequency_hz;
    double actual_frequency_hz;
    unsigned sample_rate_hz;
    audio_demod_mode_t mode;
    bool noise_enabled;
    bool carrier_enabled;
    float noise_open_rms;
    float noise_close_rms;
    uint64_t tx_pipeline_generation;
    double tx_requested_frequency_hz;
    double tx_actual_frequency_hz;
    unsigned tx_sample_rate_hz;
    int tx_requested_power_dbm;
    int tx_actual_power_dbm;
    bool tx_tone_mode;
    float tx_tone_hz;
    float tx_tone_amp;
    float tx_deviation_hz;
    bool tx_settings_valid; /* False for legacy 30-token v2 replies. */
    audio_mod_mode_t tx_mode;
    float tx_am_depth;
    float tx_am_backoff_db;
    bool tx_requested;
    bool rx_resume;
    bool transition;
    bool fpga_regs_present; /* Explicit register-command replies only. */
    caribou_fpga_regs_st fpga_regs;
    bool fpga_io_valid; /* Present only in explicit LED command replies. */
    unsigned fpga_io_pins; /* LED0/1 bits0/1, CFG bits3..6, BTN bit7. */
} control_reply_t;

/* Strict ASCII protocol-v1/v2 packets. v1 carries RX commands/settings; v2
 * extends them with TX controls, combined RX/TX settings, FPGA programming and
 * transition states. Current v2 replies append TX mode/depth/backoff; the
 * parser also accepts legacy 30-field replies with tx_settings_valid=false.
 * Explicit LED replies append "fpga_io N" with digital readback; ordinary
 * status replies remain unchanged.
 * Combined requests use frequency_hz, mode or sample_rate_hz
 * and require both RX and TX to be stopped. Parsers accept ASCII whitespace between
 * tokens, reject embedded NULs, nonfinite/overflowed numbers and trailing
 * tokens. They return 0 on success or -1 with errno on failure. A failed
 * request parse retains a syntactically valid request ID when available. */
int control_request_parse(const char *packet, size_t bytes,
                          control_request_t *request);
int control_reply_parse(const char *packet, size_t bytes,
                        control_reply_t *reply);

/* Formatters include a trailing newline and write a terminating NUL for the
 * caller. The returned byte count excludes that NUL; never transmit it.
 * Returns -1 for an invalid object or insufficient buffer space. */
int control_request_format(const control_request_t *request,
                           char *packet, size_t capacity);
int control_reply_format(const control_reply_t *reply,
                         char *packet, size_t capacity);

#endif
