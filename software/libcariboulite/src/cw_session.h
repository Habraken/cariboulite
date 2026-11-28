#pragma once
#include "cw_keyer.h"
#include "audio_sink.h"
#include <stddef.h>

typedef struct cw_session cw_session_t;
typedef struct {
    bool enabled, connected, practice, requested, busy;
    unsigned contacts,wpm,hang_ms;
    float ramp_ms;
    bool swap;
    cw_key_type_t type;
    int error;
    uint64_t audio_samples,rf_samples;
} cw_status_t;
/* Session owns the physical playback device. RX receives a virtual borrowed
 * sink, allowing immediate sidetone during RX, TX setup and practice. */
cw_session_t* cw_session_create(const char* input,const char* playback,bool simulated);
void cw_session_destroy(cw_session_t*);
int cw_session_enable(cw_session_t*,bool);
int cw_session_configure(cw_session_t*,cw_key_type_t,unsigned,bool,bool,unsigned,float);
void cw_session_set_audio(cw_session_t*,float pitch,float amplitude,float volume_db);
void cw_session_status(cw_session_t*,cw_status_t*);
void cw_session_cancel(cw_session_t*);
/* Called only by the single TX producer, including its preflight prime. */
int cw_session_render(cw_session_t*,float* envelope,size_t count);
audio_sink_t* cw_session_rx_sink(cw_session_t*);
void cw_session_receive(cw_session_t*,bool);
int cw_session_test_contacts(cw_session_t*,unsigned); /* simulated sessions only */
