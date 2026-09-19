#pragma once
#include "cariboulite.h"
#include "cariboulite_setup.h"
#include "pipeline_transport.h"
#include "demod_worker.h"
#include "audio_sink.h"

// Internal application handles. One control owner serializes lifecycle calls.
typedef struct {
    bool active;
    audio_sink_t* sink;
    aud10_fifo_t* fifo;     // source of 10 ms audio frames
    size_t xruns;
} audio_writer_ctrl_t;

typedef struct {
    bool active;
    const atomic_uint* squelch_flags;
    cariboulite_radio_state_st *radio;
    cariboulite_sample_complex_int16 *rx_buffer;
    size_t rx_buffer_size;
	
	// new
	rf10_fifo_t* rx_fifo;
} rx_reader_ctrl_st;

typedef struct {
    // Radio
    double freq_hz;             // e.g., 430.1e6

    // Demod/audio
    const char* pcm_dev;        // ALSA playback device ("plughw:3,0" etc.)
    float  deemph_tau_s;        // 50e-6 (EU) or 75e-6 (NA)
    float  pcm_gain;            // e.g., 8000.0f

    bool noise_squelch_disabled; // zero/default enables noise squelch
    bool carrier_squelch_enabled; // zero/default disables carrier squelch

    fm_demod_mode_t mode;       // zero/default is NBFM; selected at initialization

    // Fixed rates
    float  fs_rf;               // 4e6
    float  fs_audio;            // 48e3
} rx_params_t;

typedef struct {
    // Allocated/owned objects
    rf10_fifo_t          rxq;       // IQ@4M → 10ms frames
    aud10_fifo_t         afifo;     // 10ms PCM for ALSA
    rx_reader_ctrl_st    rx_ctrl;
    nbfm_demod_ctrl_t    demod;
    audio_writer_ctrl_t  aw;

    // Threads
    pthread_t            rx_thread;
    pthread_t            demod_thread;
    pthread_t            aw_thread;
    bool rx_thread_created;
    bool demod_thread_created;
    bool aw_thread_created;

    // State
    bool inited;
    bool running;

    // Binding
    sys_st*                         sys;
    cariboulite_radio_state_st*     radio;
} rx_pipeline_t;

typedef struct {
    rf10_stats_t rxq;
} rx_pipeline_stats_t;

extern cariboulite_sample_complex_int16 latest_rx_sample;
int rx_pipeline_init(rx_pipeline_t*, sys_st*, cariboulite_radio_state_st*, const rx_params_t*);
int rx_pipeline_start(rx_pipeline_t*);
void rx_pipeline_stop(rx_pipeline_t*);
void rx_pipeline_destroy(rx_pipeline_t*);
bool rx_pipeline_running(const rx_pipeline_t*);
int rx_pipeline_set_freq(rx_pipeline_t*, double freq_hz);
int rx_pipeline_set_pcm_gain(rx_pipeline_t*, float gain);
int rx_pipeline_set_deemph(rx_pipeline_t*, float tau_s);
void rx_pipeline_get_stats(rx_pipeline_t*, rx_pipeline_stats_t*);
// Internal worker helpers reused by the modem self-test and lifecycle checks.
int write_audio_exact(audio_sink_t*, const int16_t*, size_t frames);
void* audio_writer_thread(void*);
void* rx_reader_thread_func(void*);

size_t rx_pipeline_frame_samples(const rx_pipeline_t*);
void rx_pipeline_reset_stats(rx_pipeline_t*);

// Thread-safe controls; configuration survives start/stop, reset on re-init.
void rx_pipeline_set_squelch(rx_pipeline_t*, bool noise_enabled, bool carrier_enabled);
bool rx_pipeline_squelch_open(const rx_pipeline_t*);
