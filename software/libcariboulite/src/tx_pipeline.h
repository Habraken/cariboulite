#pragma once
#include "cariboulite.h"
#include "cariboulite_setup.h"
#include "pipeline_transport.h"
#include "audio_source.h"
#include "nbfm_mod.h"

// Internal application handles. One control owner serializes lifecycle calls.
typedef struct {
    volatile int   frames_left;   // number of 10ms frames to override
    volatile float hz;            // 0 => zeros, else tone frequency
} tone_injector_t;

typedef struct {
    bool active;
    cariboulite_radio_state_st *radio;
    cariboulite_sample_complex_int16 *tx_buffer;
    size_t tx_buffer_size;

	// (live path)
    bool    live_from_mic;     // set true to enable live generation
    audio_source_t* mic;     // ALSA handle
    nbfm_mod_t*     fm;      // 48 kHz audio -> configured RF rate NBFM
    float*            a48k;    // 480-float scratch
    iq16_t*           iq_rf;    // One RF frame of IQ scratch
	
	// test tone generator for the FM modulator
    bool     tone_mode;        // true => synthesize 600 Hz audio
    audio_source_t* tone;      // worker-owned oscillator, shared by tone and injection
    float    tone_hz;          // default 600.0f
    float    tone_amp;         // audio amplitude (0..1), e.g. 0.8f
	
    size_t frame_samples;     // Immutable while pipeline threads exist
    rf10_fifo_t* fifo;         // FIFO for 10 ms frames
	
    // new
	tone_injector_t inj;       // tone injector parameters

} tx_writer_ctrl_st;

typedef struct {
    bool                active;
    tx_writer_ctrl_st*  tx;      // reuse your modulator/mic/tone fields
    rf10_fifo_t*        fifo;
} dsp_producer_ctrl_t;

typedef struct {
    // Radio
    double freq_hz;             // e.g., 430.1e6
    int    tx_power_dbm;        // e.g., -3
    unsigned rf_fs;            // 0 defaults to 4 MS/s; also supports 2 MS/s

    // Baseband source for NBFM mod
    bool   tone_mode;           // true => synth audio
    float  tone_hz;             // default 600.0f
    float  tone_amp;            // 0..1 (e.g., 0.4f)
    const char* mic_dev;        // ALSA capture device or NULL for no mic

    // NBFM modulator config (kept same as your current)
    float  out_scale;           // e.g., 4000.0f
    float  f_dev_hz;            // e.g., 2500.0f
} tx_params_t;

typedef struct {
    // Allocated/owned objects
    rf10_fifo_t         txq;
    tx_writer_ctrl_st   tx_ctrl;
    dsp_producer_ctrl_t dsp_ctrl;

    // Threads
    pthread_t           dsp_thread;
    pthread_t           tx_thread;
    bool dsp_thread_created;
    bool tx_thread_created;

    // State
    bool inited;
    bool running;

    // Binding
    sys_st*                         sys;
    cariboulite_radio_state_st*     radio;
} tx_pipeline_t;

typedef struct {
    rf10_stats_t txq;
} tx_pipeline_stats_t;

extern cariboulite_sample_complex_int16 latest_tx_sample;
int tx_pipeline_init(tx_pipeline_t*, sys_st*, cariboulite_radio_state_st*, const tx_params_t*);
int tx_pipeline_start(tx_pipeline_t*);
void tx_pipeline_stop(tx_pipeline_t*);
void tx_pipeline_destroy(tx_pipeline_t*);
bool tx_pipeline_running(const tx_pipeline_t*);
int tx_pipeline_set_freq_power(tx_pipeline_t*, double freq_hz, int tx_power_dbm);
void tx_pipeline_get_stats(tx_pipeline_t*, tx_pipeline_stats_t*);

size_t tx_pipeline_frame_samples(const tx_pipeline_t*);
void tx_pipeline_reset_stats(tx_pipeline_t*);
