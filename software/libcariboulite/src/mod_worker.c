#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "tx_pipeline.h"
#include "pipeline_runtime.h"
#include "mod_worker.h"
#include "tone_source.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

pthread_mutex_t g_tx_injection_lock = PTHREAD_MUTEX_INITIALIZER;
cariboulite_sample_complex_int16 latest_tx_sample = {0};

static inline void fill_tone_48k(tx_writer_ctrl_st* ctrl, float* buf, size_t n)
{
    tone_source_set(ctrl->tone, ctrl->tone_hz, ctrl->tone_amp);
    audio_source_read(ctrl->tone, buf, n);
}

static bool read_audio_exact(audio_source_t* mic, float* buf, size_t need)
{
    size_t have = 0;
    while (have < need) {
        audio_source_result_t result = audio_source_read(mic, buf + have, need - have);
        if (result.status == AUDIO_SOURCE_ERROR || result.status == AUDIO_SOURCE_EOF)
            return false;
        size_t got = result.frames;
        if (got == 0) {
            // tiny sleep to avoid hot spin if device is momentarily empty
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 2 * 1000 * 1000 }; // 2 ms
            nanosleep(&ts, NULL);
        }
        have += got;
    }
    return true;
}

void* nbfm_mod_thread(void* arg)
{
    pthread_setname_np(pthread_self(), "dsp_producer_thread");
    //set_rt_and_affinity();   // make sure this logs failures
    //set_rt_and_affinity_prio(45,-1);
    set_rt_and_affinity_prio(40,0);

    dsp_producer_ctrl_t* ctrl = (dsp_producer_ctrl_t*)arg;
    if (!ctrl || !ctrl->tx || !ctrl->tx->fm || !ctrl->fifo ||
        !ctrl->tx->a48k || !ctrl->tx->iq_rf)
        return NULL;

    const uint64_t PERIOD_NS = 10ull * 1000ull * 1000ull; // 10 ms
    uint64_t next_ns = mono_ns();   // anchor current time
    uint64_t last_wake = 0;
    size_t frame_idx = 0;

    while (ctrl->active) {

        // ============================================================
        // TX OFF: do not generate or enqueue frames
        // ============================================================
        if (!nbfm_tx_active) {
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 2 * 1000 * 1000 }; // 2 ms
            nanosleep(&ts, NULL);

            // Re-anchor timing so we don't accumulate drift
            next_ns = mono_ns();
            last_wake = 0;
            continue;
        }

        // ============================================================
        // Normal TX ON path (10 ms cadence)
        // ============================================================
        // ---- schedule next absolute wake ----
        next_ns += PERIOD_NS;
        struct timespec next_ts = {
            .tv_sec  = (time_t)(next_ns / 1000000000ull),
            .tv_nsec = (long)(next_ns % 1000000000ull)
        };

        // ---- sleep until the absolute deadline, handle EINTR ----
        int rc;
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_ts, NULL);
        } while (rc == EINTR);

        // ---- timing diagnostics ----
        uint64_t now = mono_ns();
        if (last_wake) {
            double dt_ms = (now - last_wake) / 1e6;
            if ((frame_idx++ % 50) == 0)
                fprintf(stderr, "producer: dt = %.3f ms  rc = %d\n", dt_ms, rc);
        }
        last_wake = now;

        // ---- if sleep failed or we drifted >50 ms, re-anchor ----
        if (rc != 0 || now > next_ns + 5 * PERIOD_NS) {
            next_ns = now;
            fprintf(stderr, "producer: re-anchor (rc=%d)\n", rc);
        }

        // ============================================================
        // 1) Generate 10 ms of audio @ 48 kHz (480 samples)
        // ============================================================
        
        // if (ctrl->tx->tone_mode) {
        //     fill_tone_48k(ctrl->tx, ctrl->tx->a48k, 480);
        // } else if (ctrl->tx->mic) {
        //     read_audio_exact(ctrl->tx->mic, ctrl->tx->a48k, 480);
        // } else {
        //     memset(ctrl->tx->a48k, 0, 480 * sizeof(float));
        // }
        
        // 1) Generate 10 ms of audio @ 48 kHz (480 samples)
        // Injector overrides: hz==0 => silence, else tone(hz)
        pthread_mutex_lock(&g_tx_injection_lock);
        int inj_left = ctrl->tx->inj.frames_left;
        float inj_hz = ctrl->tx->inj.hz;

        if (inj_left > 0) ctrl->tx->inj.frames_left = inj_left - 1;
        pthread_mutex_unlock(&g_tx_injection_lock);

        if (inj_left > 0) {

            tone_source_set(ctrl->tx->tone, inj_hz, ctrl->tx->tone_amp);
            audio_source_read(ctrl->tx->tone, ctrl->tx->a48k, 480);

            __sync_synchronize();
        } else {
            // normal path
            if (ctrl->tx->tone_mode) {
                fill_tone_48k(ctrl->tx, ctrl->tx->a48k, 480);
            } else if (ctrl->tx->mic) {
                if (!read_audio_exact(ctrl->tx->mic, ctrl->tx->a48k, 480)) {
                    fprintf(stderr, "TX audio source failed; stopping stream\n");
                    nbfm_tx_active = false;
                    continue;
                }
            } else {
                memset(ctrl->tx->a48k, 0, 480 * sizeof(float));
            }
        }


        // ============================================================
        // 2) Modulate 10 ms of audio at the selected RF rate
        // ============================================================
        nbfm_result_t mod = nbfm_process(ctrl->tx->fm, ctrl->tx->a48k, 480,
                                          ctrl->tx->iq_rf, ctrl->tx->frame_samples);
        if (mod.error || mod.consumed != 480 ||
            mod.produced != ctrl->tx->frame_samples || mod.held_audio) {
            fprintf(stderr, "TX modulator error: %d, audio=%zu, IQ=%zu, held=%zu\n",
                    mod.error, mod.consumed, mod.produced, mod.held_audio);
            nbfm_tx_active = false;
            continue;
        }

        // ============================================================
        // 3) Pack one rf10_frame_t and push to FIFO (tag TX_EN)
        // ============================================================
        rf10_frame_t frm = {0};
        for (size_t i = 0; i < ctrl->tx->frame_samples; i++) {
            frm.data[i].i = ctrl->tx->iq_rf[i].i | 0x0001;  // TX_EN in LSB
            frm.data[i].q = ctrl->tx->iq_rf[i].q;
        }

        // Optional live sample for UI/debug
        latest_tx_sample.i = frm.data[ctrl->tx->frame_samples / 2].i;
		latest_tx_sample.q = frm.data[ctrl->tx->frame_samples / 2].q;

        // Blocking put; don’t drop frames
        bool ok = rf10_fifo_put(ctrl->fifo, &frm, -1);
        if (!ok) break; // stop signal
    }

    return NULL;
}
