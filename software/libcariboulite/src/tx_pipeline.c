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
#include "alsa_source.h"
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

static void* tx_writer_thread_func(void* arg)
{
    pthread_setname_np(pthread_self(), "tx_writer_thread");
    //set_rt_and_affinity();
    //set_rt_and_affinity_prio(42,-1);
    set_rt_and_affinity_prio(48,0);


    tx_writer_ctrl_st* ctrl = (tx_writer_ctrl_st*)arg;
    if (!ctrl || !ctrl->radio || !ctrl->radio->sys) return NULL;

    caribou_smi_st *smi = &ctrl->radio->sys->smi;
    if (!smi || smi->filedesc < 0) return NULL;

    rf10_fifo_t* fifo = ctrl->fifo;
    if (!fifo) return NULL;

    const caribou_smi_channel_en ch =
        (ctrl->radio == &ctrl->radio->sys->radio_low) ?
            caribou_smi_channel_900 : caribou_smi_channel_2400;

    // non-blocking fd
    int flags = fcntl(smi->filedesc, F_GETFL, 0);
    if (flags != -1) fcntl(smi->filedesc, F_SETFL, flags | O_NONBLOCK);

    // Discover kernel "native" buffer and its quarter size (in samples)
    size_t native_bytes = caribou_smi_get_native_batch_samples(smi);
    const int BYTES_PER_SAMPLE = (int)sizeof(caribou_smi_sample_complex_int16);
    size_t quarter_samples = (native_bytes / 4) / BYTES_PER_SAMPLE;
    if (quarter_samples == 0) quarter_samples = 8192; // safe default if ioctl failed

    // Arm TX state once, then just keep feeding
    int tx_active_hw = 0;

    while (1) {
        pthread_testcancel();
        if (!ctrl->active) break;

        if (!nbfm_tx_active) {
            if (tx_active_hw) {
                caribou_smi_set_driver_streaming_state(smi, (smi_stream_state_en)0);
                tx_active_hw = 0;
            }
            // light idle: don't busy spin
            struct timespec ts = {0, 2000000}; // 2 ms
            nanosleep(&ts, NULL);
            continue;
        }

        if (!tx_active_hw) {
            caribou_smi_set_driver_streaming_state(smi, (smi_stream_state_en)3); // TX
            tx_active_hw = 1;
        }

        // Get one frame from the producer (blocking). Size = 40k IQ16 samples.
        rf10_frame_t frm;
        if (!rf10_fifo_get(fifo, &frm, /*timeout_ms=*/-1)) {
            continue;
        }

        // Stream it out in chunks ≈ kernel quarter (keep kfifo topped up)
        size_t off = 0;
        const size_t total = ctrl->frame_samples; // Selected rate * 10 ms
        struct pollfd pfd = { .fd = smi->filedesc, .events = POLLOUT, .revents = 0 };

        while (off < total && nbfm_tx_active) {
            // Aim for quarter-sized writes; last piece can be smaller
            size_t todo = total - off;
            if (todo > quarter_samples) todo = quarter_samples;

            // Wait until driver is ready to accept bytes
            int pr = poll(&pfd, 1, 10);  // short timeout; loop if needed
            if (pr <= 0 || !(pfd.revents & POLLOUT)) continue;

            caribou_smi_sample_complex_int16 *p =
                (caribou_smi_sample_complex_int16 *)(frm.data + off);

            int sent = caribou_smi_write_samples(smi, ch, p, (int)todo);  // returns *samples*
            if (sent > 0) {
                off += (size_t)sent;
            } else if (sent == 0 || (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                // transient backpressure -> try again
                continue;
            } else {
                // hard error: drop to idle cleanly
                nbfm_tx_active = false;
                break;
            }
        }
    }

    if (tx_active_hw) {
        caribou_smi_set_driver_streaming_state(smi, (smi_stream_state_en)0);
        HW_LOCK();
        cariboulite_radio_activate_channel(ctrl->radio, cariboulite_channel_dir_tx, false);
        HW_UNLOCK();
    }
    return NULL;
}

int tx_pipeline_init(tx_pipeline_t* p, sys_st* sys,
                     cariboulite_radio_state_st* radio,
                     const tx_params_t* par)
{
    if (!p || !sys || !radio || !par) return -1;
    memset(p, 0, sizeof(*p));
    p->sys   = sys;
    p->radio = radio;

    unsigned rf_fs = par->rf_fs ? par->rf_fs : 4000000;
    if (rf_fs != 4000000 && rf_fs != 2000000) return -1;
    p->tx_ctrl.frame_samples = rf_fs / 100;

    // FIFOs
    rf10_fifo_init(&p->txq, /*cap=*/64, /*drop_oldest_on_full=*/false);

    p->inited = true; // FIFO synchronization is ready for staged cleanup.
    int error = -2;
    if (!p->txq.q) goto fail;

    // tx_ctrl wiring (reuse your structures/threads)
    p->tx_ctrl.active         = true;
    p->tx_ctrl.radio          = radio;
    p->tx_ctrl.fifo           = &p->txq;
    p->tx_ctrl.live_from_mic  = (par->mic_dev && *par->mic_dev);
    p->tx_ctrl.mic            = NULL;            // (open later if needed)
    p->tx_ctrl.tone_mode      = par->tone_mode;
    p->tx_ctrl.tone_hz        = par->tone_hz;
    p->tx_ctrl.tone_amp       = par->tone_amp;
    p->tx_ctrl.tone = tone_source_open(par->tone_hz, par->tone_amp,
                                        (audio_format_t){48000, 1});
    if (!p->tx_ctrl.tone) goto fail;
    p->tx_ctrl.fm             = NULL;
    p->tx_ctrl.a48k           = NULL;
    p->tx_ctrl.iq_rf           = NULL;

    // NBFM mod init
    nbfm_cfg_t cfg = {
        .audio_fs      = 48000.0,
        .rf_fs         = rf_fs,
        .f_dev_hz      = par->f_dev_hz,     // 2500.0
        .preemph_tau_s = 0.0,
        .out_scale     = par->out_scale,    // 4000.0
        .linear_interp = 1,
    };
    p->tx_ctrl.fm   = nbfm_create(&cfg);
    p->tx_ctrl.a48k = (float*) calloc(480,    sizeof(float));
    p->tx_ctrl.iq_rf = (iq16_t*)calloc(p->tx_ctrl.frame_samples,  sizeof(iq16_t));
    if (!p->tx_ctrl.fm || !p->tx_ctrl.a48k || !p->tx_ctrl.iq_rf) {
        goto fail;
    }
    
    p->tx_ctrl.inj.frames_left = 0;
    p->tx_ctrl.inj.hz = 0.0f;

    // Optional mic
    if (p->tx_ctrl.live_from_mic) {
        p->tx_ctrl.mic = alsa_source_open(par->mic_dev, 1.0f, (audio_format_t){48000, 1});
        if (!p->tx_ctrl.mic) {
            fprintf(stderr, "[tx_pipeline] ALSA capture open failed (%s)\n",
                    par->mic_dev ? par->mic_dev : "(null)");
            error = -5;
            goto fail;
        }
    }

    // Radio basic set
    HW_LOCK();
    double init_frequency = par->freq_hz;
    cariboulite_radio_set_frequency(radio, true, &init_frequency);
    cariboulite_radio_set_tx_power (radio, par->tx_power_dbm);
    HW_UNLOCK();

    // Configure and verify rate before creating threads or allowing TX.
    cariboulite_radio_set_tx_samp_cutoff_flt(radio, rf_fs);
    uint8_t gap = 255;
    float actual_fs = 0;
    if (cariboulite_radio_get_tx_samp_cutoff_flt(radio, &actual_fs) < 0 ||
        caribou_fpga_get_sys_ctrl_tx_sample_gap(&sys->fpga, &gap) != 0 ||
        actual_fs != rf_fs || gap != 4000000 / rf_fs - 1) {
        fprintf(stderr, "[tx_pipeline] rate/gap verification failed\n");
        goto fail;
    }

    // Prepare DSP/Writer threads (running idle until .start)
    p->dsp_ctrl.active = true;
    p->dsp_ctrl.tx     = &p->tx_ctrl;
    p->dsp_ctrl.fifo   = &p->txq;
    if (pthread_create(&p->dsp_thread, NULL, nbfm_mod_thread, &p->dsp_ctrl) != 0) {
        error = -3;
        goto fail;
    }
    p->dsp_thread_created = true;

    if (pthread_create(&p->tx_thread,  NULL, tx_writer_thread_func,    &p->tx_ctrl) != 0) {
        error = -4;
        goto fail;
    }
    p->tx_thread_created = true;

    p->inited = true;
    p->running = false;
    return 0;

fail:
    tx_pipeline_destroy(p);
    return error;
}

static inline int ms_to_frames_10ms(int ms) { return (ms + 9) / 10; }

static void tx_clear_injection(tx_pipeline_t* p)
{
    pthread_mutex_lock(&g_tx_injection_lock);
    p->tx_ctrl.inj.frames_left = 0;
    pthread_mutex_unlock(&g_tx_injection_lock);
}

static bool tx_injection_can_run(const tx_pipeline_t* p)
{
    return p && p->running && nbfm_tx_active &&
           p->tx_ctrl.active && p->dsp_ctrl.active;
}

static bool tx_inject_frames(tx_pipeline_t* p, float hz, int frames,
                             uint64_t deadline)
{
    if (!tx_injection_can_run(p) || frames <= 0 || mono_ns() >= deadline)
        return false;

    pthread_mutex_lock(&g_tx_injection_lock);
    p->tx_ctrl.inj.hz = hz;
    p->tx_ctrl.inj.frames_left = frames;
    pthread_mutex_unlock(&g_tx_injection_lock);

    while (true) {
        pthread_mutex_lock(&g_tx_injection_lock);
        int left = p->tx_ctrl.inj.frames_left;
        pthread_mutex_unlock(&g_tx_injection_lock);
        if (left <= 0) break;
        if (!tx_injection_can_run(p) || mono_ns() >= deadline) {
            tx_clear_injection(p);
            return false;
        }
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 2*1000*1000 };
        nanosleep(&ts, NULL);
    }
    return tx_injection_can_run(p);
}

static bool tx_inject_tone_with_zeros(tx_pipeline_t* p,
                                      float hz, int tone_ms,
                                      int pre_zero_frames,
                                      int post_zero_frames)
{
    if (pre_zero_frames  < 1) pre_zero_frames  = 1;
    if (post_zero_frames < 1) post_zero_frames = 1;

    /* One budget for the whole sequence, not a fresh timeout for each stage. */
    const uint64_t deadline = mono_ns() + 1000000000ULL;
    return tx_inject_frames(p, 0.0f, pre_zero_frames, deadline) &&
           tx_inject_frames(p, hz, ms_to_frames_10ms(tone_ms), deadline) &&
           tx_inject_frames(p, 0.0f, post_zero_frames, deadline);
}

int tx_pipeline_start(tx_pipeline_t* p)
{
    if (!p || !p->inited || p->running) return -1;

    // Disable RX if it was on and arm TX chain
    if (nbfm_rx_active) {
        nbfm_rx_active = false;
        HW_LOCK();
        cariboulite_radio_activate_channel(p->radio, cariboulite_channel_dir_rx, false);
        HW_UNLOCK();
        usleep(2000);
    }

    // Clear any stale queued frames before starting TX
    rf10_fifo_flush(&p->txq);

    HW_LOCK();
    caribou_fpga_set_io_ctrl_mode(&p->sys->fpga, 0, caribou_fpga_io_ctrl_rfm_tx_lowpass);
    cariboulite_radio_activate_channel(p->radio, cariboulite_channel_dir_tx, true);
    // give writer a head-start so FIFO fills a bit
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 30*1000*1000 };
    nanosleep(&ts, NULL);
    caribou_smi_set_driver_streaming_state(&p->sys->smi, (smi_stream_state_en)3); // TX
    HW_UNLOCK();

    __sync_synchronize();
    nbfm_tx_active = true;

    p->running = true;
    
    // --- Quindar "start" tone: 2525 Hz for 250 ms with 5 frames of padding ---
    if (!tx_inject_tone_with_zeros(p, 2525.0f, 250, 10, 5)) {
        fprintf(stderr, "TX start tone aborted; stopping TX\n");
        nbfm_tx_active = false; // stop must skip another tone attempt
        tx_pipeline_stop(p);
        return -2;
    }
    
    return 0;
}

static void tx_wait_fifo_drain(tx_pipeline_t* p, int timeout_ms)
{
    if (!p) return;
    const uint64_t t0 = mono_ns();
    while (tx_injection_can_run(p)) {
        rf10_stats_t s;
        rf10_fifo_get_stats(&p->txq, &s);
        if (s.count == 0) return;

        const uint64_t now = mono_ns();
        const double ms = (now - t0) / 1e6;
        if (ms >= (double)timeout_ms) return;

        struct timespec ts = { .tv_sec = 0, .tv_nsec = 2*1000*1000 };
        nanosleep(&ts, NULL);
    }
}

void tx_pipeline_stop(tx_pipeline_t* p)
{
    if (!p || !p->inited || !p->running) return;
    
    // --- Quindar "stop" tone: 2475 Hz for the last 250 ms with 5 frames of padding ---
    // Keep TX running while we send the tail tone
    if (tx_inject_tone_with_zeros(p, 2475.0f, 250, 5, 25)) {
        tx_wait_fifo_drain(p, 600);
    } else {
        fprintf(stderr, "TX tail tone aborted; continuing hardware shutdown\n");
    }
    tx_clear_injection(p);

    nbfm_tx_active = false;
    __sync_synchronize();

    HW_LOCK();
    caribou_smi_set_driver_streaming_state(&p->sys->smi, (smi_stream_state_en)0); // idle
    cariboulite_radio_activate_channel(p->radio, cariboulite_channel_dir_tx, false);
    caribou_fpga_set_io_ctrl_mode(&p->sys->fpga, 0, caribou_fpga_io_ctrl_rfm_low_power);
    HW_UNLOCK();

    p->running = false;
}

void tx_pipeline_destroy(tx_pipeline_t* p)
{
    if (!p || !p->inited) return;

    tx_pipeline_stop(p);

    // stop threads and free
    p->tx_ctrl.active  = false;
    p->dsp_ctrl.active = false;
    rf10_fifo_stop(&p->txq);

    if (p->tx_thread_created) {
        pthread_cancel(p->tx_thread);
        pthread_join(p->tx_thread, NULL);
        p->tx_thread_created = false;
    }
    if (p->dsp_thread_created) {
        pthread_cancel(p->dsp_thread);
        pthread_join(p->dsp_thread, NULL);
        p->dsp_thread_created = false;
    }

    rf10_fifo_destroy(&p->txq);

    if (p->tx_ctrl.iq_rf) free(p->tx_ctrl.iq_rf);
    if (p->tx_ctrl.a48k) free(p->tx_ctrl.a48k);
    if (p->tx_ctrl.fm)   nbfm_destroy(p->tx_ctrl.fm);
    if (p->tx_ctrl.mic)  audio_source_destroy(p->tx_ctrl.mic);
    audio_source_destroy(p->tx_ctrl.tone);
    p->tx_ctrl.tone = NULL;

    p->tx_ctrl.iq_rf = NULL;
    p->tx_ctrl.a48k = NULL;
    p->tx_ctrl.fm = NULL;
    p->tx_ctrl.mic = NULL;
    p->inited = false;
}

bool tx_pipeline_running(const tx_pipeline_t* p) { return p && p->running; }

int tx_pipeline_set_freq_power(tx_pipeline_t* p, double freq_hz, int tx_power_dbm)
{
    if (!p || !p->sys || !p->radio) return -1;
    HW_LOCK();
    int rc = cariboulite_radio_set_frequency(p->radio, true, &freq_hz);
    if (rc == 0) rc = cariboulite_radio_set_tx_power(p->radio, tx_power_dbm);
    HW_UNLOCK();
    return rc;
}

void tx_pipeline_get_stats(tx_pipeline_t* p, tx_pipeline_stats_t* out)
{
    if (!p || !out) return;
    rf10_fifo_get_stats(&p->txq, &out->txq);
}

size_t tx_pipeline_frame_samples(const tx_pipeline_t* p)
{
    return p ? (size_t)(p->tx_ctrl.frame_samples) : 0;
}

void tx_pipeline_reset_stats(tx_pipeline_t* p)
{
    if (p && p->inited) rf10_fifo_reset_stats(&p->txq);
}
