#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "demod_worker.h"
#include "pipeline_transport.h"
#include "pipeline_runtime.h"
#include <math.h>
#include <stdio.h>
#include <time.h>

void* nbfm_demod_thread(void* arg)
{
    pthread_setname_np(pthread_self(), "nbfm_demod_thread");
    set_rt_and_affinity_prio(55, 1);
    nbfm_demod_ctrl_t* c = arg;
    if (!c || !c->fifo_in || !c->sink || !c->dsp) return NULL;

    // The FIFO carries complete 10 ms RF blocks. Sample depth immediately before
    // processing the last IQ pair: this is the same 500th 50k interval at which
    // the previous worker updated correction, before its resampler output.
    const size_t frame_samples = (size_t)(c->fs_rf / 100);
    double corr48 = 0.0, depth_ema = 0.0;
    int primed = 0;
    size_t nout = 0;
    aud10_frame_t audio;
    uint64_t last_log_ms = 0;
    if (c->prime_blocks_10ms <= 0) c->prime_blocks_10ms = 8;
    c->reset = true;
    while (c->active) {
        if (c->reset) {
            nbfm_demod_reset(c->dsp);
            corr48 = depth_ema = 0.0;
            primed = 0;
            nout = 0;
            c->priming = c->prime_blocks_10ms > 0;
            c->reset = false;
        }
        rf10_frame_t frm;
        if (!rf10_fifo_get(c->fifo_in, &frm, -1)) continue;
        size_t offset = 0;
        while (offset < frame_samples) {
            size_t count = frame_samples - 1 - offset;
            if (!count) {
                const double alpha = 0.05;
                const double ki = 2.0e-4;
                const double corr_ppm_cap = 3.0e-4;
                const double corr_ppm_slew = 1.0e-5;
                const double target_fill = 0.50;
                const double deadband = 0.01;
                size_t acnt = 0, acap = 0;
                aud10_fifo_peek_depth(c->afifo_out, &acnt, &acap);
                const double fill = (acap ? (double)acnt / (double)acap : 0.0);

                // smooth depth
                if (depth_ema == 0.0) depth_ema = (double)acnt;  // init on first call
                depth_ema = (1.0 - alpha) * depth_ema + alpha * (double)acnt;

                // engage after we’re in the neighborhood (prevents big initial pulls)
                if (!primed) {
                    if (fill >= 0.35) primed = 1;   // start controlling once buffer >35%
                }

                double err = 0.0;
                if (primed) {
                    const double target = target_fill * (double)acap;
                    const double err_raw = (double)depth_ema - target; // +err => overfilling
                    if (fabs(err_raw) > deadband * (double)acap)
                        err = err_raw;
                }

                // integral update with slew limit
                double corr_prev = corr48;
                if (acap) corr48 += -ki * (err / (double)acap);  // unitless; negative feedback

                // slew limit (per update) to avoid pitch steps
                double dc = corr48 - corr_prev;
                if (dc >  corr_ppm_slew) corr48 = corr_prev + corr_ppm_slew;
                if (dc < -corr_ppm_slew) corr48 = corr_prev - corr_ppm_slew;

                // hard clamp
                if (corr48 >  corr_ppm_cap) corr48 =  corr_ppm_cap;
                if (corr48 < -corr_ppm_cap) corr48 = -corr_ppm_cap;

                // emergency nudges
                if (fill > 0.95) corr48 = fmin(corr48, -5e-4);
                if (fill < 0.05) corr48 = fmax(corr48,  5e-4);
                count = 1;
            }
            if (nbfm_demod_set_audio(c->dsp, c->deemph_tau, c->pcm_gain) != 0) {
                fprintf(stderr, "DEMOD: invalid audio configuration\n");
                return NULL;
            }
            nbfm_demod_result_t result = nbfm_demod_process(c->dsp,
                frm.data + offset, count, audio.pcm + nout, 480 - nout, corr48);
            if (result.error || !result.consumed) {
                fprintf(stderr, "DEMOD: processing failed (%d)\n", result.error);
                return NULL;
            }
            offset += result.consumed;
            nout += result.produced;
            if (nout == 480) {
                aud10_fifo_put(c->afifo_out, &audio, 10);
                c->pcm_total_frames += 480;
                nout = 0;
                struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t ms = (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
                if (!last_log_ms) last_log_ms = ms;
                if (ms - last_log_ms >= 1000) {
                    size_t acnt = 0, acap = 0;
                    aud10_fifo_peek_depth(c->afifo_out, &acnt, &acap);
                    fprintf(stderr,
                        "DEMOD: frames=%llu (%.1fs) ALSA=%s  aud_fifo=%zu/%zu (%.0f%%)  corr=%.5f\n",
                        (unsigned long long)c->pcm_total_frames,
                        (double)c->pcm_total_frames / (double)c->pcm_rate,
                        audio_sink_state(c->sink), acnt, acap,
                        100.0 * (double)acnt / (double)acap, corr48);
                    last_log_ms = ms;
                }
            }
        }
    }
    return NULL;
}
