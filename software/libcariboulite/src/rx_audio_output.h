#pragma once
// Shared RX playback stage. Raw squelch/SINAD taps never pass through here.
#include "audio_demod.h"
#include "math_compat.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    float gain, target, step;
    unsigned ramp_left, count;
    double power;
    float peak;
    uint64_t clipped;
} rx_audio_output_t;

static inline bool rx_audio_volume_valid(float db)
{
    uint32_t bits;
    memcpy(&bits, &db, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u &&
        (db == -120.f || (db >= -60.f && db <= 12.f));
}

static inline double rx_audio_one_pole(double a, double w)
{
    return (1-a)/sqrt(1+a*a-2*a*cos(w));
}
static inline double rx_audio_biquad(double hz, double q, bool high)
{
    const double w=2*M_PI*hz/48000, a=sin(w)/(2*q), d=1+a;
    const double sign=high?1:-1, b0=(1+sign*cos(w))/(2*d);
    const double b1=-2*sign*b0, b2=b0;
    const double a1=-2*cos(w)/d, a2=(1-a)/d;
    const double t=2*M_PI*1000/48000;
    double nr=b0+b1*cos(t)+b2*cos(2*t), ni=-b1*sin(t)-b2*sin(2*t);
    double dr=1+a1*cos(t)+a2*cos(2*t), di=-a1*sin(t)-a2*sin(2*t);
    return sqrt((nr*nr+ni*ni)/(dr*dr+di*di));
}
// Reference: 1 kHz sine at 50% nominal modulation -> -12 dBFS RMS at 0 dB
// volume and legacy trim 8000. Full modulation has about 3 dB peak headroom.
// Compensate the audio filters at 1 kHz, preserving their frequency response.
static inline float rx_audio_calibration(audio_demod_mode_t mode, float tau)
{
    double response;
    const double w=2*M_PI*1000/48000;
    if (mode==AUDIO_DEMOD_AM || audio_demod_is_ssb(mode)) {
        response=rx_audio_biquad(300,.7071067811865476,true)*
            rx_audio_biquad(3000,.541196100146197,false)*
            rx_audio_biquad(3000,1.306562964876377,false);
    } else {
        double a=exp(-2*M_PI*5/48000);
        response=sqrt(2-2*cos(w))/sqrt(1+a*a-2*a*cos(w));
        if (tau>0) response*=rx_audio_one_pole(exp(-1/(48000.*tau)),w);
        if (mode==AUDIO_DEMOD_NBFM)
            response*=rx_audio_one_pole(exp(-2*M_PI*3200/48000),w);
    }
    return (float)(sqrt(2.)*32768*pow(10.,-12./20)/(.5*response));
}
static inline void rx_audio_set_gain(rx_audio_output_t* s, float target)
{
    if (target==s->target) return;
    s->target=target;
    s->ramp_left=960; // 20 ms linear ramp; no discontinuity on live edits
    s->step=(target-s->gain)/960;
}
// Meter actual, gated and clipped PCM over non-overlapping 100 ms windows.
// Packed RMS/peak milli-dBFS + 120000 (17 bits each); bit 63 = valid.
// Silence is reported as the explicit -120 dBFS floor.
static inline int16_t rx_audio_output_sample(rx_audio_output_t* s, float input,
                                            float gate, uint64_t* levels)
{
    if (s->ramp_left && !--s->ramp_left) s->gain=s->target;
    else if (s->ramp_left) s->gain+=s->step;
    float pcm=input*s->gain*gate;
    if (pcm>32767) { pcm=32767; ++s->clipped; }
    if (pcm< -32768) { pcm=-32768; ++s->clipped; }
    int16_t out=(int16_t)lrintf(pcm);
    const float y=out/32768.f;
    s->power+=(double)y*y;
    s->peak=fmaxf(s->peak,fabsf(y));
    if (++s->count==4800) {
        double rms=sqrt(s->power/4800);
        unsigned r=(unsigned)lround(fmax(-120.,20*log10(fmax(rms,1e-6))) * 1000+120000);
        unsigned p=(unsigned)lround(fmax(-120.,20*log10(fmax(s->peak,1e-6))) * 1000+120000);
        *levels=UINT64_C(1)<<63 | (uint64_t)p<<17 | r;
        s->count=0; s->power=0; s->peak=0;
    }
    return out;
}
