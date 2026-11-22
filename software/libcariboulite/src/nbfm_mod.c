#include "nbfm_mod.h"
#include "audio_mod.h"
#include "am_dac_shaper.h"
#include <stdbool.h>
#include "radio_rates.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include "math_compat.h"

typedef struct { double a0,a1; float x1; } preemph_t;
static void preemph_init(preemph_t* p,double fs,double tau){
    if(tau<=0){p->a0=1;p->a1=0;p->x1=0;return;}
    double T=1.0/fs, alpha=tau/(tau+T);
    p->a0=1.0+alpha; p->a1=-alpha; p->x1=0;
}
static inline float preemph_run(preemph_t* p,float x){
    float y=(float)p->a0*x+(float)p->a1*p->x1; p->x1=x; return y;
}

typedef struct { double b0,b1,b2,a1,a2,z1,z2; } voice_biquad_t;
static void voice_filter_init(voice_biquad_t* b,double hz,double q,int high)
{
    double w=2*M_PI*hz/48000,a=sin(w)/(2*q),d=1+a,sign=high?1:-1;
    *b=(voice_biquad_t){.b0=(1+sign*cos(w))/(2*d),.b1=-sign*(1+sign*cos(w))/d,
        .b2=(1+sign*cos(w))/(2*d),.a1=-2*cos(w)/d,.a2=(1-a)/d};
}
static float voice_filter(voice_biquad_t* b,float x)
{
    double y=b->b0*x+b->z1;
    b->z1=b->b1*x-b->a1*y+b->z2;b->z2=b->b2*x-b->a2*y;
    return (float)y;
}
// A Hamming-windowed Hilbert transformer at 48 kHz. Delay I by the
// same 192 samples as Q. Symmetry leaves 96 nonzero tap pairs.
enum { SSB_DELAY = 192, SSB_TAPS = 2 * SSB_DELAY + 1 };
typedef struct {
    float taps[SSB_DELAY / 2];
    float history[2 * SSB_TAPS];
    unsigned head;
    float q_cur, q_next;
    bool has_audio;
} ssb_tx_t;

static void ssb_reset(ssb_tx_t* ssb)
{
    memset(ssb->history, 0, sizeof(ssb->history));
    ssb->head = 0;
    ssb->q_cur = ssb->q_next = 0;
    ssb->has_audio = false;
}

static void ssb_audio(ssb_tx_t* ssb, float audio, bool lower, double* in_phase)
{
    // Mirror the ring so the convolution can read contiguous history.
    ssb->head = ssb->head ? ssb->head - 1 : SSB_TAPS - 1;
    float* history = &ssb->history[ssb->head];
    history[0] = history[SSB_TAPS] = audio;
    ssb->has_audio = true;
    float i = history[SSB_DELAY], q = 0;
    for (unsigned tap = 0; tap < SSB_DELAY / 2; ++tap) {
        unsigned offset = 2 * tap + 1;
        q += ssb->taps[tap] * (history[SSB_DELAY + offset] - history[SSB_DELAY - offset]);
    }

    // Limit the envelope before interpolation to bound peak IQ without
    // independently clipping I and Q.
    float magnitude_sq = i * i + q * q;
    if (magnitude_sq > 1) {
        float gain = 1 / sqrtf(magnitude_sq);
        i *= gain;
        q *= gain;
    }
    *in_phase = i;
    ssb->q_next = lower ? -q : q;
}

struct nbfm_mod {
    double fs_a, fs_rf, f_dev, R, a_to_rf, k;
    float  out_scale; int lin;
    double phase, dphi_cur, dphi_next, interp_step, interp_acc;
    preemph_t pe;
    float ctcss_hz, ctcss_deviation;
    bool ctcss_muted;
    double ctcss_phase;
    ctcss_voice_t ctcss_voice;
    audio_mod_mode_t mode;
    bool am_dac_shaping;
    float am_depth;
    voice_biquad_t voice[3];
    ssb_tx_t ssb;
    am_dac_shaper_t am_shaper;
    float* afifo; size_t cap, head, tail, cnt;
    
    //new
    uint32_t lm_phase, divider;
    int use_lin;
};

static int finite_double(double x)
{
    uint64_t bits;memcpy(&bits,&x,sizeof(bits));
    return (bits&UINT64_C(0x7ff0000000000000))!=UINT64_C(0x7ff0000000000000);
}
static int finite_float(float x)
{
    uint32_t bits;memcpy(&bits,&x,sizeof(bits));
    return (bits&0x7f800000u)!=0x7f800000u;
}
static unsigned native_divider(double hz)
{
    uint64_t bits;memcpy(&bits,&hz,sizeof(bits));
    if ((bits & UINT64_C(0x7ff0000000000000)) == UINT64_C(0x7ff0000000000000) ||
        hz < 400000 || hz > 4000000) return 0;
    unsigned d = radio_rate_divider((unsigned)hz);
    return d && (hz == (unsigned)hz || hz == 4000000.0 / d) ? d : 0;
}
static int valid_config(const nbfm_cfg_t* c)
{
    return finite_double(c->audio_fs) && c->audio_fs == 48000 &&
        native_divider(c->rf_fs) &&
        finite_double(c->f_dev_hz) && c->f_dev_hz >= 0 && c->f_dev_hz <= 24000 &&
        finite_double(c->preemph_tau_s) && c->preemph_tau_s >= 0 &&
        finite_float(c->out_scale) && c->out_scale >= 0 && c->out_scale <= 32767 &&
        (c->linear_interp == 0 || c->linear_interp == 1);
}
nbfm_mod_t* nbfm_create(const nbfm_cfg_t* config)
{
    const nbfm_cfg_t defaults = {48000, 4000000, NBFM_DEFAULT_DEVIATION_HZ, 0, 12000, 1};
    const nbfm_cfg_t* c = config ? config : &defaults;
    if (!valid_config(c)) { errno = EINVAL; return NULL; }
    nbfm_mod_t* m = calloc(1, sizeof(*m));
    if (!m) { errno = ENOMEM; return NULL; }
    m->divider=native_divider(c->rf_fs);
    m->fs_a=c->audio_fs; m->fs_rf=4000000.0/m->divider;
    m->f_dev=c->f_dev_hz; m->out_scale=c->out_scale;
    m->lin=c->linear_interp; m->R=m->fs_rf/m->fs_a; m->a_to_rf=1.0/m->R;
    m->k=2.0*M_PI*m->f_dev/m->fs_rf;
    preemph_init(&m->pe,m->fs_a,c->preemph_tau_s);
    m->interp_acc=1.0; m->cap=4096;
    m->afifo=calloc(m->cap,sizeof(float));
    if (!m->afifo) { free(m); errno = ENOMEM; return NULL; }
    m->lm_phase=0; m->use_lin=m->lin;
    return m;
}
void nbfm_reset(nbfm_mod_t* m)
{
    if (!m) return;
    m->head=m->tail=m->cnt=0;
    m->phase=m->dphi_cur=m->dphi_next=0; m->lm_phase=0;
    memset(&m->am_shaper, 0, sizeof(m->am_shaper));
    m->interp_step=0; m->interp_acc=1;
    m->pe.x1=0;
    audio_mod_reset_ctcss(m);
    ssb_reset(&m->ssb);
    for(unsigned k=0;k<3;k++)m->voice[k].z1=m->voice[k].z2=0;
}
size_t nbfm_buffered_audio(const nbfm_mod_t* m) { return m ? m->cnt : 0; }
static int valid_audio(const float* audio, size_t frames)
{
    if (!audio && frames) return 0;
    for (size_t n=0; n<frames; ++n) if (!finite_float(audio[n])) return 0;
    return 1;
}

void nbfm_destroy(nbfm_mod_t* m) { 
    if(!m)return; 
    free(m->afifo); 
    free(m); 
}

static size_t enqueue_audio(nbfm_mod_t* m,const float* a,size_t N) {
    size_t p=0; 
    for(size_t n=0;n<N;n++) {
        if(m->cnt == m->cap) break;
        m->afifo[m->tail] = a[n]; 
        m->tail=(m->tail+1)%m->cap; 
        m->cnt++; p++; 
    } 
    return p;
}
size_t nbfm_push_audio(nbfm_mod_t* m, const float* audio, size_t frames)
{
    if (!m || !valid_audio(audio, frames)) { errno = EINVAL; return 0; }
    return enqueue_audio(m, audio, frames);
}
static int fetch_audio(nbfm_mod_t* m) {
    if (m->cnt == 0) return 0;

    float x = m->afifo[m->head];
    m->head = (m->head+1)%m->cap; 
    m->cnt--; 
    
    if (x>1) x=1; 
    else if (x<-1) x=-1;
    
    if (m->mode != AUDIO_MOD_NBFM) {
        x=voice_filter(&m->voice[0],x);
        x=voice_filter(&m->voice[2],voice_filter(&m->voice[1],x));
        if (m->mode == AUDIO_MOD_AM)
            m->dphi_next = m->am_depth * fmaxf(-1, fminf(1, x));
        else
            ssb_audio(&m->ssb, x, m->mode == AUDIO_MOD_LSB, &m->dphi_next);
        return 1;
    }
    if (m->ctcss_hz) x=ctcss_voice_sample(&m->ctcss_voice,x);
    float xp=preemph_run(&m->pe,x);
    double deviation=m->f_dev*xp;
    if (m->ctcss_hz) {
        double headroom=m->f_dev-m->ctcss_deviation;
        deviation=fmax(-headroom,fmin(headroom,deviation));
        if (!m->ctcss_muted) deviation+=m->ctcss_deviation*sin(m->ctcss_phase);
        m->ctcss_phase+=2*M_PI*m->ctcss_hz/48000;
        if (m->ctcss_phase>=2*M_PI) m->ctcss_phase-=2*M_PI;
        m->dphi_next=2*M_PI*deviation/m->fs_rf;
    } else m->dphi_next=m->k*(double)xp;
    return 1;
}

static size_t generate_iq(nbfm_mod_t* m, iq16_t* dst, size_t N, size_t* held)
{
    const uint32_t L = 4000000;
    const uint32_t M = 48000 * m->divider;
    size_t out = 0;
    const bool ssb = m->mode == AUDIO_MOD_USB || m->mode == AUDIO_MOD_LSB;

    while (out < N) {
        // Advance audio time using the configured input/output rates.
        m->lm_phase += M;
        if (m->lm_phase >= L) {
            m->lm_phase -= L;

            // Advance interpolation endpoints at the audio clock.
            m->dphi_cur = m->dphi_next;
            if (ssb) m->ssb.q_cur = m->ssb.q_next;
            if (!fetch_audio(m)) {
                // Report the underrun and interpolate voice modes toward silence.
                if (held) ++*held;
                m->dphi_next = m->mode == AUDIO_MOD_NBFM ? m->dphi_cur : 0;
                if (ssb && m->ssb.has_audio) {
                    float q_cur = m->ssb.q_cur;
                    ssb_reset(&m->ssb);
                    m->ssb.q_cur = q_cur;
                    for (unsigned k = 0; k < 3; ++k) m->voice[k].z1 = m->voice[k].z2 = 0;
                }
            }
        }

        // Linear interpolation between audio ticks (optional).
        double frac = m->use_lin ? (double)m->lm_phase / (double)L : 0.0;
        double dphi = m->dphi_cur + (m->dphi_next - m->dphi_cur) * frac;

        if (ssb) {
            double q = m->ssb.q_cur + (m->ssb.q_next - m->ssb.q_cur) * frac;
            dst[out++] = (iq16_t){(int16_t)lrint(dphi * m->out_scale),
                                  (int16_t)lrint(q * m->out_scale)};
            continue;
        }
        if (m->mode == AUDIO_MOD_AM) {
            float envelope=m->out_scale*.5f*(1+(float)dphi);
            int16_t value=m->am_dac_shaping
                ? am_dac_shaper_process(&m->am_shaper,envelope,m->divider)
                : (int16_t)lrintf(envelope);
            dst[out++]=(iq16_t){value,0};
            continue;
        }
        // Integrate to phase with robust wrap
        m->phase += dphi;
        if (m->phase >  M_PI) m->phase -= 2.0 * M_PI;
        if (m->phase < -M_PI) m->phase += 2.0 * M_PI;

        // IQ is quantized to int16; float trig is sufficient at this final step.
        // Keep phase and rate integration double to avoid accumulating drift.
        float ci = cosf((float)m->phase);
        float sq = sinf((float)m->phase);
        int32_t I = (int32_t)lrintf(ci * m->out_scale);
        int32_t Q = (int32_t)lrintf(sq * m->out_scale);
        if (I >  32767) I =  32767; else if (I < -32768) I = -32768;
        if (Q >  32767) Q =  32767; else if (Q < -32768) Q = -32768;

        dst[out].i = (int16_t)I;
        dst[out].q = (int16_t)Q;
        out++;
    }
    return out;
}
size_t nbfm_pull_iq(nbfm_mod_t* m, iq16_t* dst, size_t frames)
{
    if (!m || (!dst && frames)) { errno = EINVAL; return 0; }
    return generate_iq(m, dst, frames, NULL);
}
nbfm_result_t nbfm_process(nbfm_mod_t* m, const audio_f32_t* audio, size_t frames,
                           iq16_t* output, size_t capacity)
{
    nbfm_result_t result = {0};
    if (!m || (!output && capacity) || !valid_audio(audio, frames)) {
        result.error = -EINVAL;
        return result;
    }
    result.consumed = enqueue_audio(m, audio, frames);
    result.produced = generate_iq(m, output, capacity, &result.held_audio);
    return result;
}

void audio_mod_reset_ctcss(audio_mod_t* m)
{
    if (!m) return;
    m->ctcss_phase=0; m->ctcss_muted=false;
    memset(&m->ctcss_voice,0,sizeof(m->ctcss_voice));
}
void audio_mod_mute_ctcss(audio_mod_t* m, bool muted)
{ if (m) m->ctcss_muted=muted; }

int audio_mod_set_ctcss(audio_mod_t* m, float hz, float deviation)
{
    if (!m || !ctcss_frequency_valid(hz) || !ctcss_deviation_valid(deviation) ||
        (hz && m->mode==AUDIO_MOD_NBFM && deviation>=m->f_dev)) { errno=EINVAL; return -1; }
    m->ctcss_hz=m->mode==AUDIO_MOD_NBFM ? hz : 0;
    m->ctcss_deviation=deviation;
    audio_mod_reset_ctcss(m);
    return 0;
}

int audio_mod_set_am_dac_shaping(audio_mod_t* m, bool enabled)
{
    if (!m || m->mode != AUDIO_MOD_AM || (enabled && m->out_scale > 4032)) {
        errno=EINVAL;
        return -1;
    }
    m->am_dac_shaping=enabled;
    memset(&m->am_shaper,0,sizeof(m->am_shaper));
    return 0;
}

audio_mod_t* audio_mod_create(audio_mod_mode_t mode,const nbfm_cfg_t* config,float depth)
{
    uint32_t bits;memcpy(&bits,&depth,sizeof(bits));
    if (!audio_mod_mode_valid(mode) ||
        (bits&0x7f800000u)==0x7f800000u || depth<0 || depth>1 ||
        (mode!=AUDIO_MOD_NBFM && (!config || config->preemph_tau_s!=0))) {
        errno=EINVAL;return NULL;
    }
    nbfm_mod_t* m=nbfm_create(config);
    if (!m) return NULL;
    m->mode = mode;
    m->am_depth = depth;
    if (mode != AUDIO_MOD_NBFM) {
        voice_filter_init(&m->voice[0],300,.7071067811865476,1);
        voice_filter_init(&m->voice[1],3000,.541196100146197,0);
        voice_filter_init(&m->voice[2],3000,1.306562964876377,0);
    }
    if (mode == AUDIO_MOD_USB || mode == AUDIO_MOD_LSB) {
        for (unsigned tap = 0; tap < SSB_DELAY / 2; ++tap) {
            unsigned offset = 2 * tap + 1;
            double window = 0.54 + 0.46 * cos(M_PI * offset / SSB_DELAY);
            m->ssb.taps[tap] = (float)(2 * window / (M_PI * offset));
        }
    }
    return m;
}
