#include "nbfm_mod.h"
#include "audio_mod.h"
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

typedef struct { double b0,b1,b2,a1,a2,z1,z2; } am_tx_biquad_t;
static void am_tx_filter_init(am_tx_biquad_t* b,double hz,double q,int high)
{
    double w=2*M_PI*hz/48000,a=sin(w)/(2*q),d=1+a,sign=high?1:-1;
    *b=(am_tx_biquad_t){.b0=(1+sign*cos(w))/(2*d),.b1=-sign*(1+sign*cos(w))/d,
        .b2=(1+sign*cos(w))/(2*d),.a1=-2*cos(w)/d,.a2=(1-a)/d};
}
static float am_tx_filter(am_tx_biquad_t* b,float x)
{
    double y=b->b0*x+b->z1;
    b->z1=b->b1*x-b->a1*y+b->z2;b->z2=b->b2*x-b->a2*y;
    return (float)y;
}
struct nbfm_mod {
    double fs_a, fs_rf, f_dev, R, a_to_rf, k;
    float  out_scale; int lin;
    double phase, dphi_cur, dphi_next, interp_step, interp_acc;
    preemph_t pe;
    bool am; float am_depth; am_tx_biquad_t voice[3];
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
    m->interp_step=0; m->interp_acc=1;
    m->pe.x1=0;
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
    
    if(m->am) {
        x=am_tx_filter(&m->voice[0],x);
        x=am_tx_filter(&m->voice[2],am_tx_filter(&m->voice[1],x));
        m->dphi_next=m->am_depth*fmaxf(-1,fminf(1,x));
        return 1;
    }
    float xp=preemph_run(&m->pe,x); 
    m->dphi_next=m->k*(double)xp; 
    return 1;
}

static size_t generate_iq(nbfm_mod_t* m, iq16_t* dst, size_t N, size_t* held)
{
    const uint32_t L = 4000000;
    const uint32_t M = 48000 * m->divider;
    size_t out = 0;

    while (out < N) {
        // Advance audio time using the configured input/output rates.
        m->lm_phase += M;
        if (m->lm_phase >= L) {
            m->lm_phase -= L;

            // Move current -> next and fetch next audio-derived freq
            m->dphi_cur = m->dphi_next;
            if (!fetch_audio(m)) {
                // Preserve frequency hold, but report the underrun to explicit callers.
                if (held) ++*held;
                m->dphi_next = m->am ? 0 : m->dphi_cur;
            }
        }

        // Linear interpolation of frequency between audio ticks (optional)
        double frac = m->use_lin ? (double)m->lm_phase / (double)L : 0.0;
        double dphi = m->dphi_cur + (m->dphi_next - m->dphi_cur) * frac;

        if (m->am) {
            float amplitude=m->out_scale*.5f*(1+(float)dphi); dst[out++]=(iq16_t){(int16_t)lrintf(amplitude*cosf(AM_DIAG_PHASE)),(int16_t)lrintf(amplitude*sinf(AM_DIAG_PHASE))};
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

audio_mod_t* audio_mod_create(audio_mod_mode_t mode,const nbfm_cfg_t* config,float depth)
{
    uint32_t bits;memcpy(&bits,&depth,sizeof(bits));
    if ((mode!=AUDIO_MOD_NBFM && mode!=AUDIO_MOD_AM) ||
        (bits&0x7f800000u)==0x7f800000u || depth<0 || depth>1 ||
        (mode==AUDIO_MOD_AM && (!config || config->preemph_tau_s!=0))) {
        errno=EINVAL;return NULL;
    }
    nbfm_mod_t* m=nbfm_create(config);
    if (m && mode==AUDIO_MOD_AM) {
        m->am=true;m->am_depth=depth;
        am_tx_filter_init(&m->voice[0],300,.7071067811865476,1);
        am_tx_filter_init(&m->voice[1],3000,.541196100146197,0);
        am_tx_filter_init(&m->voice[2],3000,1.306562964876377,0);
    }
    return m;
}
