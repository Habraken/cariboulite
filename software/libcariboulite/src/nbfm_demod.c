// Compatibility facade: dispatch once per block; mode state is private.
#include "fm_demod_internal.h"
#include <stdlib.h>
nbfm_demod_t* nbfm_demod_create(const nbfm_demod_config_t* c) { return nb_demod_create(c); }
nbfm_demod_t* wbfm_demod_create(const nbfm_demod_config_t* c) { return wb_demod_create(c); }
void nbfm_demod_destroy(nbfm_demod_t* s) { free(s); }
void nbfm_demod_reset(nbfm_demod_t* s)
{
    if (!s) return;
    if (s->mode == FM_MODE_WBFM) wb_demod_reset(s); else nb_demod_reset(s);
}
int nbfm_demod_set_audio(nbfm_demod_t* s, float tau, float gain)
{
    return s && s->mode == FM_MODE_WBFM ? wb_demod_set_audio(s,tau,gain) : nb_demod_set_audio(s,tau,gain);
}
nbfm_demod_result_t nbfm_demod_process_with_raw(nbfm_demod_t* s,
    const iq16_t* input, size_t count, int16_t* output, float* raw,
    size_t capacity, double correction)
{
    return s && s->mode == FM_MODE_WBFM
        ? wb_demod_process_with_raw(s,input,count,output,raw,capacity,correction)
        : nb_demod_process_with_raw(s,input,count,output,raw,capacity,correction);
}
nbfm_demod_result_t nbfm_demod_process(nbfm_demod_t* s,
    const iq16_t* input, size_t count, int16_t* output, size_t capacity,
    double correction)
{
    return nbfm_demod_process_with_raw(s,input,count,output,NULL,capacity,correction);
}
