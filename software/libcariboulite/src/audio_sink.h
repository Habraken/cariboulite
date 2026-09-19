#pragma once
#include <stddef.h>
#include <stdint.h>
#include <errno.h>

// Transitional RX boundary: signed 16-bit mono PCM, counts in frames.
// No conversion or level change at this boundary.
typedef enum { AUDIO_SINK_OK, AUDIO_SINK_AGAIN, AUDIO_SINK_ERROR } audio_sink_status_t;
typedef struct {
    size_t frames;
    audio_sink_status_t status;
    int error; // negative errno-style code on ERROR
} audio_sink_result_t;
typedef struct audio_sink audio_sink_t;
typedef struct {
    audio_sink_result_t (*write)(audio_sink_t*, const int16_t*, size_t);
    void (*destroy)(audio_sink_t*);
    const char* (*state)(audio_sink_t*); // optional diagnostic, static string
} audio_sink_ops_t;
struct audio_sink {
    unsigned sample_rate; // immutable; mono S16 input
    const audio_sink_ops_t* ops;
};

// Single writer. Caller owns samples; no pointer retained. May block.
static inline audio_sink_result_t audio_sink_write(audio_sink_t* s, const int16_t* src, size_t frames)
{
    if (!s || !s->ops || !s->ops->write || (!src && frames))
        return (audio_sink_result_t){0, AUDIO_SINK_ERROR, -EINVAL};
    if (!frames) return (audio_sink_result_t){0, AUDIO_SINK_OK, 0};
    return s->ops->write(s, src, frames);
}
// Stop/join writer and diagnostic readers first; NULL is allowed.
static inline void audio_sink_destroy(audio_sink_t* s)
{
    if (s) s->ops->destroy(s);
}
static inline const char* audio_sink_state(audio_sink_t* s)
{
    return s && s->ops->state ? s->ops->state(s) : "?";
}
