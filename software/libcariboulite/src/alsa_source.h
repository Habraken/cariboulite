#pragma once
#include <stddef.h>

typedef struct alsa_source alsa_source_t;

// device examples: "default", "hw:1,0"
alsa_source_t* alsa_source_create(const char* device, float gain);
size_t alsa_source_read(alsa_source_t* s, float* dst, size_t max_frames);
void   alsa_source_destroy(alsa_source_t* s);
#include "audio_source.h"
// Currently supports only 48000 Hz mono. Returns NULL and sets errno on failure.
audio_source_t* alsa_source_open(const char* device, float gain, audio_format_t format);
