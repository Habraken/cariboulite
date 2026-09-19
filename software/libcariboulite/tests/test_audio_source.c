// Exercise actual ALSA adapter buffering with scripted capture; no audio hardware.
#include <assert.h>
#include <errno.h>
#include "alsa_source.c"

static int reads;
snd_pcm_sframes_t __wrap_snd_pcm_readi(snd_pcm_t* pcm, void* data, snd_pcm_uframes_t frames)
{
    (void)pcm;
    assert(frames == 2);
    if (reads++ == 0) {
        int16_t* samples = data;
        samples[0] = 16384; samples[1] = -16384;
        return 2;
    }
    return -EIO;
}

int main(void)
{
    errno = 0;
    assert(!alsa_source_open("must-not-open", 1, (audio_format_t){44100, 1}));
    assert(errno == EINVAL);
    assert(!alsa_source_open("must-not-open", 1, (audio_format_t){48000, 2}));
    float output[4] = {0}, ring[16] = {0};
    int16_t capture[2];
    alsa_source_t fixture = { .base = {{48000, 1}, &alsa_audio_ops},
        .gain = 1, .period = 2, .rcap = 16, .ring = ring, .cap_i16 = capture };
    audio_source_result_t r = audio_source_read(&fixture.base, output, 4);
    assert(r.frames == 2 && r.status == AUDIO_SOURCE_ERROR && r.error == -EIO);
    assert(output[0] == 0.5f && output[1] == -0.5f);
    r = audio_source_read(&fixture.base, output, 4);
    assert(r.frames == 0 && r.status == AUDIO_SOURCE_ERROR);
    reads = 0;
    r = audio_source_read(&fixture.base, output, 1);
    assert(r.frames == 1 && r.status == AUDIO_SOURCE_OK && r.error == 0);
    r = audio_source_read(&fixture.base, output, 1);
    assert(r.frames == 1 && output[0] == -0.5f && reads == 1);
    assert(audio_source_read(NULL, output, 1).status == AUDIO_SOURCE_ERROR);
    assert(audio_source_read(&fixture.base, NULL, 1).status == AUDIO_SOURCE_ERROR);
    assert(audio_source_read(&fixture.base, NULL, 0).status == AUDIO_SOURCE_OK);
    audio_source_destroy(NULL);
    puts("PASS: source format validation, partial/error results, conversion and buffered reads");
}
