#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include "alsa_sink.h"
#include <alsa/asoundlib.h>

static int opens, closes, stereo, fail_sw, fail_prepare, writes;
static snd_pcm_sframes_t next_write;
static const int16_t* expected;
static size_t expected_frames;
int __real_snd_pcm_open(snd_pcm_t**, const char*, snd_pcm_stream_t, int);
int __wrap_snd_pcm_open(snd_pcm_t** p, const char* d, snd_pcm_stream_t st, int mode) {
    ++opens; return __real_snd_pcm_open(p,d,st,mode);
}
int __real_snd_pcm_close(snd_pcm_t*);
int __wrap_snd_pcm_close(snd_pcm_t* p) { ++closes; return __real_snd_pcm_close(p); }
int __real_snd_pcm_hw_params_set_channels(snd_pcm_t*, snd_pcm_hw_params_t*, unsigned);
int __wrap_snd_pcm_hw_params_set_channels(snd_pcm_t* p, snd_pcm_hw_params_t* hw, unsigned ch) {
    if (stereo && ch==1) return -EINVAL;
    return __real_snd_pcm_hw_params_set_channels(p,hw,ch);
}
int __real_snd_pcm_sw_params(snd_pcm_t*, snd_pcm_sw_params_t*);
int __wrap_snd_pcm_sw_params(snd_pcm_t* p, snd_pcm_sw_params_t* sw) {
    return fail_sw ? -EIO : __real_snd_pcm_sw_params(p,sw);
}
int __real_snd_pcm_prepare(snd_pcm_t*);
int __wrap_snd_pcm_prepare(snd_pcm_t* p) {
    return fail_prepare ? -EIO : __real_snd_pcm_prepare(p);
}
snd_pcm_sframes_t __wrap_snd_pcm_writei(snd_pcm_t* p, const void* data, snd_pcm_uframes_t frames) {
    (void)p; ++writes;
    assert(frames==expected_frames);
    const int16_t* v=data;
    for(size_t i=0;i<frames;++i) {
        assert(v[i*(stereo?2:1)]==expected[i]);
        if(stereo) assert(v[2*i+1]==expected[i]);
    }
    return next_write;
}
static void* retry_writer(void* arg) {
    int16_t samples[480]={0};
    // Use the production pipeline helper, included by the Python runner.
    extern int test_write_exact(audio_sink_t*, const int16_t*, size_t);
    test_write_exact(arg,samples,480);
    return NULL;
}
int main(void) {
    assert(!alsa_sink_open("null",44100) && errno==EINVAL && opens==0);
    int16_t samples[960];
    for(int i=0;i<960;++i) samples[i]=(int16_t)(i*67-32768);
    for(stereo=0;stereo<=1;++stereo) {
        audio_sink_t* s=alsa_sink_open("null",48000); assert(s);
        assert(alsa_sink_channels(s)==(unsigned)(stereo?2:1));
        assert(audio_sink_state(s));
        assert(audio_sink_write(s,NULL,1).error==-EINVAL);
        int before=writes;
        assert(audio_sink_write(s,NULL,0).status==AUDIO_SINK_OK && writes==before);
        expected=samples; expected_frames=stereo?480:960; next_write=123;
        audio_sink_result_t r=audio_sink_write(s,samples,960);
        assert(r.frames==123 && r.status==AUDIO_SINK_OK);
        expected=samples+123; expected_frames=stereo?480:837; next_write=37;
        r=audio_sink_write(s,samples+123,837); assert(r.frames==37);
        expected=samples; expected_frames=480;
        next_write=-EPIPE; r=audio_sink_write(s,samples,480);
        assert(r.frames==0 && r.status==AUDIO_SINK_AGAIN);
        fail_prepare=1; r=audio_sink_write(s,samples,480);
        assert(r.status==AUDIO_SINK_ERROR && r.error==-EIO); fail_prepare=0;
        next_write=-EAGAIN; assert(audio_sink_write(s,samples,480).status==AUDIO_SINK_AGAIN);
        next_write=0; assert(audio_sink_write(s,samples,480).status==AUDIO_SINK_AGAIN);
        next_write=-ENODEV; assert(audio_sink_write(s,samples,480).error==-ENODEV);
        audio_sink_destroy(s);
    }
    stereo=0;
    fail_sw=1; assert(!alsa_sink_open("null",48000) && errno==EIO); fail_sw=0;
    fail_prepare=1; assert(!alsa_sink_open("null",48000) && errno==EIO); fail_prepare=0;
    assert(opens==closes);
    audio_sink_t* s=alsa_sink_open("null",48000); assert(s);
    int16_t zeros[480]={0}; expected=zeros; expected_frames=480; next_write=-EAGAIN;
    pthread_t thread; assert(!pthread_create(&thread,NULL,retry_writer,s));
    usleep(10000); pthread_cancel(thread);
    void* result; pthread_join(thread,&result); assert(result==PTHREAD_CANCELED);
    next_write=0;
    assert(!pthread_create(&thread,NULL,retry_writer,s));
    usleep(10000); pthread_cancel(thread); pthread_join(thread,&result);
    assert(result==PTHREAD_CANCELED);
    audio_sink_destroy(s); audio_sink_destroy(NULL); assert(opens==closes);
    puts("PASS: sink PCM preservation, stereo fallback, partial writes, recovery, errors, cleanup and retry cancellation");
}
