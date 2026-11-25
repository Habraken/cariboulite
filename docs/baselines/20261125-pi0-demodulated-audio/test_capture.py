"""Native mocked forwarding/partial-write/bounds test. No hardware access."""
from pathlib import Path
import json,os,subprocess,tempfile
root=Path('/home/pi/src/cariboulite');out=root/'build/demod-audio-20261125'
src=root/'software/libcariboulite/src'
(out/'mock.c').write_text(r'''
#include "audio_demod.h"
#include <alsa/asoundlib.h>
#include <assert.h>
static unsigned iq_calls,audio_calls,pcm_calls;
int cariboulite_radio_read_samples(void*r,void*b,void*m,size_t n){(void)r;(void)m;++iq_calls;short*p=b;for(size_t k=0;k<2*n;k++)p[k]=(short)k;return n/2;}
audio_demod_result_t audio_demod_process_with_taps(audio_demod_t*d,const iq16_t*in,size_t n,audio_s16_t*out,float*raw,float*filtered,size_t cap,double corr){(void)d;(void)in;++audio_calls;assert(corr==.0001);for(size_t k=0;k<cap;k++){out[k]=(short)k;raw[k]=.25f;filtered[k]=.125f;}return (audio_demod_result_t){n,cap,0};}
snd_pcm_sframes_t snd_pcm_writei(snd_pcm_t*p,const void*b,snd_pcm_uframes_t n){(void)p;(void)b;++pcm_calls;return n/2;}
int snd_pcm_hw_params_current(snd_pcm_t*p,snd_pcm_hw_params_t*h){(void)p;(void)h;return 0;}
int snd_pcm_hw_params_get_channels(const snd_pcm_hw_params_t*h,unsigned*n){(void)h;*n=2;return 0;}
void check_counts(void){assert(iq_calls==101&&audio_calls==101&&pcm_calls==101);}
''')
(out/'mock_main.c').write_text(r'''
#include "audio_demod.h"
#include <alsa/asoundlib.h>
#include <assert.h>
#include <signal.h>
int cariboulite_radio_read_samples(void*,void*,void*,size_t);
void check_counts(void);
int main(void){short iq[160],pcm[16]={0};float raw[8],filtered[8];
 for(unsigned n=0;n<101;n++){
  if(n==1)raise(SIGUSR1);
  assert(cariboulite_radio_read_samples(0,iq,0,80)==40);assert(iq[159]==159);
  audio_demod_result_t r=audio_demod_process_with_taps(0,(iq16_t*)iq,80,pcm,raw,filtered,8,.0001);
  assert(r.consumed==80&&r.produced==8&&!r.error&&raw[7]==.25f&&filtered[7]==.125f&&pcm[7]==7);
  assert(snd_pcm_writei(0,pcm,8)==4);
 }check_counts();return 0;}
''')
flags=['cc','-std=c11','-O2','-D_GNU_SOURCE','-Wall','-Wextra','-Werror','-I'+str(src)]
subprocess.run([*flags,'-shared','-fPIC',str(out/'mock.c'),'-o',str(out/'libcapturemock.so')],check=True)
subprocess.run([*flags,str(out/'mock_main.c'),str(out/'capture.o'),'-Wl,--wrap=audio_demod_process_with_taps',
 '-L'+str(out),'-Wl,-rpath,'+str(out),'-lcapturemock','-lasound','-ldl','-o',str(out/'test-capture')],check=True)
with tempfile.TemporaryDirectory(prefix='demod-capture-mock-') as directory:
 p=Path(directory)/'capture'
 env=dict(os.environ,CL_CAPTURE_PREFIX=str(p),CL_CAPTURE_IQ_SAMPLES='128',CL_CAPTURE_AUDIO_FRAMES='64')
 subprocess.run([str(out/'test-capture')],env=env,check=True)
 info=json.loads(p.with_suffix('.json').read_text())
 assert info['iq_samples']==128 and info['raw_frames']==info['filtered_frames']==info['pcm_frames']==64
 assert info['channels']==2 and info['clock_chunks']==8
 for suffix,size in (('.iq',512),('.raw.f32',256),('.filtered.f32',256),('.pcm',256)):
  assert p.with_suffix(suffix).stat().st_size==size
 chunks=[json.loads(line) for line in p.with_suffix('.clock.jsonl').read_text().splitlines()]
 assert [c['first_sample'] for c in chunks]==list(range(0,64,8))
 assert all(c['frames']==8 and c['correction']==.0001 for c in chunks)
(out/'capture-test.json').write_text(json.dumps({'passed':True,'checks':['unchanged forwarded calls and outputs','signal arm excludes prior samples','bounded IQ/raw/filtered/PCM capture','partial ALSA writes count accepted frames only','complete correction trace']},indent=2))
print('PASS capture forwarding, signal arming, bounds, partial writes, clock metadata; no hardware used.')
