"""Build diagnostic-only copies; never opens a radio or modifies installed apps."""
from pathlib import Path
import hashlib,json,re,subprocess
root=Path('/home/pi/src/cariboulite')
out=root/'build/demod-audio-20261125';out.mkdir(exist_ok=True)
src=root/'software/libcariboulite/src';here=root/'software/libcariboulite/tools/radio_backend'
text=(here/'main.c').read_text();old='request->tx_tone_amp <= 0';assert text.count(old)==1
(out/'main.c').write_text(text.replace(old,'request->tx_tone_amp < 0'))
flags=['cc','-D_GNU_SOURCE','-D_POSIX_C_SOURCE=200809L','-std=c11','-O2','-g','-Wall','-Wextra','-Wno-unused-parameter','-Wno-missing-braces','-I'+str(src),'-I'+str(here)]
subprocess.run([*flags,'-c',str(out/'main.c'),'-o',str(out/'main.o')],check=True)
subprocess.run([*flags,'-Werror','-c',str(out/'capture.c'),'-o',str(out/'capture.o')],check=True)
make=(here/'Makefile').read_text();names=[]
for group in ('BASE_NAMES','OWN_NAMES'):names+=re.search(r'^'+group+r' := (.*)$',make,re.M).group(1).split()
objects=[out/'main.o' if name=='main' else root/'build/radio-backend'/(name+'.o') for name in names]+[root/'build/radio-backend/modem_snapshot_shared.o',out/'capture.o']
subprocess.run(['cc',*map(str,objects),'-Wl,--wrap=audio_demod_process_with_taps','-L'+str(root/'build'),'-Wl,-rpath,'+str(root/'build'),'-lcariboulite','-lasound','-lpthread','-lm','-ldl','-o',str(out/'backend')],check=True)
meta={'diagnostic_changes':['Permit zero-amplitude synthetic tone','Bounded RAM capture of unchanged IQ/raw/filtered/accepted PCM; save after shutdown'],
 'installed_backend_sha256':hashlib.sha256((root/'build/radio-backend/cariboulite_radio_backend').read_bytes()).hexdigest(),
 'library_sha256':hashlib.sha256((root/'build/libcariboulite.so').read_bytes()).hexdigest(),
 'diagnostic_backend_sha256':hashlib.sha256((out/'backend').read_bytes()).hexdigest(),
 'capture_source_sha256':hashlib.sha256((out/'capture.c').read_bytes()).hexdigest()}
(out/'build.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta),flush=True)
