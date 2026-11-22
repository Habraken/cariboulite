"""Build an isolated backend permitting zero-amplitude synthetic tone."""
from pathlib import Path
import subprocess,re,json,hashlib
root=Path('/home/pi/src/cariboulite');out=root/'build/whistle-bypass';out.mkdir(exist_ok=True)
here=root/'software/libcariboulite/tools/radio_backend';src=root/'software/libcariboulite/src'
text=(here/'main.c').read_text();old='request->tx_tone_amp <= 0';assert text.count(old)==1
(out/'main.c').write_text(text.replace(old,'request->tx_tone_amp < 0'))
subprocess.run(['cc','-D_GNU_SOURCE','-D_POSIX_C_SOURCE=200809L','-std=c11','-O3','-g','-Wall','-Wextra','-Wno-unused-parameter','-Wno-missing-braces','-I'+str(src),'-I'+str(here),'-c',str(out/'main.c'),'-o',str(out/'main.o')],check=True)
make=(here/'Makefile').read_text();names=[]
for group in ('BASE_NAMES','OWN_NAMES'):names+=re.search(r'^'+group+r' := (.*)$',make,re.M).group(1).split()
objects=[out/'main.o' if name=='main' else root/'build/radio-backend'/(name+'.o') for name in names]+[root/'build/radio-backend/modem_snapshot_shared.o']
subprocess.run(['cc',*map(str,objects),'-L'+str(root/'build'),'-Wl,-rpath,'+str(root/'build'),'-lcariboulite','-lasound','-lpthread','-lm','-o',str(out/'backend')],check=True)
(out/'build.json').write_text(json.dumps({'change':'Accept zero amplitude for tx_tone; all other code and DSP objects from deployed build','sha256':hashlib.sha256((out/'backend').read_bytes()).hexdigest()},indent=2))
