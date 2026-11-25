"""Build the standalone, offline production-DSP replay; no radio access."""
from pathlib import Path
import hashlib,json,subprocess
here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'software/libcariboulite/src/nbfm_demod.c').exists())
src=root/'software/libcariboulite/src'
sources=[src/n for n in ('nbfm_demod.c','nbfm_demod_dsp.c','wbfm_demod.c','am_demod.c')]
subprocess.run(['cc','-O3','-ffast-math','-fno-math-errno','-std=c11','-Wall','-Wextra','-I'+str(src),str(here/'decode_iq.c'),*map(str,sources),'-lm','-o',str(here/'decode-iq')],check=True)
info={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
 'compiler':subprocess.check_output(['cc','--version'],text=True),
 'source_sha256':{str(s.relative_to(root)):hashlib.sha256(s.read_bytes()).hexdigest() for s in sources},
 'binary_sha256':hashlib.sha256((here/'decode-iq').read_bytes()).hexdigest()}
(here/'replay-build.json').write_text(json.dumps(info,indent=2)+'\n');print('Built offline replay without accessing hardware.')
