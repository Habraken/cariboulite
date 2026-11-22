#!/usr/bin/env python3
"""Offline CDCSS encoder/decoder + actual modem integration, no hardware."""
from pathlib import Path
import subprocess
import tempfile
here=Path(__file__).resolve().parent
src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='cdcss-') as directory:
    for policy,flags in [('strict',[]),('production',['-ffast-math','-fno-math-errno','-funroll-loops'])]:
        exe=str(Path(directory)/policy)
        subprocess.run(['cc','-std=c11','-D_GNU_SOURCE','-O3','-Wall','-Wextra','-Werror',*flags,
            '-I'+str(src),str(here/'test_cdcss.c'),*[str(src/n) for n in
            ('nbfm_mod.c','nbfm_demod.c','nbfm_demod_dsp.c','wbfm_demod.c','am_demod.c')],'-lm','-o',exe],check=True)
        print('CDCSS '+policy,flush=True)
        subprocess.run([exe],check=True,timeout=180)
