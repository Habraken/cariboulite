#!/usr/bin/env python3
"""Hardware-free CW timing, waveform and receiver checks."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='cw-test-') as tmp:
    common=['cc','-std=c11','-O3','-Wall','-Wextra','-Werror','-I'+str(src)]
    oracle=Path(tmp)/'oracle.o'
    subprocess.run(common+['-c',str(here/'test_cw.c'),'-o',str(oracle)],check=True)
    for policy,flags in [('strict',[]),('production',['-ffast-math','-fno-math-errno','-funroll-loops'])]:
        objects=[]
        for name in ['cw_keyer','nbfm_mod','nbfm_demod','nbfm_demod_dsp','wbfm_demod','am_demod']:
            obj=Path(tmp)/(policy+'-'+name+'.o');objects.append(str(obj))
            subprocess.run(common+flags+['-c',str(src/(name+'.c')),'-o',str(obj)],check=True)
        exe=Path(tmp)/policy
        subprocess.run(['cc',str(oracle),*objects,'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=180)
