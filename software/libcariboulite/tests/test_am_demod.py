#!/usr/bin/env python3
"""Synthetic AM receiver checks; never opens a radio or audio device."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='am-demod-') as d:
 common=['cc','-std=c11','-O3','-Wall','-Wextra','-Werror','-I'+str(src)]
 oracle=Path(d)/'oracle.o'
 subprocess.run(common+['-c',str(here/'test_am_demod.c'),'-o',str(oracle)],check=True)
 for policy,flags in [('strict',[]),('production',['-ffast-math','-fno-math-errno','-funroll-loops'])]:
  objects=[]
  for name in ['nbfm_demod','nbfm_demod_dsp','wbfm_demod','am_demod']:
   obj=Path(d)/(policy+'-'+name+'.o');objects.append(str(obj))
   subprocess.run(common+flags+['-c',str(src/(name+'.c')),'-o',str(obj)],check=True)
  exe=str(Path(d)/policy)
  subprocess.run(['cc',str(oracle),*objects,'-lm','-o',exe],check=True)
  print('Testing AM '+policy,flush=True);subprocess.run([exe],check=True,timeout=180)
