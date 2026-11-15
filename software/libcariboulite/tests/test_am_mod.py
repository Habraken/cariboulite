#!/usr/bin/env python3
"""AM TX waveform/packet tests with a strict independent measurement oracle."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='am-tx-') as d:
 common=['cc','-std=c11','-D_GNU_SOURCE','-O3','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-ffunction-sections','-fdata-sections',*['-I'+str(p) for p in [src,src/'at86rf215',src/'rffc507x']]]
 oracle=str(Path(d)/'oracle.o')
 subprocess.run([*common,'-c',str(here/'test_am_mod.c'),'-o',oracle],check=True)
 for policy,flags in [('strict',[]),('production',['-ffast-math','-fno-math-errno','-funroll-loops'])]:
  objects=[]
  for name in ['nbfm_mod','mod_worker']:
   obj=str(Path(d)/(policy+'-'+name+'.o'));objects.append(obj)
   subprocess.run([*common,*flags,'-c',str(src/(name+'.c')),'-o',obj],check=True)
  exe=str(Path(d)/policy)
  subprocess.run(['cc',oracle,*objects,'-Wl,--gc-sections','-lm','-pthread','-o',exe],check=True)
  print('Testing AM TX '+policy,flush=True)
  subprocess.run([exe],check=True,timeout=120)
