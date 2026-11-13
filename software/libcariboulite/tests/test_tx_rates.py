#!/usr/bin/env python3
"""Exercise real TX packet generation and modulation at every native clock."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='tx-rates-') as d:
 for policy,flags in [('strict',[]),('production',['-ffast-math','-fno-math-errno','-funroll-loops'])]:
  exe=str(Path(d)/policy)
  subprocess.run(['cc','-std=c11','-D_GNU_SOURCE','-O3','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-ffunction-sections','-fdata-sections',*flags,*['-I'+str(p) for p in [src,src/'at86rf215',src/'rffc507x']],str(here/'test_tx_rates.c'),str(src/'nbfm_mod.c'),str(src/'mod_worker.c'),'-Wl,--gc-sections','-lm','-pthread','-o',exe],check=True)
  subprocess.run([exe],check=True,timeout=120)
