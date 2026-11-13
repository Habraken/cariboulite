#!/usr/bin/env python3
"""Pure DSP checks for all native RX rates, no hardware or audio devices."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='rx-rates-') as d:
 for policy,flags in [('strict',[]),('production',['-ffast-math','-fno-math-errno','-funroll-loops'])]:
  exe=str(Path(d)/policy)
  subprocess.run(['cc','-std=c11','-O3','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',*flags,'-I'+str(src),str(here/'test_rx_rates.c'),*[str(src/name) for name in ['nbfm_demod.c','nbfm_demod_dsp.c','wbfm_demod.c']],'-lm','-o',exe],check=True)
  subprocess.run([exe],check=True,timeout=120)
