#!/usr/bin/env python3
"""Hardware-free detector and real RX worker gating checks."""
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
src = here.parent / 'src'
with tempfile.TemporaryDirectory(prefix='squelch-') as directory:
    binary = Path(directory) / 'test'
    modules = ['noise_squelch.c', 'carrier_squelch.c', 'nbfm_mod.c',
               'nbfm_demod.c', 'demod_worker.c']
    subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(src), str(here/'test_squelch.c'),
                    *[str(src/m) for m in modules], '-lm', '-pthread',
                    '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)
