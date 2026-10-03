#!/usr/bin/env python3
"""Compare current FM DSP with the frozen pre-refactor implementation."""
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
src = here.parent / 'src'
with tempfile.TemporaryDirectory(prefix='fm-reference-') as directory:
    binary = Path(directory) / 'test'
    subprocess.run(['cc', '-O3', '-Wall', '-Wextra', '-I'+str(src),
                    str(here/'test_fm_reference.c'), str(src/'nbfm_demod.c'),
                    str(here/'fixtures/legacy_wbfm_demod/oracle.c'),
                    '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)
