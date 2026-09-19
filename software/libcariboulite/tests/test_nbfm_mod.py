#!/usr/bin/env python3
"""Verify the explicit modulator contract and unchanged IQ against step 5."""
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
src = here.parent / 'src'
with tempfile.TemporaryDirectory(prefix='nbfm-mod-') as directory:
    binary = Path(directory) / 'test'
    subprocess.run(['cc', '-O2', '-Wall', '-Wextra', '-Werror', '-I'+str(src),
                    str(here/'test_nbfm_mod.c'), str(src/'nbfm_mod.c'),
                    '-Wl,--wrap=calloc', '-Wl,--wrap=free', '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=30)
