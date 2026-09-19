#!/usr/bin/env python3
"""Compile and run the hardware-free audio source contract test."""
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='audio-source-') as directory:
    binary = Path(directory) / 'test'
    subprocess.run(['cc', '-Wall', '-Wextra', '-I'+str(here.parent/'src'),
                    str(here/'test_audio_source.c'), '-Wl,--wrap=snd_pcm_readi',
                    '-lasound', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
