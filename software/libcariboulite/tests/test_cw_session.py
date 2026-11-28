#!/usr/bin/env python3
"""Linux CW mixer/session checks with simulated input and no sound device."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='cw-session-') as tmp:
    exe=Path(tmp)/'test'
    subprocess.run(['cc','-D_GNU_SOURCE','-std=c11','-O3','-ffast-math','-Wall','-Wextra','-Werror','-I'+str(src),str(here/'test_cw_session.c'),str(src/'cw_session.c'),str(src/'cw_keyer.c'),str(src/'alsa_sink.c'),'-lasound','-lm','-pthread','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=10)
