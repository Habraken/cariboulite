#!/usr/bin/env python3
"""Hardware-free SMI framing regression under UBSan, including real false locks."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[3];src=root/'software/libcariboulite/src'
with tempfile.TemporaryDirectory(prefix='smi-rx-regression-') as d:
    exe=str(Path(d)/'test')
    subprocess.run(['cc','-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=undefined',
        '-I'+str(src),str(Path(__file__).with_suffix('.c')),str(src/'caribou_smi/smi_rx_stream.c'),'-o',exe],check=True)
    subprocess.run([exe,*[str(root/f'docs/baselines/20261113-rx-stutter/alignment-offset-{p}.bin') for p in (1,2)]],check=True,timeout=30)
