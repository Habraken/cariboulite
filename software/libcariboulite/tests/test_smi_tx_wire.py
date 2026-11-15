#!/usr/bin/env python3
"""Real SMI serializer regression; raw output bytes, no device access."""
from pathlib import Path
import subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'src'
with tempfile.TemporaryDirectory(prefix='smi-tx-wire-') as directory:
 exe=str(Path(directory)/'test')
 subprocess.run(['cc','-std=c11','-D_GNU_SOURCE','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-unused-function','-Wno-unused-variable','-Wno-missing-field-initializers','-Wno-parentheses','-ffunction-sections','-fdata-sections','-I'+str(src),str(here/'test_smi_tx_wire.c'),str(src/'caribou_smi/caribou_smi.c'),'-Wl,--gc-sections','-Wl,--wrap=write','-pthread','-o',exe],check=True)
 subprocess.run([exe],check=True,timeout=10)
