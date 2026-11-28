#!/usr/bin/env python3
"""Portable HID tests with UBSan; HEADSET_TEST_ASAN=1 adds AddressSanitizer.

Some Pi kernels cannot map GCC ASan's required address range. The ASan run
can be enabled on a host that supports it, independently of the Pi checks.
"""
from pathlib import Path
import os,subprocess,tempfile
here=Path(__file__).resolve().parent;src=here.parent/'tools/radio_backend'
with tempfile.TemporaryDirectory(prefix='hid-decoder-') as tmp:
    runs=[('normal',['-O2']),('ubsan',['-O1','-fsanitize=undefined,bounds','-fno-omit-frame-pointer'])]
    if os.environ.get('HEADSET_TEST_ASAN')=='1':
        runs.append(('asan',['-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer']))
    for name,flags in runs:
        exe=Path(tmp)/name
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',*flags,'-I'+str(src),
            str(here/'test_headset_decoder.c'),str(src/'headset_ptt.c'),'-o',str(exe)],check=True)
        subprocess.run([str(exe),str(here/'fixtures/hid-h650e.hex'),str(here/'fixtures/hid-jabra-510.hex')],check=True)
