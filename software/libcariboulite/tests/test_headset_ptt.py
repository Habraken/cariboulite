#!/usr/bin/env python3
"""Controller call-button tests; simulated backend only, no headset or RF."""
from pathlib import Path
import os, subprocess, tempfile, time
root=Path(__file__).resolve().parents[3]
backend=Path(os.environ.get('CARIBOULITE_BACKEND',root/'build/radio-backend/cariboulite_radio_backend'))
src=root/'software/libcariboulite'
with tempfile.TemporaryDirectory(prefix='headset-ptt-') as tmp:
    base=Path(tmp);exe=base/'test'
    subprocess.run(['cc','-D_GNU_SOURCE','-D_POSIX_C_SOURCE=200809L','-std=c11','-O2',
        '-Wall','-Wextra','-Werror','-I'+str(src/'src'),'-I'+str(src/'tools/radio_backend'),
        str(src/'tests/test_headset_ptt.c'),str(src/'tools/radio_backend/headset_ptt.c'),str(src/'tools/radio_backend/command_protocol.c'),
        '-lm','-o',str(exe)],check=True)
    with (base/'backend.log').open('w+') as log:
        p=subprocess.Popen([str(backend),'--dry-run','--rx-start','--socket',str(base/'status.sock')],stdout=log,stderr=log)
        try:
            end=time.monotonic()+5
            while not (base/'control.sock').exists():
                assert p.poll() is None and time.monotonic()<end
                time.sleep(.01)
            r=subprocess.run([str(exe),str(base/'control.sock'),str(src/'tests/fixtures/hid-h650e.hex'),str(src/'tests/fixtures/hid-jabra-510.hex')],capture_output=True,text=True,timeout=15)
            if r.returncode:
                log.flush();log.seek(0);print(log.read());print(r.stdout,r.stderr)
            assert r.returncode==0
            print(r.stdout.splitlines()[-1])
        finally:
            p.terminate();assert p.wait(timeout=5)==0
