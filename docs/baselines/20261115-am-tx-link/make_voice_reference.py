#!/usr/bin/env python3
"""Prepare real human speech from Open Speech Repository; requires curl, ffmpeg and NumPy.

Source credit: Open Speech Repository. Conditions permit copying, modification
and test-equipment use with this attribution; see the catalog URL below.
No compression or limiting is applied; a single gain sets peak amplitude to 0.8.
"""
import argparse,hashlib,pathlib,subprocess,tempfile,wave
import numpy as np
URL='https://www.voiptroubleshooter.com/open_speech/american/OSR_us_000_0010_8k.wav'
CATALOG='https://www.voiptroubleshooter.com/open_speech/american.html'
SHA256='a4bf9becd046d7aedb6d05b6e12347a6294a44f74d263089c636fb0a2b1e6561'
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',default='/tmp/am-voice-reference.wav')
a=p.parse_args()
with tempfile.TemporaryDirectory(prefix='am-human-voice-') as directory:
 original=pathlib.Path(directory)/'original.wav'
 subprocess.run(['curl','--fail','--silent','--show-error','--location','--max-time','30',URL,'-o',str(original)],check=True)
 data=original.read_bytes()
 if hashlib.sha256(data).hexdigest()!=SHA256:raise RuntimeError('Source hash changed; inspect before use')
 original.write_bytes(data)
 converted=pathlib.Path(directory)/'converted.wav'
 subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(original),
                 '-t','15','-ar','48000','-ac','1',str(converted)],check=True)
 with wave.open(str(converted),'rb') as f:
  assert (f.getsampwidth(),f.getnchannels(),f.getframerate())==(2,1,48000)
  samples=np.frombuffer(f.readframes(f.getnframes()),dtype='<i2').astype(float)
 peak=np.max(np.abs(samples));assert peak>0
 samples=np.rint(samples*(.8*32767/peak)).astype('<i2')
 with wave.open(a.output,'wb') as f:
  f.setnchannels(1);f.setsampwidth(2);f.setframerate(48000);f.writeframes(samples.tobytes())
 print('Source: Open Speech Repository; '+CATALOG)
 print('Wrote '+a.output)
