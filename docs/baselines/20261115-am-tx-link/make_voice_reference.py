#!/usr/bin/env python3
"""Create the optional speech test WAV on pi4g; requires ffmpeg with flite."""
import pathlib,subprocess,wave
import numpy as np
text=('Caribou Lite amplitude modulation voice test. The quick brown fox jumps over the lazy dog. '
      'This is a test of narrow band voice communications. The received speech should be clear and continuous. ')*2
pathlib.Path('/tmp/am-voice-text.txt').write_text(text)
subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-f','lavfi','-i',
                'flite=textfile=/tmp/am-voice-text.txt:voice=slt','-ar','48000','-ac','1',
                '/tmp/am-voice-synthesis.wav'],check=True)
with wave.open('/tmp/am-voice-synthesis.wav','rb') as f:
 assert (f.getsampwidth(),f.getnchannels(),f.getframerate())==(2,1,48000)
 data=np.frombuffer(f.readframes(f.getnframes()),dtype='<i2').astype(float)
assert np.max(np.abs(data))>0
data=np.rint(data*(.8*32767/np.max(np.abs(data)))).astype('<i2')
with wave.open('/tmp/am-voice-reference.wav','wb') as f:
 f.setnchannels(1);f.setsampwidth(2);f.setframerate(48000);f.writeframes(data.tobytes())
