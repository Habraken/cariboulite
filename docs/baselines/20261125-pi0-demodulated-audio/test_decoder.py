"""Synthetic offline CFO/deviation check; never opens a radio."""
from pathlib import Path
import json,subprocess,tempfile
import numpy as np
root=Path(__file__).resolve().parent;fs=1000000
with tempfile.TemporaryDirectory(prefix='demod-replay-test-') as directory:
 p=Path(directory);t=np.arange(2000000)/fs
 phase=2*np.pi*500*t+np.sin(2*np.pi*1000*t)
 a=np.empty((len(t),2),dtype='<i2');a[:,0]=np.rint(8192*np.cos(phase));a[:,1]=np.rint(8192*np.sin(phase));a.tofile(p/'test.iq')
 metrics={}
 for name,cfo in (('original',0),('centered',500)):
  f=p/(name+'.f32');subprocess.run([str(root/'decode-iq'),str(p/'test.iq'),str(f),str(fs),str(cfo)],check=True)
  raw=np.fromfile(f,'<f4').reshape(-1,2)[24000:,0].astype(float);tt=np.arange(len(raw))/48000
  basis=np.column_stack((np.cos(2*np.pi*1000*tt),np.sin(2*np.pi*1000*tt),np.ones(len(raw))))
  coef=np.linalg.lstsq(basis,raw,rcond=None)[0];residual=raw-basis@coef
  metrics[name]={'dc':float(coef[2]),'tone_amplitude':float(np.hypot(*coef[:2])),
   'fit_sinad_db':float(10*np.log10(np.mean((raw-coef[2])**2)/np.mean(residual**2)))}
  assert metrics[name]['fit_sinad_db']>45 and abs(metrics[name]['tone_amplitude']-.4)<.003
 assert abs(metrics['original']['dc']-.2)<.0001 and abs(metrics['centered']['dc'])<.0001
(root/'decoder-test.json').write_text(json.dumps({'passed':True,'synthetic_carrier_offset_hz':500,'synthetic_tone_hz':1000,'metrics':metrics},indent=2))
print('PASS: carrier centering removes DC and preserves the 1 kHz modulation.')
