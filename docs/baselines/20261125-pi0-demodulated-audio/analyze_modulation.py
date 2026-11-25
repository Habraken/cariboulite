"""Check whether wanted and receiver-relative IQ components carry the tone."""
from pathlib import Path
import json
import numpy as np
from scipy import signal
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/'analysis/summary.json').read_text());result=[]
kernel=signal.firwin(4001,1100,fs=50000,window=('kaiser',8))
for row in rows:
 if row['name'] not in ('tone_a','tone_b'):continue
 cap=HERE/'data'/row['case']/'rx/capture.iq'
 a=np.fromfile(cap,'<i2').reshape(-1,2).astype(float)
 z=signal.resample_poly(a[:,0]+1j*a[:,1],1,20);del a
 t=np.arange(len(z))/50000
 carrier=row['carrier_offset_in_decoded_iq_hz']
 guess=-2600 if row['rx_node']=='a' else 3180
 spur=min(row['iq_lines'],key=lambda x:abs(x['frequency_hz']-guess))['frequency_hz']
 coefficients=[]
 for center in (carrier,spur):
  isolated=signal.fftconvolve(z*np.exp(-2j*np.pi*center*t),kernel,mode='same')
  audio=np.angle(isolated[1:]*isolated[:-1].conj())*50000/(2*np.pi)
  y=audio[50000:-50000];tt=np.arange(len(y))/50000;hz=row['replay_tone_frequency_hz']
  design=np.column_stack((np.cos(2*np.pi*hz*tt),np.sin(2*np.pi*hz*tt),np.ones(len(y))))
  coefficients.append(np.linalg.lstsq(design,y,rcond=None)[0][:2])
 wanted,unwanted=coefficients
 v=wanted[0]-1j*wanted[1];u=unwanted[0]-1j*unwanted[1]
 item={'case':row['case'],'wanted_iq_frequency_hz':carrier,'spur_iq_bin_hz':spur,
  'wanted_isolated_tone_amplitude_hz':float(abs(v)),'spur_isolated_tone_amplitude_hz':float(abs(u)),
  'spur_to_wanted_modulation_amplitude_ratio':float(abs(u/v)),
  'relative_modulation_phase_deg':float(np.angle(u/v)*180/np.pi),
  'modulation_difference_relative_percent':float(abs(v-u)/abs(v)*100)}
 result.append(item);print(item,flush=True)
(HERE/'analysis/modulation-correlation.json').write_text(json.dumps(result,indent=2)+'\n')
