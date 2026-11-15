#!/usr/bin/env python3
"""Analyze recorded RX PCM; these are end-to-end audio metrics, not RF purity."""
import csv,json,math,pathlib,sys,wave
import numpy as np
folder=pathlib.Path(sys.argv[1]);records=json.loads((folder/'results.json').read_text())

def analyze(path):
 with wave.open(str(path),'rb') as f:
  assert (f.getnchannels(),f.getsampwidth(),f.getframerate())==(1,2,48000)
  frames=f.getnframes();data=np.frombuffer(f.readframes(frames),dtype='<i2').astype(float)/32768
 assert frames==144000,(path,frames)
 y=data[24000:120000];y=y-y.mean();n=len(y);t=np.arange(n)/48000
 spectrum=np.abs(np.fft.rfft(y*np.hanning(n)));freqs=np.fft.rfftfreq(n,1/48000)
 mask=np.flatnonzero((freqs>=950)&(freqs<=1050));center=freqs[mask[np.argmax(spectrum[mask])]]
 total=float(y@y)
 def fit(freq):
  a=np.sin(2*np.pi*freq*t);b=np.cos(2*np.pi*freq*t)
  a-=a.mean();b-=b.mean()
  matrix=np.array([[a@a,a@b],[a@b,b@b]])
  projection=np.array([a@y,b@y]);coefs=np.linalg.solve(matrix,projection)
  residual=max(1e-30,total-float(coefs@projection))
  return residual,coefs,a,b
 lo,hi=center-.6,center+.6;ratio=(math.sqrt(5)-1)/2
 c=hi-ratio*(hi-lo);d=lo+ratio*(hi-lo)
 fc=fit(c)[0];fd=fit(d)[0]
 for _ in range(28):
  if fc<fd:hi=d;d=c;fd=fc;c=hi-ratio*(hi-lo);fc=fit(c)[0]
  else:lo=c;c=d;fc=fd;d=lo+ratio*(hi-lo);fd=fit(d)[0]
 hz=(lo+hi)/2;residual,coefs,a,b=fit(hz)
 tone=float(np.hypot(*coefs))/math.sqrt(2);model=coefs[0]*a+coefs[1]*b
 remaining=y-model;harmonics=[]
 for k in (2,3):
  basis=np.column_stack((np.sin(2*np.pi*hz*k*t),np.cos(2*np.pi*hz*k*t),np.ones(n)))
  coef=np.linalg.lstsq(basis,remaining,rcond=None)[0]
  harmonics.append(float(np.hypot(coef[0],coef[1]))/math.sqrt(2))
 bins=np.sqrt(np.mean(y.reshape(-1,480)**2,axis=1));median=float(np.median(bins))
 return {'tone_hz':hz,'tone_rms_dbfs':20*math.log10(tone),'rms_dbfs':10*math.log10(total/n),
         'peak_dbfs':20*math.log10(float(np.max(np.abs(y)))),
         'tone_to_residual_db':10*math.log10(tone*tone/(residual/n)),
         'second_third_harmonic_percent':100*math.sqrt(sum(v*v for v in harmonics))/tone,
         'minimum_10ms_rms_fraction':float(np.min(bins))/median,
         'low_level_10ms_windows':int(np.count_nonzero(bins<median/4)),
         'pcm_clipped_samples':int(np.count_nonzero(np.abs(data)>=32767/32768)),'recorded_frames':frames}

rows=[]
for record in records:
 path=folder/'rx'/pathlib.Path(record['wav']).name
 metrics=analyze(path)
 volume=record['audio_output']['volume_db']
 metrics['apparent_rx_modulation_fraction']=.5*10**((metrics['tone_rms_dbfs']+12-volume)/20)
 row={k:record[k] for k in ('case','tx_rate_hz','rx_rate_hz','tx_power_setting_dbm','tx_am_depth','tx_tone_amplitude','rssi_dbm')}
 row.update(metrics);row['rx_volume_db']=volume
 row['tx_am_backoff_db']=record.get('tx_am_backoff_db',record['tx_snapshot']['config'].get('tx_am_backoff_db',0))
 row['rx_rf_gain_db']=record['rx_snapshot']['config']['am_rf_gain_db']
 row['rx_frequency_hz']=record['rx_snapshot']['config']['requested_frequency_hz']
 row['am_offset_tuning_enabled']=record['rx_snapshot']['config'].get('am_offset_tuning_enabled',False)
 row['rx_counter_window_s']=record.get('counter_window_s',{}).get('rx','')
 row['tx_counter_window_s']=record.get('counter_window_s',{}).get('tx','')
 row['stream_error_deltas']=sum(record['error_counter_deltas'].values());rows.append(row)
(folder/'audio-analysis.json').write_text(json.dumps(rows,indent=2)+'\n')
with (folder/'audio-analysis.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print(json.dumps(rows,indent=2))
