"""Drive sweep spectra and metrics; analyze.py must have completed first."""
from pathlib import Path
import json,csv,wave
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
DATA=ROOT/'build/whistle-drive-analysis/data';OUT=HERE/'analysis';OUT.mkdir(exist_ok=True)
measurements=json.loads((HERE/'measurements.json').read_text())
if (HERE/'confirmation-measurements.json').exists():measurements+=json.loads((HERE/'confirmation-measurements.json').read_text())
blocks=['a8','b4','c2','d8'];labels=['8 mA initial','4 mA','2 mA','8 mA repeat']
rows=[];spectra={};health=[]
def psd(x,fs,n):
 f,p=signal.welch(x,fs,nperseg=min(n,len(x)),return_onesided=not np.iscomplexobj(x),detrend=False if np.iscomplexobj(x) else 'constant')
 order=np.argsort(f);return f[order],p[order]
def power(f,p,mask):return float(np.trapezoid(p[mask],f[mask]))
def db(x):return float(10*np.log10(max(x,1e-30)))
for m in measurements:
 cfg=m['case'];name=cfg['name'];base=cfg['base_case'];rate=cfg['rate'];cap=DATA/name/'rx/capture'
 info=json.loads(cap.with_suffix('.json').read_text())
 iq=np.fromfile(cap.with_suffix('.iq'),'<i2').reshape(-1,2);assert len(iq)==rate*4
 z=iq[:,0].astype(float)+1j*iq[:,1];f,p=psd(z[rate:],rate,262144)
 audio=np.fromfile(cap.with_suffix('.pcm'),'<i2').reshape(-1,info['channels'])[:,0].astype(float)/32768;assert len(audio)==48000*6
 fa,pa=psd(audio[48000:],48000,65536)
 row=dict(case=name,base_case=base,block=cfg['block'],rx_drive_ma=cfg['drive_ma'],rate=rate,
          sinad_db=m['after']['rx']['rx']['sinad']['db'],rssi_dbm=m['after']['rx']['hardware']['rf24']['registers']['RSSI']-256,
          audio_300_3000_dbfs=db(power(fa,pa,(fa>=300)&(fa<=3000))),
          audio_300_8000_dbfs=db(power(fa,pa,(fa>=300)&(fa<=8000))))
 spectrum_db=10*np.log10(np.maximum(p,1e-30))
 if not cfg.get('no_tx') and base.startswith('carrier'):
  inside=np.where(np.abs(f)<12000)[0];idx=inside[np.argmax(p[inside])];carrier=float(f[idx]);cp=power(f,p,abs(f-carrier)<80)
  peaks,_=signal.find_peaks(spectrum_db,prominence=8,distance=max(1,int(60/(f[1]-f[0]))))
  peaks=[i for i in peaks if abs(f[i])<12000 and abs(f[i]-carrier)>300 and abs(f[i])>80]
  peaks=sorted(peaks,key=lambda i:p[i],reverse=True)[:8]
  candidates=[]
  for i in peaks:
   peak=float(f[i]);sp=power(f,p,abs(f-peak)<60)
   candidates.append(dict(hz=peak,dbc=db(sp/cp),carrier_separation_hz=abs(peak-carrier)))
  amask=np.where((fa>300)&(fa<10000))[0];ai=amask[np.argmax(pa[amask])]
  row.update(carrier_hz=carrier,carrier_power_adc_units2=cp,iq_candidates=candidates,
             strongest_iq_spur_hz=candidates[0]['hz'] if candidates else None,
             strongest_iq_spur_dbc=candidates[0]['dbc'] if candidates else None,
             strongest_audio_hz=float(fa[ai]),strongest_audio_psd_dbfs_hz=db(pa[ai]))
  spectra[name]=(f,p/cp,fa,pa)
 else:spectra[name]=(f,p,fa,pa)
 rows.append(row)
 h={'case':name}
 for role,section,field in [('rx','queue','drops'),('rx','audio_queue','drops'),('rx','playback','underruns'),('rx','playback','write_errors'),('rx','reader','errors'),('tx','writer','errors'),('tx','driver','underrun_periods')]:
  before=m['before'][role][role][section];after=m['after'][role][role][section]
  h[f'{role}.{section}.{field}']=after[field]-before[field] if after is not None and field in after else None
 h['rx_clipped_samples']=m['after']['rx']['rx']['audio_output']['clipped_samples']-m['before']['rx']['rx']['audio_output']['clipped_samples']
 health.append(h)
 assert all(v==0 or v is None for k,v in h.items() if k!='case'),h
 if base in ('carrier','tone_1k','ctcss_only_rx_on'):
  with wave.open(str(OUT/(name+'.wav')),'wb') as w:
   w.setnchannels(info['channels']);w.setsampwidth(2);w.setframerate(48000);w.writeframes(cap.with_suffix('.pcm').read_bytes())
for base in ('carrier','carrier_rx_plus5k','carrier_rx_minus5k','carrier_2m'):
 fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
 for block,label in zip(blocks,labels):
  name=block+'__'+base
  if name not in spectra:continue
  f,p,fa,pa=spectra[name]
  axes[0].plot(f/1000,10*np.log10(np.maximum(p,1e-30)),label=label)
  axes[1].plot(fa/1000,10*np.log10(np.maximum(pa,1e-30)),label=label)
 axes[0].set_xlim(-12,12);axes[0].set_ylim(-105,0);axes[0].set_ylabel('IQ PSD (dBc/Hz)');axes[0].set_xlabel('Offset from RX tuning (kHz)')
 axes[0].set_title(f'{base}: RX CLKO drive sweep; TX fixed at 8 mA')
 axes[1].set_xlim(0,10);axes[1].set_ylim(-130,-35);axes[1].set_ylabel('Playback PCM PSD (dBFS/Hz)');axes[1].set_xlabel('Audio frequency (kHz)')
 for ax in axes:ax.grid(alpha=.25);ax.legend()
 fig.savefig(OUT/(base+'-drive-comparison.png'),dpi=160);plt.close(fig)
# Short reverse-order clean-carrier confirmation.
fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
for block,label in zip(('e8','f2','g4','h8'),('8 mA before','2 mA','4 mA','8 mA after')):
 name=block+'__carrier'
 if name not in spectra:continue
 f,p,fa,pa=spectra[name]
 axes[0].plot(f/1000,10*np.log10(np.maximum(p,1e-30)),label=label)
 axes[1].plot(fa/1000,10*np.log10(np.maximum(pa,1e-30)),label=label)
axes[0].set_xlim(-8,8);axes[0].set_ylim(-105,0);axes[0].set_ylabel('IQ PSD (dBc/Hz)');axes[0].set_xlabel('Offset from RX tuning (kHz)');axes[0].set_title('Short clean-carrier confirmation: 8 / 2 / 4 / 8 mA RX drive')
axes[1].set_xlim(0,10);axes[1].set_ylim(-130,-35);axes[1].set_ylabel('Playback PCM PSD (dBFS/Hz)');axes[1].set_xlabel('Audio frequency (kHz)')
for ax in axes:ax.grid(alpha=.25);ax.legend()
fig.savefig(OUT/'confirmation-drive-comparison.png',dpi=160);plt.close(fig)
(OUT/'drive-metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
(OUT/'health.json').write_text(json.dumps(health,indent=2)+'\n')
fields=['case','base_case','block','rx_drive_ma','rate','rssi_dbm','sinad_db','carrier_hz','strongest_iq_spur_hz','strongest_iq_spur_dbc','strongest_audio_hz','audio_300_3000_dbfs','audio_300_8000_dbfs']
with (OUT/'drive-metrics.csv').open('w') as out:
 w=csv.DictWriter(out,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
print('ALL CAPTURE HEALTH DELTAS ZERO')
for row in rows:
 if row['base_case'] in ('carrier','tone_1k','tone_1k_ctcss'):print(json.dumps(row),flush=True)
