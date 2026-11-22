from pathlib import Path
import json,csv,wave,hashlib,tarfile
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];DATA=ROOT/'build/whistle-bypass-analysis/data';OUT=HERE/'analysis'
rows=json.loads((OUT/'summary.json').read_text());measurements=json.loads((HERE/'measurements.json').read_text())
health=[];lines=[]
fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
for name in ('carrier','carrier_rx_plus5k','carrier_rx_minus5k'):
 r=next(r for r in rows if r['case']==name);cfg=r['config'];rate=cfg['rate'];cap=DATA/name/'rx/capture'
 iq=np.fromfile(cap.with_suffix('.iq'),'<i2').reshape(-1,2);z=iq[:,0].astype(float)+1j*iq[:,1]
 f,p=signal.welch(z[rate:],fs=rate,nperseg=262144,return_onesided=False,detrend=False);order=np.argsort(f);f,p=f[order],p[order]
 carrier=r['carrier_offset_hz'];mask=(f-carrier>4500)&(f-carrier<6000);idx=np.where(mask)[0][np.argmax(p[mask])];spur=float(f[idx]);power_carrier=np.trapezoid(p[np.abs(f-carrier)<80],f[np.abs(f-carrier)<80]);power_spur=np.trapezoid(p[np.abs(f-spur)<60],f[np.abs(f-spur)<60]);dbc=10*np.log10(power_spur/power_carrier)
 raw=r['raw_peaks'][0]['hz'];pcm=r['pcm_peaks'][0]['hz'];lines.append(dict(case=name,carrier_offset_hz=carrier,tracked_iq_line_hz=spur,line_band_dbc=float(dbc),carrier_to_line_hz=abs(spur-carrier),strongest_raw_audio_hz=raw,strongest_pcm_audio_hz=pcm))
 axes[0].plot(f/1000,10*np.log10(np.maximum(p/power_carrier,1e-30)),label=f'{name}: carrier {carrier:.0f} Hz; tracked line {spur:.0f} Hz')
 pcm_info=json.loads(cap.with_suffix('.json').read_text());audio=np.fromfile(cap.with_suffix('.pcm'),'<i2').reshape(-1,pcm_info['channels'])[:,0].astype(float)/32768
 af,ap=signal.welch(audio[48000:],48000,nperseg=65536)
 axes[1].plot(af/1000,10*np.log10(np.maximum(ap,1e-30)),label=f'{name}: strongest audio peak {pcm:.0f} Hz')
axes[0].set_xlim(-14,14);axes[0].set_ylim(-90,0);axes[0].set_ylabel('IQ PSD (dBc/Hz)');axes[0].set_xlabel('Offset from RX tuning (kHz)');axes[0].set_title('HiF bypass IQ spectra with centered and offset RX tuning')
axes[1].set_xlim(0,9);axes[1].set_ylim(-115,-40);axes[1].set_ylabel('Playback PCM PSD (dBFS/Hz)');axes[1].set_xlabel('Audio frequency (kHz)');axes[1].set_title('HiF bypass playback audio spectra')
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.savefig(OUT/'localization.png',dpi=160);plt.close(fig)
for m in measurements:
 name=m['case']['name'];b=m['before'];a=m['after'];h={'case':name}
 for role,section,field in [('rx','queue','drops'),('rx','audio_queue','drops'),('rx','playback','underruns'),('rx','playback','write_errors'),('rx','reader','errors'),('tx','writer','errors'),('tx','driver','underrun_periods')]:
  h[f'{role}.{section}.{field}']=a[role][role][section][field]-b[role][role][section][field] if a[role][role][section] is not None and field in a[role][role][section] else None
 h['rx_clipped_samples']=a['rx']['rx']['audio_output']['clipped_samples']-b['rx']['rx']['audio_output']['clipped_samples'];health.append(h)
 assert all(v==0 or v is None for k,v in h.items() if k!='case'),h
for name in ('carrier','ctcss_only_rx_on','tone_1k'):
 cap=DATA/name/'rx/capture';info=json.loads(cap.with_suffix('.json').read_text())
 with wave.open(str(OUT/(name+'.wav')),'wb') as w:w.setnchannels(info['channels']);w.setsampwidth(2);w.setframerate(48000);w.writeframes(cap.with_suffix('.pcm').read_bytes())
(OUT/'localization.json').write_text(json.dumps(lines,indent=2));(OUT/'health.json').write_text(json.dumps(health,indent=2))
with (OUT/'localization.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=lines[0],lineterminator="\n");w.writeheader();w.writerows(lines)
print(json.dumps(lines,indent=2));print('ALL CAPTURE HEALTH DELTAS ZERO')
# Preserve original IQ, PCM, metadata and logs; replay output is reproducible.
with tarfile.open(HERE/'captures.tar.gz','w:gz',compresslevel=5) as t:
 for p in sorted(DATA.rglob('*')):
  if p.is_file() and '.decoded.' not in p.name:t.add(p,arcname=str(p.relative_to(DATA)),recursive=False)
