"""Compare the original HiF and current S1G link, same TX/RX boards."""
from pathlib import Path
import json
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];OUT=HERE/'analysis'
fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained');rows=[]
for label,folder in [('HiF (previous run)','whistle-analysis'),('S1G (current run)','whistle-s1g-analysis')]:
 cap=ROOT/'build'/folder/'data/carrier/rx/capture'
 iq=np.fromfile(cap.with_suffix('.iq'),'<i2').reshape(-1,2);z=iq[:,0].astype(float)+1j*iq[:,1]
 f,p=signal.welch(z[1000000:],1000000,nperseg=262144,return_onesided=False,detrend=False);order=np.argsort(f);f,p=f[order],p[order]
 carrier=f[np.argmax(p)];mask=np.abs(f-carrier)<80;power=np.trapezoid(p[mask],f[mask])
 axes[0].plot(f/1000,10*np.log10(np.maximum(p/power,1e-30)),label=label)
 info=json.loads(cap.with_suffix('.json').read_text());audio=np.fromfile(cap.with_suffix('.pcm'),'<i2').reshape(-1,info['channels'])[:,0].astype(float)/32768
 af,ap=signal.welch(audio[48000:],48000,nperseg=65536)
 axes[1].plot(af/1000,10*np.log10(np.maximum(ap,1e-30)),label=label)
 def band(f,p,lo,hi):
  mask=(f>=lo)&(f<=hi);return float(10*np.log10(max(np.trapezoid(p[mask],f[mask]),1e-30)))
 mask=(af>=300)&(af<=3000);idx=np.where(mask)[0][np.argmax(ap[mask])]
 rows.append(dict(channel=label,carrier_offset_hz=float(carrier),audio_300_3000_dbfs=band(af,ap,300,3000),audio_1200_2000_dbfs=band(af,ap,1200,2000),previous_whistle_band_dbfs=band(af,ap,1474.4,1594.4),strongest_audio_300_3000_hz=float(af[idx]),strongest_audio_density_dbfs_per_hz=float(10*np.log10(ap[idx])),iq_1000_2200_dbc=band(f,p,1000,2200)-float(10*np.log10(power))))
axes[0].set_xlim(-5,5);axes[0].set_ylim(-105,0);axes[0].set_xlabel('Offset from RX tuning (kHz)');axes[0].set_ylabel('IQ PSD (dBc/Hz)');axes[0].set_title('Clean carrier: pi0a TX, pi0b RX; HiF versus S1G')
axes[1].set_xlim(0,8);axes[1].set_ylim(-140,-35);axes[1].set_xlabel('Audio frequency (kHz)');axes[1].set_ylabel('Playback PCM PSD (dBFS/Hz)');axes[1].set_title('Same nominal 90 dB link, requested 0 dBm TX, matched RX gain')
for ax in axes:ax.grid(alpha=.25);ax.legend()
fig.savefig(OUT/'channel-comparison.png',dpi=160)
(OUT/'channel-comparison.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows,indent=2))
