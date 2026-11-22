from pathlib import Path
import json,subprocess,sys
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
DATA=ROOT/'build/whistle-s1g-analysis/data'
OUT=HERE/'analysis';OUT.mkdir(exist_ok=True)

def psd(x,fs,n=65536):
 return signal.welch(x,fs=fs,window='hann',nperseg=min(n,len(x)),noverlap=min(n,len(x))//2,detrend=False if np.iscomplexobj(x) else 'constant',return_onesided=not np.iscomplexobj(x),scaling='density')
def peaks(f,p,lo=100,hi=20000,number=8):
 mask=(f>lo)&(f<hi);ii=signal.find_peaks(10*np.log10(np.maximum(p,1e-30)),prominence=6,distance=max(1,int(30/(f[1]-f[0]))))[0]
 ii=[i for i in ii if mask[i]];ii=sorted(ii,key=lambda i:p[i],reverse=True)[:number]
 return [dict(hz=float(f[i]),db_per_hz=float(10*np.log10(max(p[i],1e-30)))) for i in ii]

rows=[];spectra={}
for directory in sorted(DATA.iterdir()):
 cap=directory/'rx/capture'
 if not cap.with_suffix('.iq').exists():continue
 name=directory.name;info=json.loads(cap.with_suffix('.json').read_text())
 measurements=json.loads((HERE/'measurements.json').read_text());row=next(r for r in measurements if r['case']['name']==name)
 cfg=row['case'];rate=cfg['rate']
 iq=np.fromfile(cap.with_suffix('.iq'),'<i2').reshape(-1,2).astype(np.float64)
 z=iq[:,0]+1j*iq[:,1];assert len(z)==rate*4,(name,len(z),rate)
 pcm=np.fromfile(cap.with_suffix('.pcm'),'<i2').reshape(-1,info['channels'])[:,0].astype(np.float64)/32768
 assert len(pcm)==48000*6,(name,len(pcm))
 replay=cap.with_suffix('.decoded.f32')
 subprocess.run(['/tmp/cl-whistle-decode',str(cap.with_suffix('.iq')),str(replay),str(rate)],check=True)
 decoded=np.fromfile(replay,'<f4').reshape(-1,2)
 # Independent float64 FM discriminator after a purpose-designed channel FIR.
 # RF -> 50 kHz with polyphase anti-aliasing, +/-6k channel FIR, then angle.
 zi=signal.resample_poly(z,1,rate//50000)
 zi=signal.lfilter(signal.firwin(161,7000,fs=50000),[1],zi)
 independent=np.angle(zi[1:]*zi[:-1].conj())*50000/(2*np.pi*2500)
 f,p=psd(pcm[48000:],48000);fr,pr=psd(decoded[48000:,0],48000);ff,pf=psd(decoded[48000:,1],48000);fi,pi=psd(independent[50000:],50000)
 fq,pq=psd(z[rate:],rate,262144);sort=np.argsort(fq);fq,pq=fq[sort],pq[sort]
 carrier_idx=np.argmax(pq);carrier_offset=float(fq[carrier_idx])
 s=dict(case=name,config=cfg,iq_samples=len(z),pcm_frames=len(pcm),pcm_rms_dbfs=float(20*np.log10(max(np.std(pcm[48000:]),1e-12))),carrier_offset_hz=carrier_offset,pcm_peaks=peaks(f,p),raw_peaks=peaks(fr,pr),independent_peaks=peaks(fi,pi),iq_peaks=peaks(fq,pq,-15000,15000),rx_status=row['after']['rx']['rx'])
 rows.append(s);spectra[name]=(f,p,fr,pr,ff,pf,fi,pi,fq,pq)
 fig,axes=plt.subplots(3,1,figsize=(12,10),layout='constrained')
 axes[0].plot(fq,10*np.log10(np.maximum(pq,1e-30)));axes[0].set_xlim(-15000,15000);axes[0].set_ylabel('IQ PSD (ADC units²/Hz)');axes[0].set_title(f'{name}: RF spectrum around RX tuning; carrier {carrier_offset:.1f} Hz')
 axes[1].plot(fr,10*np.log10(np.maximum(pr,1e-30)),label='production raw discriminator');axes[1].plot(fi,10*np.log10(np.maximum(pi,1e-30)),alpha=.7,label='independent float64 discriminator');axes[1].set_xlim(0,12000);axes[1].set_ylabel('Normalized PSD (dB/Hz)');axes[1].legend()
 axes[2].plot(f,10*np.log10(np.maximum(p,1e-30)),label='live PCM before ALSA conversion');axes[2].set_xlim(0,12000);axes[2].set_ylabel('PCM PSD (dBFS/Hz)');axes[2].set_xlabel('Frequency (Hz)');axes[2].legend()
 for ax in axes:ax.grid(alpha=.25)
 fig.savefig(OUT/(name+'.png'),dpi=150);plt.close(fig)
 print(name,'carrier',carrier_offset,'pcm peaks',s['pcm_peaks'][:5],flush=True)
(OUT/'summary.json').write_text(json.dumps(rows,indent=2))
if rows:
 fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
 for name in ('carrier','ctcss_only_rx_off','ctcss_only_rx_on','carrier_2m','no_tx'):
  if name in spectra:
   f,p,fr,pr,*_=spectra[name];axes[0].plot(f,10*np.log10(np.maximum(p,1e-30)),label=name);axes[1].plot(fr,10*np.log10(np.maximum(pr,1e-30)),label=name)
 for ax in axes:ax.set_xlim(0,12000);ax.set_ylim(-140,5);ax.grid(alpha=.25);ax.legend()
 axes[0].set_ylabel('Live PCM PSD (dBFS/Hz)');axes[1].set_ylabel('Raw discriminator PSD (dB/Hz)');axes[1].set_xlabel('Frequency (Hz)');fig.savefig(OUT/'comparison.png',dpi=150)
