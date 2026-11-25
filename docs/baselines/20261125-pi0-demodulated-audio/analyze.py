"""Offline audio/IQ analysis; never accesses the radios."""
from pathlib import Path
import json,math,subprocess,sys
import numpy as np
from scipy import signal,optimize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent;DATA=HERE/'data';OUT=HERE/'analysis';OUT.mkdir(exist_ok=True)
FS=48000;RF=1000000
def coefficients(hz,q,kind):
 w=2*np.pi*hz/FS;c=np.cos(w);a=np.sin(w)/(2*q);d=1+a
 b0=((1+c)/2 if kind=='hp' else (1-c)/2 if kind=='lp' else 1)/d
 b1=(-(1+c) if kind=='hp' else 1-c if kind=='lp' else -2*c)/d
 return [b0,b1,b0],[1,-2*c/d,(1-a)/d]
def band(x):
 for hz,kind in ((300,'hp'),(3000,'lp')):
  b,a=coefficients(hz,2**-.5,kind);x=signal.lfilter(b,a,x)
 return x
def notch(x,hz):
 b,a=coefficients(hz,30,'notch');return signal.lfilter(b,a,x)
def sinad(x,hz=1000):
 x=band(x);r=notch(x,hz);power=np.mean(x[FS:]**2);residual=np.mean(r[FS:]**2)
 db=10*np.log10(max(power,1e-30)/max(residual,1e-30))
 return float(db) if power>1e-12 and db>1 else None
def spectrum(x,fs,n=65536):
 return signal.welch(x,fs=fs,window='hann',nperseg=min(n,len(x)),noverlap=min(n,len(x))//2,
   detrend=False if np.iscomplexobj(x) else 'constant',return_onesided=not np.iscomplexobj(x),scaling='density')
def db(x):return 10*np.log10(np.maximum(x,1e-30))
def line_peaks(x,fs=FS,lo=300,hi=8000,exclude=None):
 f,p=spectrum(x,fs);spacing=f[1]-f[0]
 if np.iscomplexobj(x):
  order=np.argsort(f);f=f[order];p=p[order]
 indexes=signal.find_peaks(db(p),prominence=6,distance=max(1,int(15/spacing)))[0]
 indexes=[i for i in indexes if lo<f[i]<hi and (exclude is None or abs(f[i]-exclude)>35)]
 indexes=sorted(indexes,key=lambda i:p[i],reverse=True)[:8]
 return [{'frequency_hz':float(f[i]),'psd_db_per_hz':float(db(p[i]))} for i in indexes]
def fit_frequency(x):
 x=np.asarray(x,dtype=float);x=x-x.mean();nfft=1048576 if len(x)>48000 else 262144
 p=np.abs(np.fft.rfft(x*np.hanning(len(x)),nfft))**2;f=np.fft.rfftfreq(nfft,1/FS)
 region=np.where((f>990)&(f<1010))[0];guess=float(f[region[np.argmax(p[region])]])
 t=np.arange(len(x))/FS
 def projection(hz,ret=False):
  angle=2*np.pi*hz*t;c=np.cos(angle);s=np.sin(angle)
  g=np.array([[np.dot(c,c),np.dot(c,s)],[np.dot(c,s),np.dot(s,s)]])
  v=np.array([np.dot(x,c),np.dot(x,s)]);coef=np.linalg.solve(g,v)
  return (coef,c,s) if ret else -float(v@coef)
 width=2/(len(x)/FS)
 opt=optimize.minimize_scalar(projection,bounds=(guess-width,guess+width),method='bounded',options={'xatol':2e-7})
 assert opt.success
 hz=float(opt.x);coef,_,_=projection(hz,True)
 return hz,float(np.hypot(*coef))
def replay(cap,cfo=0):
 target=cap.with_suffix('.replay.f32' if not cfo else '.centered.f32')
 if not target.exists():
  subprocess.run([str(HERE/'decode-iq'),str(cap.with_suffix('.iq')),str(target),str(RF),format(cfo,'.12g')],check=True)
 a=np.fromfile(target,'<f4').reshape(-1,2)
 # A fresh 50k -> 48k fractional phase may finish one sample below the
 # nominal boundary; retain it without padding or changing the DSP.
 assert abs(len(a)-FS*8)<=1
 return a[:,0].astype(float),a[:,1].astype(float)
def harmonic_amplitudes(raw,hz):
 y=raw[FS:];t=np.arange(len(y))/FS
 columns=[np.ones(len(y))]
 for h in (1,2,3):columns += [np.cos(2*np.pi*h*hz*t),np.sin(2*np.pi*h*hz*t)]
 coef=np.linalg.lstsq(np.column_stack(columns),y,rcond=None)[0]
 a=[float(np.hypot(coef[2*h-1],coef[2*h])) for h in (1,2,3)]
 return {'fundamental_peak_normalized':a[0],'fundamental_peak_deviation_hz_nominal':a[0]*2500,
  'second_harmonic_dbc':float(20*np.log10(max(a[1],1e-15)/a[0])),
  'third_harmonic_dbc':float(20*np.log10(max(a[2],1e-15)/a[0])),
  'second_third_thd_percent':float(100*np.hypot(a[1],a[2])/a[0])}

rows=[];plot_data={};offset_data={};if_plot_data={};if_plans={}
for run in sorted(HERE.glob('[ab]tx_[ab]rx_*')):
 if not run.is_dir() or not (run/'measurements.json').exists():continue
 plan=json.loads((run/'plan.json').read_text())
 for measured in json.loads((run/'measurements.json').read_text()):
  case=measured['case'];cap=DATA/case/'rx/capture'
  if not cap.with_suffix('.json').exists():continue
  info=json.loads(cap.with_suffix('.json').read_text())
  assert info['iq_samples']==RF*8 and info['raw_frames']==info['filtered_frames']==info['pcm_frames']==FS*8
  raw=np.fromfile(cap.with_suffix('.raw.f32'),'<f4').astype(float)
  filtered=np.fromfile(cap.with_suffix('.filtered.f32'),'<f4').astype(float)
  assert np.isfinite(raw).all() and np.isfinite(filtered).all()
  pcm=np.fromfile(cap.with_suffix('.pcm'),'<i2').reshape(-1,info['channels'])[:,0].astype(float)/32768
  chunks=[json.loads(line) for line in cap.with_suffix('.clock.jsonl').read_text().splitlines()]
  assert sum(c['frames'] for c in chunks)==len(raw)
  assert [c['first_sample'] for c in chunks]==list(np.cumsum([0]+[c['frames'] for c in chunks[:-1]]))
  corrections=np.array([c['correction'] for c in chunks]);weights=np.array([c['frames'] for c in chunks])
  iq=np.fromfile(cap.with_suffix('.iq'),'<i2').reshape(-1,2).astype(float);z=iq[:,0]+1j*iq[:,1];del iq
  zi=signal.resample_poly(z,1,20)
  zi=signal.lfilter(signal.firwin(161,7000,fs=50000),[1],zi)
  independent=np.angle(zi[1:]*zi[:-1].conj())*50000/(2*np.pi*2500)
  cfo=float(np.mean(independent[50000:])*2500) if measured['tx_on'] else None
  production,pfiltered=replay(cap)
  centered,cfiltered=replay(cap,cfo) if cfo is not None else (None,None)
  is_tone=measured['name'].startswith('tone')
  hz,amp=fit_frequency(band(raw)[FS:]) if is_tone else (None,None)
  replay_hz=fit_frequency(band(production)[FS:])[0] if is_tone else None
  phz=fit_frequency(pcm[FS:])[0] if is_tone else None
  chz=fit_frequency(band(centered)[FS:])[0] if is_tone else None
  hf=harmonic_amplitudes(raw,hz) if is_tone else {}
  window_hz=[fit_frequency(band(raw)[i*FS:(i+1)*FS])[0] for i in range(1,8)] if is_tone else []
  residual=notch(band(raw),hz) if is_tone else band(raw)
  rf,rp=spectrum(residual[FS:],FS)
  bf,bp=spectrum(band(raw)[FS:],FS);voice=(rf>=300)&(rf<=3000)
  uf,up=spectrum(raw[FS:],FS);whistle_band=(uf>=3000)&(uf<=3800)
  deltas={}
  for section,field in [('queue','drops'),('audio_queue','drops'),('reader','errors'),('playback','underruns'),('playback','write_errors'),('alignment','invalid_frames'),('audio_output','clipped_samples')]:
   deltas[section+'.'+field]=measured['after']['rx']['rx'][section][field]-measured['before']['rx']['rx'][section][field]
  if measured['tx_on']:
   for section,field in [('writer','errors'),('driver','underrun_periods'),('queue','drops')]:
    deltas['tx.'+section+'.'+field]=measured['after']['tx']['tx'][section][field]-measured['before']['tx']['tx'][section][field]
  rssi=measured['after']['rx']['hardware']['rf24']['registers']['RSSI'];rssi=rssi-256 if rssi>=128 else rssi
  row={'case':case,'phase':plan['phase'],'rx_node':plan['rx_node'],'tx_node':plan['tx_node'],'name':measured['name'],
   'tx_if_control_hz':plan.get('tx_if_control_hz'),
   'rssi_after_dbm':rssi,'carrier_offset_in_decoded_iq_hz':cfo,
   'raw_audio_dc_equivalent_hz_nominal':float(raw[FS:].mean()*2500),
   'live_sinad_after_db':measured['after']['rx']['rx']['sinad']['db'],'production_replay_frames':len(production),
   'raw_fixed_notch_sinad_db':sinad(raw),'raw_tracked_notch_sinad_db':sinad(raw,hz) if is_tone else None,
   'replay_fixed_notch_sinad_db':sinad(production),
   'replay_tracked_notch_sinad_db':sinad(production,replay_hz) if is_tone else None,
   'centered_replay_tracked_notch_sinad_db':sinad(centered,chz) if is_tone else None,
   'raw_tone_frequency_hz':hz,'replay_tone_frequency_hz':replay_hz,'pcm_tone_frequency_hz':phz,
   'raw_one_second_frequency_estimates_hz':window_hz,
   'raw_correction_ppm_min_mean_max':[float(corrections.min()*1e6),float(np.average(corrections,weights=weights)*1e6),float(corrections.max()*1e6)],
   'pcm_rms_dbfs':float(20*np.log10(max(np.std(pcm[FS:]),1e-15))),
   'residual_lines':line_peaks(residual[FS:],exclude=hz),
   'estimator_residual_power_fraction_above_3khz':float(np.sum(rp[rf>3000])/np.sum(rp)),
   'raw_3000_3800hz_power_db_normalized':float(10*np.log10(np.sum(up[whistle_band])*(uf[1]-uf[0]))),
   'strict_300_3000hz_spectral_ratio_db':float(10*np.log10(np.sum(bp[voice])/np.sum(rp[voice]))) if is_tone else None,
   'iq_lines':line_peaks(z[RF:],RF,lo=-15000,hi=15000),
   'raw_fullband_lines':line_peaks(raw[FS:],lo=100,hi=20000,exclude=hz),
   'independent_lines':line_peaks(independent[50000:],50000,exclude=hz),
   'health_deltas':deltas,'harmonics':hf}
  rows.append(row)
  print(case,'CFO',cfo,'tone',hz,'fixed/tracked/centered',row['raw_fixed_notch_sinad_db'],row['raw_tracked_notch_sinad_db'],row['centered_replay_tracked_notch_sinad_db'],'residual',row['residual_lines'][:2],flush=True)
  if '--plot' in sys.argv:
   f,p=spectrum(z[RF:],RF,262144);order=np.argsort(f)
   fa,pa=spectrum(raw[FS:],FS);fp,pp=spectrum(production[FS:],FS);fi,pi=spectrum(independent[50000:],50000)
   fc,pc=spectrum(pcm[FS:],FS);fr,pr=spectrum(residual[FS:],FS)
   fig,ax=plt.subplots(4,1,figsize=(12,12),layout='constrained')
   ax[0].plot(f[order],db(p[order]));ax[0].set_xlim(-6000,6000);ax[0].set_ylabel('IQ PSD (ADC units²/Hz)')
   ax[0].set_title(case+(' — IQ carrier offset %.2f Hz'%cfo if cfo is not None else ' — no transmitter'))
   ax[1].plot(fa,db(pa),label='live raw discriminator');ax[1].plot(fp,db(pp),alpha=.7,label='production replay');ax[1].plot(fi,db(pi),alpha=.5,label='independent FM');ax[1].set_xlim(0,8000);ax[1].set_ylabel('Normalized audio PSD (dB/Hz)');ax[1].legend()
   ax[2].plot(fc,db(pc));ax[2].set_xlim(0,8000);ax[2].set_ylabel('Playback PSD (dBFS/Hz)')
   voice_plot=(fr>=300)&(fr<=3000)
   ax[3].plot(fr[voice_plot],db(pr[voice_plot]));ax[3].set_xlim(300,3000);ax[3].set_ylabel('Residual PSD (dB/Hz)');ax[3].set_xlabel('Frequency (Hz)')
   for a in ax:a.grid(alpha=.25)
   fig.savefig(OUT/(case+'.png'),dpi=140);plt.close(fig)
   plot_data[(plan['phase'],measured['name'])]=(fa,pa,fc,pc,fr,pr)
   if plan['phase']=='atx_brx' and measured['name'] in ('carrier','offset_p5k','offset_m5k'):
    offset_data[measured['name']]=(f[order],p[order],fa,pa,cfo)
   if plan.get('tx_if_control_hz'):
    if_plot_data[(run.name,measured['name'])]=(fa,pa,fc,pc,fr,pr)
    if_plans[run.name]=plan
(OUT/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
if '--plot' in sys.argv and len({r['phase'] for r in rows})==2:
 fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
 for phase,label in [('btx_arx','Original RX (pi0a)'),('atx_brx','Repaired #2 RX (pi0b)')]:
  for name,col in [('carrier',0),('tone_a',1)]:
   f,p,fc,pc,fr,pr=plot_data[(phase,name)]
   axes[0,col].plot(f,db(p),label=label)
   voice=(fr>=300)&(fr<=3000);axes[1,col].plot(fr[voice],db(pr[voice]),label=label)
 for col,title in enumerate(('Clean carrier','1 kHz test tone')):
  axes[0,col].set_title(title);axes[0,col].set_xlim(0,8000);axes[0,col].set_ylabel('Raw audio PSD (dB/Hz)')
  axes[1,col].set_xlim(300,3000);axes[1,col].set_ylabel('Voice-band residual PSD (dB/Hz)');axes[1,col].set_xlabel('Frequency (Hz)')
 for a in axes.flat:a.grid(alpha=.25);a.legend()
 fig.savefig(OUT/'direction-comparison.png',dpi=150);fig.savefig(OUT/'direction-comparison.svg');plt.close(fig)
if '--plot' in sys.argv and set(offset_data)=={'carrier','offset_p5k','offset_m5k'}:
 fig,axes=plt.subplots(2,3,figsize=(16,8),layout='constrained')
 for col,(name,title) in enumerate((('carrier','Nominal RX tuning'),('offset_p5k','RX +5 kHz'),('offset_m5k','RX -5 kHz'))):
  f,p,fa,pa,cfo=offset_data[name];mask=(f>=-10000)&(f<=10000)
  axes[0,col].plot(f[mask],db(p[mask]));axes[0,col].axvline(cfo,color='tab:green',ls='--',label='Wanted carrier')
  axes[0,col].axvline(3180,color='tab:red',ls=':',label='Receiver-relative spur (~3.18 kHz)')
  axes[0,col].set_title(title);axes[0,col].set_xlim(-10000,10000);axes[0,col].set_xlabel('Decoded IQ frequency (Hz)');axes[0,col].set_ylabel('IQ PSD (ADC units²/Hz)');axes[0,col].legend(fontsize=8)
  mask=(fa>=100)&(fa<=10000);axes[1,col].plot(fa[mask],db(pa[mask]));axes[1,col].set_xlim(100,10000)
  axes[1,col].set_xlabel('Audio frequency (Hz)');axes[1,col].set_ylabel('Raw audio PSD (dB/Hz)')
 for ax in axes.flat:ax.grid(alpha=.25)
 fig.savefig(OUT/'receiver-offset-localization.png',dpi=150);fig.savefig(OUT/'receiver-offset-localization.svg');plt.close(fig)
if '--plot' in sys.argv and len(if_plans)==3:
 fig,axes=plt.subplots(2,3,figsize=(15,8),layout='constrained',sharey='row')
 for col,name in enumerate(sorted(if_plans)):
  mhz=if_plans[name]['tx_if_control_hz']/1e6
  f,p,fc,pc,fr,pr=if_plot_data[(name,'carrier_probe')];m=(f>=100)&(f<=8000)
  axes[0,col].plot(f[m],db(p[m]));axes[0,col].set_title(('A: ' if col==0 else 'B: ' if col==1 else 'A return: ')+f'TX IF {mhz:.0f} MHz')
  f,p,fc,pc,fr,pr=if_plot_data[(name,'tone_probe')];m=(fr>=300)&(fr<=8000)
  axes[1,col].plot(fr[m],db(pr[m]))
  axes[0,col].set_ylabel('Clean-carrier raw audio PSD (dB/Hz)')
  axes[1,col].set_ylabel('Tone residual PSD (dB/Hz)')
  for row_index in (0,1):
   axes[row_index,col].set_xlim(100 if row_index==0 else 300,8000);axes[row_index,col].set_xlabel('Audio frequency (Hz)');axes[row_index,col].grid(alpha=.25)
 fig.suptitle('Pi0a TX → pi0b RX; RF held at 430.1 MHz, nominal 0 dBm and 90 dB link; RX IF remains 2495 MHz')
 fig.savefig(OUT/'tx-if-aba-comparison.png',dpi=150);fig.savefig(OUT/'tx-if-aba-comparison.svg');plt.close(fig)
