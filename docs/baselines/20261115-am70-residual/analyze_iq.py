from pathlib import Path
import numpy as np,json

rows=[]
for path in sorted(Path('/tmp/am70-iq-recordings').glob('*/*.iq16')):
 raw=np.fromfile(path,dtype='<i2').reshape(-1,2);assert len(raw)==2000000
 z=raw[500000:1500000,0].astype(float)+1j*raw[500000:1500000,1];n=len(z);fs=1000000
 fft=np.fft.fft(z*np.hanning(n));power=abs(fft)**2;freq=np.fft.fftfreq(n,1/fs)
 region=np.flatnonzero((abs(freq)>10000)&(abs(freq)<15000));peak=region[np.argmax(power[region])];carrier=freq[peak]
 t=np.arange(n)/fs;env=abs(np.fft.ifft(np.fft.fft(z)*(abs(freq-carrier)<6000)))[::20][1000:-1000];af=50000;t=np.arange(len(env))/af;dc=env.mean();y=env/dc-1
 def fit(f):
  a=np.column_stack((np.sin(2*np.pi*f*t),np.cos(2*np.pi*f*t),np.ones(len(t))));c=np.linalg.lstsq(a,y,rcond=None)[0];return np.sum((y-a@c)**2),c
 lo,hi=998.8,1000.2;ratio=(np.sqrt(5)-1)/2
 for _ in range(32):
  a=hi-ratio*(hi-lo);b=lo+ratio*(hi-lo)
  if fit(a)[0]<fit(b)[0]:hi=b
  else:lo=a
 f=(lo+hi)/2;res,c=fit(f);fund=np.hypot(c[0],c[1]);harm=[]
 for k in (2,3):
  a=np.column_stack((np.sin(2*np.pi*k*f*t),np.cos(2*np.pi*k*f*t),np.ones(len(t))));c=np.linalg.lstsq(a,y,rcond=None)[0];harm.append(np.hypot(c[0],c[1]))
 carrierpower=power[abs(freq-carrier)<30].sum();spurs={str(off):float(10*np.log10(max(1e-30,power[abs(freq-carrier-off)<30].sum())/carrierpower)) for off in (-50400,-25200,25200,50400)}
 r={'run':path.parent.name,'file':path.name,'carrier_if_hz':float(carrier),'fitted_tone_hz':float(f),'modulation_fraction':float(fund),'second_third_harmonic_percent':float(100*np.hypot(*harm)/fund),'tone_to_residual_db':float(10*np.log10((fund**2/2)/(res/len(y)))),'adc_rail_samples':int(np.count_nonzero(abs(raw.astype(float))>=4095)),'relative_offset_band_power_dbc':spurs,'spectral_bandwidth_hz':60};rows.append(r);print(json.dumps(r))
Path('/tmp/am70-iq-analysis.json').write_text(json.dumps(rows,indent=2)+'\n')
