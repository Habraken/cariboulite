import numpy as np

def measure(path,fs,if_center=12500):
 raw=np.fromfile(path,dtype='<i2').reshape(-1,2);z=raw[:,0].astype(float)+1j*raw[:,1];n=len(z);f=np.fft.fftfreq(n,1/fs);window=np.hanning(n);spec=abs(np.fft.fft(z*window))**2
 candidates=np.flatnonzero((abs(f)>if_center-2000)&(abs(f)<if_center+2000));carrier=float(f[candidates[np.argmax(spec[candidates])]])
 reference=spec[abs(f-carrier)<100].sum();relative=f-carrier
 def dbc(p):return float(10*np.log10(max(1e-30,p)/max(reference,1e-30)))
 offsets={str(o):dbc(spec[abs(relative-o)<100].sum()) for o in (-201600,-151200,-100800,-50400,-25200,25200,50400,100800,151200,201600)}
 outside=(abs(relative)>3500)&(abs(relative)<60000);outside_no_dc=outside&(abs(f)>100)
 envelope=abs(np.fft.ifft(np.fft.fft(z)*(abs(relative)<6000)))[::10];af=fs/10;dc=envelope.mean();env=envelope/dc-1
 af_f=np.fft.fftfreq(len(env),1/af);env=np.fft.ifft(np.fft.fft(env)*((abs(af_f)>=300)&(abs(af_f)<=3300))).real
 cut=int(.01*af);env=env[cut:-cut];t=np.arange(len(env))/af
 basis=np.column_stack([fun(2*np.pi*k*1000*t) for k in (1,2,3) for fun in (np.sin,np.cos)]+[np.ones(len(t))]);coefs=np.linalg.lstsq(basis,env,rcond=None)[0];fund=float(np.hypot(*coefs[:2]));harm=float(np.linalg.norm(coefs[2:6]));model=basis[:,:2]@coefs[:2];res=float(np.mean((env-model)**2));tone=fund*fund/2
 wide=(abs(relative)>3500)&(abs(relative)<200000)&(abs(f)>100)
 return {'outside_3p5_to_200k_excluding_RX_DC_dbc':dbc(spec[wide].sum()),'inband_envelope_power_dbc':float(10*np.log10(max(float(np.mean(env*env)),1e-30))),'samples':n,'carrier_if_hz':carrier,'carrier_rms_codes':float(np.sqrt(reference/(window@window*n))),'modulation_fraction':fund,'second_third_harmonic_percent':100*harm/max(fund,1e-15),'inband_tone_residual_db':float(10*np.log10(max(tone,1e-30)/max(res,1e-30))),'outside_3p5_to_60k_dbc':dbc(spec[outside].sum()),'outside_3p5_to_60k_excluding_RX_DC_dbc':dbc(spec[outside_no_dc].sum()),'offset_200hz_band_dbc':offsets,'adc_rail_samples':int(np.count_nonzero(abs(raw.astype(float))>=4095))}
