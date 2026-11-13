import pathlib,json,numpy as np,sys
# A free-frequency least-squares fit avoids calling clock offset distortion.
def tone_fit(y,fs=48000):
 y=np.asarray(y,dtype=float);y=y-y.mean();n=len(y);t=np.arange(n)/fs
 spec=abs(np.fft.rfft(y*np.hanning(n)));freq=np.fft.rfftfreq(n,1/fs)
 ix=np.flatnonzero((freq>950)&(freq<1050));k=ix[np.argmax(spec[ix])];f=freq[k]
 def fit(f):
  a=np.column_stack((np.sin(2*np.pi*f*t),np.cos(2*np.pi*f*t),np.ones(n)))
  co=np.linalg.lstsq(a,y,rcond=None)[0];r=y-a@co
  return float(np.dot(r,r)),co,r
 lo,hi=f-fs/n,f+fs/n
 for _ in range(32):
  a=lo+(hi-lo)*.38196601125;b=hi-(hi-lo)*.38196601125
  if fit(a)[0]<fit(b)[0]:hi=b
  else:lo=a
 f=(lo+hi)/2;err,co,r=fit(f);amp=float(np.hypot(co[0],co[1]))
 harmonics=[]
 for m in [2,3,4,5]:
  a=np.column_stack((np.sin(2*np.pi*f*m*t),np.cos(2*np.pi*f*m*t)))
  c=np.linalg.lstsq(a,r,rcond=None)[0];harmonics.append(20*np.log10(max(np.hypot(*c)/amp,1e-12)))
 return {'samples':n,'frequency_hz':f,'amplitude':amp,'fit_snr_db':float(10*np.log10(n*amp*amp/2/err)), 'harmonic_2_to_5_dbc':[float(v) for v in harmonics],'peak':float(np.max(abs(y)))}
if __name__=='__main__':
 root=pathlib.Path(sys.argv[1]);results=[]
 for p in sorted(root.glob('*/playback.s16')):
  # The Jabra negotiated mono in the backend log. Analyze trailing 4 s.
  y=np.fromfile(p,dtype='<i2').astype(float);r=tone_fit(y[-192000:]);r.update(case=p.parent.name,clipped_samples=int(np.sum(abs(y)>=32767)));results.append(r);print(json.dumps(r),flush=True)
 (root/'audio-analysis.json').write_text(json.dumps(results,indent=2))
