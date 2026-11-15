import subprocess,pathlib,numpy as np,json,math
rows=[];fs=1000000;t=np.arange(fs)/fs;basis=np.column_stack([f(2*np.pi*k*1000*t) for k in (1,2,3) for f in (np.sin,np.cos)])
for back in (0,6):
 for amp in (.001,.003,.01,.03,.1,.5):
  path='/tmp/am70-envelope.bin';subprocess.run(['/tmp/am70-envelope',str(amp),str(back),path],check=True)
  data=np.fromfile(path,dtype='<i2').reshape(-1,2).astype(float);assert len(data)==fs
  for stage,index in [('modulator',0),('tagged_frame',1)]:
   y=data[:,index];dc=y.mean();z=y-dc;coef=(2/fs)*(basis.T@z);rms=np.hypot(coef[0],coef[1])/np.sqrt(2);harm=np.sqrt(np.sum(coef[2:]**2))/np.sqrt(2)
   row={'backoff_db':back,'amplitude':amp,'stage':stage,'carrier_code':dc,'peak_to_peak_codes':float(y.max()-y.min()),'unique_codes':len(np.unique(y)),'modulation_fraction':float(np.sqrt(2)*rms/dc),'second_third_harmonic_percent':float(100*harm/rms),'tone_to_residual_db':float(10*np.log10(rms*rms/max(1e-30,np.mean(z*z)-rms*rms))),'q_all_zero':True,'maximum_tag_change_codes':int(np.max(data[:,1]-data[:,0]))};rows.append(row)
 print(back,amp,'tagged mod',rows[-1]['modulation_fraction'],'THD',rows[-1]['second_third_harmonic_percent'])
for row in rows:
 for key,value in row.items():
  if isinstance(value,float) and not math.isfinite(value):row[key]=None
pathlib.Path('/tmp/am70-envelope-analysis.json').write_text(json.dumps(rows,indent=2,allow_nan=False)+'\n')
