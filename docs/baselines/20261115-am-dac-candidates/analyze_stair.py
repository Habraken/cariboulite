from pathlib import Path
import numpy as np,json,sys,csv
folder=Path(sys.argv[1]);allrows=[]
pattern=np.repeat(np.array([v for k in range(106) for v in (2001,1891+2*k)]),100).astype(float);period=len(pattern)
for path in sorted(folder.glob('*.iq16')):
 raw=np.fromfile(path,dtype='<i2').reshape(-1,2);assert len(raw)==2000000
 z=raw[:,0].astype(float)+1j*raw[:,1];n=len(z);freq=np.fft.fftfreq(n,1e-6);fft=np.fft.fft(z);ix=np.flatnonzero((abs(freq)>10000)&(abs(freq)<15000));carrier=freq[ix[np.argmax(abs(fft[ix]))]]
 env=abs(np.fft.ifft(fft*(abs(freq-carrier)<10000)))[::20];idx=np.arange(len(env));keep=(idx>=5000)&(idx<len(env)-5000);counts=np.bincount(idx[keep]%period,minlength=period);fold=np.bincount(idx[keep]%period,weights=env[keep],minlength=period)/counts
 rail_count=int(np.count_nonzero(abs(raw.astype(float))>=4095))
 corr=np.fft.ifft(np.fft.fft(fold-fold.mean())*np.conj(np.fft.fft(pattern-pattern.mean()))).real;lag=int(np.argmax(corr));aligned=np.roll(fold,-lag)
 print(path.name,'lag',lag,'carrier',carrier,'correlation',np.corrcoef(aligned,pattern)[0,1])
 for k in range(106):
  ratios=[]
  for cycle in range(-1,8):
   ref=lag+cycle*period+200*k
   target=ref+100;nextref=ref+200
   if ref+25<5000 or nextref+75>len(env)-5000:continue
   r0=np.mean(env[ref+25:ref+75]);r1=np.mean(env[nextref+25:nextref+75]);a=np.mean(env[target+25:target+75]);ratios.append(a/((r0+r1)/2))
  if not ratios:continue
  row={'repeat':path.stem,'input_i_code':1891+2*k,'reference_i_code':2001,'mean_normalized_output':float(np.mean(ratios)),'std_normalized_output':float(np.std(ratios)),'cycles':len(ratios),'ideal_normalized_output':(1891+2*k)/2001,'adc_rail_samples':rail_count};allrows.append(row)
 print('selected',[(r['input_i_code'],round(r['mean_normalized_output'],5),round(r['std_normalized_output'],5)) for r in allrows if r['repeat']==path.stem and (r['input_i_code']%10==1 or r['input_i_code'] in (1951,1953,2015,2017))])
Path('/tmp/am-candidate/stair-analysis.json').write_text(json.dumps(allrows,indent=2)+'\n')
with open('/tmp/am-candidate/stair-analysis.csv','w') as f:w=csv.DictWriter(f,fieldnames=list(allrows[0]),lineterminator='\n');w.writeheader();w.writerows(allrows)
