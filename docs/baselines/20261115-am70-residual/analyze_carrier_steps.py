from pathlib import Path
import json,numpy as np,math
p=Path('/tmp/am70-carrier-recordings/am70-carrier-steps-20261115-220334');rows=[]
for r in json.loads((p/'results.json').read_text()):
 a=np.fromfile(p/(r['case']+'.iq16'),dtype='<i2').reshape(-1,2);assert len(a)==2000000
 z=a[500000:1500000,0].astype(float)+1j*a[500000:1500000,1];f=np.fft.fftfreq(len(z),1e-6);s=np.fft.fft(z);peak=np.flatnonzero((abs(f)>10000)&(abs(f)<15000));hz=f[peak[np.argmax(abs(s[peak]))]]
 amp=float(np.sqrt(np.sum(abs(s[abs(f-hz)<100])**2))/len(z));rx=r['rx_snapshot'];tx=r['tx_snapshot'];assert rx['hardware']['rf24']['registers']['AGCC']==0 and rx['hardware']['rf24']['registers']['AGCS']==21
 count={'rx_iq_drops':rx['rx']['queue']['drops'],'rx_read_errors':rx['rx']['reader']['errors'],'rx_alignment_failures':rx['rx']['alignment']['failures'],'tx_writer_errors':tx['tx']['writer']['errors'],'tx_underruns':tx['tx']['driver']['underrun_periods']};assert not any(count.values()),count
 carrier=2000*10**(-r['backoff_db']/20);code=int(np.rint(carrier))|1
 row={'case':r['case'],'backoff_db':r['backoff_db'],'intended_carrier_code':carrier,'tagged_carrier_code':code,'received_carrier_rms_codes':amp,'relative_received_db':20*math.log10(amp/rows[0]['received_carrier_rms_codes']) if rows else 0,'error_counters':count,'adc_rail_samples':int(np.count_nonzero(abs(a.astype(float))>=4095))};rows.append(row);print(json.dumps(row))
Path('/tmp/am70-carrier-analysis.json').write_text(json.dumps(rows,indent=2)+'\n')
