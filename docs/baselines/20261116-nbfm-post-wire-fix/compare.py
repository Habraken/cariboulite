"""Compare matching historical/current NBFM power/rate points; no RF control."""
import argparse,csv,json,os,pathlib,statistics
os.environ.setdefault('MPLCONFIGDIR','/tmp/cariboulite-mpl-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('current',type=pathlib.Path);p.add_argument('historical',type=pathlib.Path);p.add_argument('out',type=pathlib.Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
now=json.loads(a.current.read_text());old=json.loads(a.historical.read_text())
key=lambda r:(r['axis'],r['rx_rate_hz'],r['tx_rate_hz'],r['tx_power_dbm'])
old={key(r):r for r in old};rows=[]
for n in now:
 o=old[key(n)];r={k:n[k] for k in ('point','axis','rx_rate_hz','tx_rate_hz','tx_power_dbm')}
 assert n['tx_pa_code']==o['tx_pa_code']
 for m in ('rssi_dbm','sinad_db','noise_rms'):
  r[m+'_old']=(o[m] or {}).get('mean');r[m+'_new']=(n[m] or {}).get('mean')
  r[m+'_delta']=r[m+'_new']-r[m+'_old'] if r[m+'_new'] is not None and r[m+'_old'] is not None else None
 rows.append(r)
with (a.out/'comparison.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary={}
for axis in ('rx','tx'):
 g=[r for r in rows if r['axis']==axis and r['tx_power_dbm'] is not None]
 summary[axis]={}
 for name,group in [('all',g),('weak_minus12',[r for r in g if r['tx_power_dbm']==-12]),('zero',[r for r in g if r['tx_power_dbm']==0]),('plus8',[r for r in g if r['tx_power_dbm']==8])]:
  summary[axis][name]={}
  for m in ('rssi_dbm','sinad_db'):
   v=[r[m+'_delta'] for r in group]
   summary[axis][name][m]={'min':min(v),'max':max(v),'mean':statistics.mean(v),'n':len(v)}
ref={r['tx_power_dbm']:r for r in now if r['axis']=='rx' and r['rx_rate_hz']==4000000 and r['tx_power_dbm'] is not None}
repeat=[r for r in now if r['axis']=='tx' and r['tx_rate_hz']==4000000 and r['tx_power_dbm'] is not None]
summary['repeat_4M']={m:max(abs(r[m]['mean']-ref[r['tx_power_dbm']][m]['mean']) for r in repeat) for m in ('rssi_dbm','sinad_db')}
summary['max_point_sinad_spread']=max(r['sinad_db']['max']-r['sinad_db']['min'] for r in now if r['sinad_db'])
summary['note']='Deltas are current minus historical mean dB at matching requested power and PA code; not causal effects or independent observations.'
(a.out/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
rates=[4000000,2000000,1333333,1000000,800000,666667,500000,400000];labels=['4M','2M','4/3M','1M','800k','2/3M','500k','400k']
for metric,ylabel in [('sinad_db','Change in estimated SINAD (dB)'),('rssi_dbm','Change in raw RSSI (dB)')]:
 fig,axes=plt.subplots(1,2,figsize=(12,5),sharey=True)
 for ax,axis in zip(axes,('rx','tx')):
  ax.axhline(0,color='black',lw=.8)
  for rate,label in zip(rates,labels):
   g=[r for r in rows if r['axis']==axis and r[axis+'_rate_hz']==rate and r['tx_power_dbm'] is not None]
   ax.plot([r['tx_power_dbm'] for r in g],[r[metric+'_delta'] for r in g],'o-',ms=3,label=label)
  ax.set_title(axis.upper()+' rates; other direction fixed at 4 MS/s');ax.set_xlabel('Requested TX power (dBm)');ax.grid(alpha=.2)
 axes[0].set_ylabel(ylabel);axes[1].legend(ncol=2,fontsize=8)
 fig.suptitle('Current minus pre-wire-fix NBFM sweep | 90 dB nominal link')
 fig.text(.5,.01,'Historical comparison; uncalibrated RF levels. Changes include hardware state and the complete intervening software update.',ha='center',fontsize=8)
 fig.tight_layout(rect=(0,.03,1,.94));fig.savefig(a.out/(metric+'-delta.svg'));fig.savefig(a.out/(metric+'-delta.png'),dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))
