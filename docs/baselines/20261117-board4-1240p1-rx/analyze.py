"""Summarize recorded native-rate power sweeps; use matplotlib for figures."""
import argparse,csv,json,os,pathlib,statistics,sys
os.environ.setdefault('MPLCONFIGDIR','/tmp/cariboulite-mpl-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser();parser.add_argument('source',type=pathlib.Path);parser.add_argument('destination',type=pathlib.Path);parser.add_argument('--partial-log',action='store_true');args=parser.parse_args()
out=args.destination;out.mkdir(parents=True,exist_ok=True)
if args.partial_log:
 rows=[json.loads(s[7:]) for s in args.source.read_text().splitlines() if s.startswith('RESULT ')];meta={'partial':True}
else:
 rows=json.loads((args.source/'results.json').read_text());meta=json.loads((args.source/'metadata.json').read_text())
 failed=json.loads((args.source/'startup-failures.json').read_text()) if (args.source/'startup-failures.json').exists() else []
 expected=(8-len(failed))*15
 assert len(rows)==expected and len({r['point'] for r in rows})==expected
 assert sum(r['tx_power_dbm'] is not None for r in rows)==(8-len(failed))*14
 meta['startup_failures']=failed
 for r in rows:
  assert r['packets']>=20 and r['rssi_dbm'] and r['rssi_dbm']['n']>=20,r
  # Nonzero counters are qualification failures retained in results.csv.
  # Retain unavailable SINAD at weak levels rather than discarding points.
 compact=[{k:v for k,v in r.items() if k not in ['rx_config','tx_config']} for r in rows]
 (out/'summary.json').write_text(json.dumps(compact,indent=2)+'\n')
 meta['profiles']={}
 for r in rows:
  key=r['axis']+'_'+str(r[r['axis']+'_rate_hz'])
  if key not in meta['profiles']:meta['profiles'][key]={'rx':r['rx_config']['profile']['rx'],'tx':r['tx_config']['profile']['tx'],'rx_observed':r['rx_config']['observed']['rx'],'tx_observed':r['tx_config']['observed']['tx']}
 meta['run_id']=args.source.name
 meta['backends']={}
 for role in ['rx','tx']:
  ready=[json.loads(p.read_text())['metadata'] for p in args.source.glob('*/'+role+'-ready.json')]
  assert ready and len({r['binary_sha256'] for r in ready})==1
  meta['backends'][role]=ready[0]
 import hashlib,datetime
 meta['source_manifest_sha256']=hashlib.sha256(pathlib.Path(__file__).with_name('source.sha256').read_bytes()).hexdigest()
 meta['recording_end_utc_host_clock']=datetime.datetime.fromtimestamp((args.source/'results.json').stat().st_mtime,datetime.timezone.utc).isoformat()
 meta['date_note']='Board #4 1240.1MHz RX sweep on 2026-11-17; timestamps and run IDs are UTC.'
 meta['module_images']=json.loads(pathlib.Path(__file__).with_name('release-check.json').read_text())
 meta['collector_legacy_shared_reference']=meta.pop('shared_reference',None)
 meta['shared_reference']='RX axis only; TX fixed at4MS/s; no independent repeated4Mreference curve'
 meta['source_commit_meaning']='Frequency-only collector variation of the committed LED1-low sweep; see setup repository_head and actual backend/library hashes; no production code changes for1240.1MHz'
 (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
 columns=['point','axis','rx_rate_hz','tx_rate_hz','rx_divider','tx_divider','rx_exact_rate_hz','tx_exact_rate_hz','tx_power_dbm','tx_pa_code','tx_model_power_dbm','nominal_input_dbm','packets','sinad_valid_fraction','gate_open_fraction']
 for metric in ['rssi_dbm','sinad_db','noise_rms']:columns += [metric+'_'+k for k in ['mean','min','max','median','stdev','n']]
 rxerrors=list(rows[0]['rx_errors']);txerrors=['driver_underruns','writer_errors','queue_drops']
 columns += ['rx_'+k+'_max' for k in rxerrors]+['tx_'+k+'_max' for k in txerrors]
 with (out/'results.csv').open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=columns,lineterminator='\n');w.writeheader()
  for r in rows:
   flat={k:r.get(k) for k in columns};flat['rx_exact_rate_hz']=4000000/r['rx_divider'];flat['tx_exact_rate_hz']=4000000/r['tx_divider']
   for metric in ['rssi_dbm','sinad_db','noise_rms']:
    for k in ['mean','min','max','median','stdev','n']:flat[metric+'_'+k]=(r[metric] or {}).get(k)
   for k in rxerrors:flat['rx_'+k+'_max']=r['rx_errors'][k]['max']
   for k in txerrors:flat['tx_'+k+'_max']=r['tx_errors'].get(k)
   w.writerow(flat)

rates=[4000000,2000000,1333333,1000000,800000,666667,500000,400000]
labels=['4 MS/s','2 MS/s','4/3 MS/s','1 MS/s','800 kS/s','2/3 MS/s','500 kS/s','400 kS/s']

old=json.loads((pathlib.Path(__file__).resolve().parents[1]/'20261116-board4-led1-low-rx/summary.json').read_text())
old={(r['rx_rate_hz'],r['tx_power_dbm']):r for r in old}
comparison=[]
for r in rows:
 o=old[(r['rx_rate_hz'],r['tx_power_dbm'])]
 c={'point':r['point'],'rx_rate_hz':r['rx_rate_hz'],'tx_power_dbm':r['tx_power_dbm'],'pa_code_1240p1':r['tx_pa_code'],'pa_code_430p1':o['tx_pa_code']}
 for metric in ['rssi_dbm','sinad_db']:
  c[metric+'_1240p1']=(r[metric] or {}).get('mean');c[metric+'_430p1']=(o[metric] or {}).get('mean')
  c[metric+'_delta']=c[metric+'_1240p1']-c[metric+'_430p1'] if c[metric+'_1240p1'] is not None and c[metric+'_430p1'] is not None else None
 comparison.append(c)
with (out/'comparison-430p1.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(comparison[0]),lineterminator='\n');w.writeheader();w.writerows(comparison)
summary={'points':len(rows),'transmitting_points':sum(r['tx_power_dbm'] is not None for r in rows),'transport_error_points':[r['point'] for r in rows if any(v['max'] for v in r['rx_errors'].values()) or any(r['tx_errors'].values())],'transmitting_points_with_full_valid_sinad':sum(r['sinad_valid_fraction']==1 for r in rows if r['tx_power_dbm'] is not None),'comparison_note':'Same board, different RF frequencies and dates; uncalibrated link with frequency-dependent TX/frontend/attenuator/cable behavior; no isolated receiver-gain or sensitivity claim'}
for metric in ['rssi_dbm','sinad_db']:
 v=[c[metric+'_delta'] for c in comparison if c['tx_power_dbm'] is not None and c[metric+'_delta'] is not None]
 summary[metric+'_1240p1_minus_430p1']={'n':len(v),'mean':statistics.mean(v),'min':min(v),'max':max(v)} if v else None
(out/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
for metric,label,name in [('rssi_dbm','Raw RSSI (dBm)','rssi'),('sinad_db','Estimated SINAD (dB)','sinad')]:
 fig,axes=plt.subplots(1,2,figsize=(12,5))
 for rate,rate_label in zip(rates,labels):
  g=[r for r in rows if r['rx_rate_hz']==rate and r['tx_power_dbm'] is not None]
  power=[r['tx_power_dbm'] for r in g]
  values=lambda key:[(r[metric] or {}).get(key,float('nan')) for r in g]
  axes[0].plot(power,values('mean'),'o-',ms=3,label=rate_label);axes[0].fill_between(power,values('min'),values('max'),alpha=.08)
  matched=[x for x in comparison if x['rx_rate_hz']==rate and x['tx_power_dbm'] is not None]
  axes[1].plot([x['tx_power_dbm'] for x in matched],[x[metric+'_delta'] if x[metric+'_delta'] is not None else float('nan') for x in matched],'o-',ms=3,label=rate_label)
 axes[0].set_title('Board #4 LED1=0, 1240.1 MHz');axes[0].set_ylabel(label)
 axes[1].set_title('Same board: 1240.1 minus 430.1 MHz');axes[1].set_ylabel('Difference (dB)');axes[1].axhline(0,color='black',lw=.8)
 for ax in axes:
  ax.set_xlabel('Requested TX power (dBm)');ax.set_xticks(list(range(-12,15,2)));ax.grid(alpha=.2);ax.axvspan(9,14.5,color='.94',zorder=0)
 axes[1].legend(ncol=2,fontsize=8)
 fig.suptitle('NBFM | original TX fixed at 4 MS/s | nominal 90 dB coax link')
 fig.text(.5,.01,'Bands: observed min/max, not uncertainty. Link uncalibrated; +10/+12/+14 repeat PA31.',ha='center',fontsize=8)
 fig.tight_layout(rect=(0,.04,1,.94));fig.savefig(out/(name+'.svg'));fig.savefig(out/(name+'.png'),dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))
