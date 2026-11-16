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
  if r['tx_power_dbm'] is not None:assert r['sinad_db'] and r['sinad_valid_fraction']==1,r
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
 meta['date_note']='Board #4 RX-only run on 2026-11-16; timestamps and run IDs are UTC.'
 meta['module_images']=json.loads(pathlib.Path(__file__).with_name('release-check.json').read_text())
 meta['collector_legacy_shared_reference']=meta.pop('shared_reference',None)
 meta['shared_reference']='RX axis only; TX fixed at4MS/s; no independent repeated4Mreference curve'
 meta['source_commit_meaning']='Binary implementation base3edde8e; qualification repository HEAD222c8a7; no production software changes'
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
old=json.loads((pathlib.Path(__file__).resolve().parents[1]/'20261116-nbfm-post-wire-fix/summary.json').read_text())
old={(r['rx_rate_hz'],r['tx_power_dbm']):r for r in old if r['axis']=='rx'}
comparison=[]
for r in rows:
 assert r['axis']=='rx' and r['tx_rate_hz']==4000000
 o=old[(r['rx_rate_hz'],r['tx_power_dbm'])];assert r['tx_pa_code']==o['tx_pa_code']
 c={k:r[k] for k in ['point','rx_rate_hz','tx_power_dbm','tx_pa_code']}
 for m in ['rssi_dbm','sinad_db','noise_rms']:
  c[m+'_board1']=(o[m] or {}).get('mean');c[m+'_board4']=(r[m] or {}).get('mean')
  c[m+'_delta']=c[m+'_board4']-c[m+'_board1'] if c[m+'_board4'] is not None and c[m+'_board1'] is not None else None
 comparison.append(c)
with (out/'comparison.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(comparison[0]));w.writeheader();w.writerows(comparison)
summary={'transport_error_points':[r['point'] for r in rows if any(v['max']!=0 for v in r['rx_errors'].values()) or any(v!=0 for v in r['tx_errors'].values())],'transmitting_points':sum(r['tx_power_dbm'] is not None for r in rows),'off_points':sum(r['tx_power_dbm'] is None for r in rows),'receiver_uuid':'c2440a8d-3852-4c44-b96f-1b64a962e7dc','reference_receiver_uuid':'d8810044-c444-46f4-a548-703412cbf971','note':'Board4 minus board1 at matched requests; uncalibrated link estimate, not a causal hardware-quality comparison.'}
for m in ['rssi_dbm','sinad_db']:
 v=[c[m+'_delta'] for c in comparison if c['tx_power_dbm'] is not None];summary[m+'_delta']={'min':min(v),'max':max(v),'mean':statistics.mean(v)}
summary['max_within_point_sinad_range']=max(r['sinad_db']['max']-r['sinad_db']['min'] for r in rows if r['sinad_db'])
(out/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
for metric,ylabel in [('sinad_db','Estimated SINAD (dB)'),('rssi_dbm','Raw RSSI (dBm)')]:
 fig,axes=plt.subplots(1,2,figsize=(12,5))
 for rate,label in zip(rates,labels):
  g=[r for r in rows if r['rx_rate_hz']==rate and r['tx_power_dbm'] is not None]
  axes[0].plot([r['tx_power_dbm'] for r in g],[r[metric]['mean'] for r in g],'o-',ms=3,label=label)
  axes[0].fill_between([r['tx_power_dbm'] for r in g],[r[metric]['min'] for r in g],[r[metric]['max'] for r in g],alpha=.08)
  c=[r for r in comparison if r['rx_rate_hz']==rate and r['tx_power_dbm'] is not None]
  axes[1].plot([r['tx_power_dbm'] for r in c],[r[metric+'_delta'] for r in c],'o-',ms=3,label=label)
 axes[0].set_title('Board #4 RX; original TX fixed at 4 MS/s');axes[0].set_ylabel(ylabel)
 axes[1].set_title('Board #4 minus board #1');axes[1].set_ylabel('Difference (dB)');axes[1].axhline(0,color='black',lw=.8)
 for ax in axes:
  ax.set_xlabel('Requested TX power (dBm)');ax.set_xticks(list(range(-12,15,2)));ax.grid(alpha=.2);ax.axvspan(9,14.5,color='.94',zorder=0)
 axes[1].legend(ncol=2,fontsize=8)
 fig.suptitle('430.100 MHz | 90 dB nominal link | 1 kHz NBFM tone')
 fig.text(.5,.01,'Shading: observed min-max, not uncertainty. Board #4 repeat sweep; RF/RSSI uncalibrated; PA31 repeats at +10/+12/+14.',ha='center',fontsize=8)
 fig.tight_layout(rect=(0,.04,1,.94));name='sinad' if metric=='sinad_db' else 'rssi';fig.savefig(out/(name+'.svg'));fig.savefig(out/(name+'.png'),dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))

# Also compare with the immediately preceding board, without mixing references.
previous=json.loads((pathlib.Path(__file__).resolve().parents[1]/'20261116-board2-rx/summary.json').read_text())
previous={(p['rx_rate_hz'],p['tx_power_dbm']):p for p in previous}
comparison2=[]
for r in rows:
 o=previous[(r['rx_rate_hz'],r['tx_power_dbm'])];assert r['tx_pa_code']==o['tx_pa_code']
 c={k:r[k] for k in ['point','rx_rate_hz','tx_power_dbm','tx_pa_code']}
 for m in ['rssi_dbm','sinad_db','noise_rms']:
  c[m+'_board2']=(o[m] or {}).get('mean');c[m+'_board4']=(r[m] or {}).get('mean')
  c[m+'_delta']=c[m+'_board4']-c[m+'_board2'] if c[m+'_board4'] is not None and c[m+'_board2'] is not None else None
 comparison2.append(c)
with (out/'comparison-board2.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(comparison2[0]));w.writeheader();w.writerows(comparison2)
summary['comparison_board2']={'receiver_uuid':'36b67ac4-b313-4369-99bd-4b3e09caabfa'}
for m in ['rssi_dbm','sinad_db']:
 v=[c[m+'_delta'] for c in comparison2 if c['tx_power_dbm'] is not None]
 summary['comparison_board2'][m+'_delta']={'min':min(v),'max':max(v),'mean':statistics.mean(v)}
(out/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({'board4_minus_board2':summary['comparison_board2']},indent=2))

# Also compare with the immediately preceding board, without mixing references.
previous=json.loads((pathlib.Path(__file__).resolve().parents[1]/'20261116-board3-rx/summary.json').read_text())
previous={(p['rx_rate_hz'],p['tx_power_dbm']):p for p in previous}
comparison3=[]
for r in rows:
 o=previous[(r['rx_rate_hz'],r['tx_power_dbm'])];assert r['tx_pa_code']==o['tx_pa_code']
 c={k:r[k] for k in ['point','rx_rate_hz','tx_power_dbm','tx_pa_code']}
 for m in ['rssi_dbm','sinad_db','noise_rms']:
  c[m+'_board3']=(o[m] or {}).get('mean');c[m+'_board4']=(r[m] or {}).get('mean')
  c[m+'_delta']=c[m+'_board4']-c[m+'_board3'] if c[m+'_board4'] is not None and c[m+'_board3'] is not None else None
 comparison3.append(c)
with (out/'comparison-board3.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(comparison3[0]));w.writeheader();w.writerows(comparison3)
summary['comparison_board3']={'receiver_uuid':'f23c0015-fd1b-47b7-8314-f5c9e7de0ea7'}
for m in ['rssi_dbm','sinad_db']:
 v=[c[m+'_delta'] for c in comparison3 if c['tx_power_dbm'] is not None]
 summary['comparison_board3'][m+'_delta']={'min':min(v),'max':max(v),'mean':statistics.mean(v)}
(out/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({'board4_minus_board3':summary['comparison_board3']},indent=2))
