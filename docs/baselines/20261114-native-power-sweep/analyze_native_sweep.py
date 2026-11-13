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
 assert len(rows)==240 and len({r['point'] for r in rows})==240
 assert sum(r['tx_power_dbm'] is not None for r in rows)==224
 for r in rows:
  assert r['packets']>=20 and r['rssi_dbm'] and r['rssi_dbm']['n']>=20,r
  assert all(v['max']==0 for v in r['rx_errors'].values()),r
  assert all(v==0 for v in r['tx_errors'].values()),r
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
 manifest=pathlib.Path(__file__).with_name('source.sha256')
 if not manifest.exists():manifest=pathlib.Path('/tmp/cariboulite-native-sweep-source.sha256')
 meta['source_manifest_sha256']=hashlib.sha256(manifest.read_bytes()).hexdigest()
 meta['recording_end_utc']=datetime.datetime.fromtimestamp((args.source/'results.json').stat().st_mtime,datetime.timezone.utc).isoformat()
 module_dir=pathlib.Path(__file__).resolve().parents[3]/'docs/baselines/20261114-tx-window-restart'
 meta['module_images']={host:json.loads((module_dir/(host+'-installed.json')).read_text()) for host in ['pi4g-trixie','pi4f-trixie']}
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
colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00','#56B4E9','#6B6B00','#333333']
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
for metric,title,ylabel,limits in [('sinad_db','Estimated SINAD','SINAD (dB)',(10,28)),('rssi_dbm','Receiver RSSI','Raw signed RSSI register (dBm)',(-132,-65))]:
 fig,axes=plt.subplots(1,2,figsize=(12.4,5.7),sharey=True)
 handles=[]
 for ax,axis in zip(axes,['rx','tx']):
  ax.axvspan(9,14.6,color='.94',zorder=0)
  for rate,label,color in zip(rates,labels,colors):
   group=[r for r in rows if r['axis']==axis and r[axis+'_rate_hz']==rate and r['tx_power_dbm'] is not None and r[metric]]
   group.sort(key=lambda r:r['tx_power_dbm'])
   if not group:continue
   x=[r['tx_power_dbm'] for r in group];mean=[r[metric]['mean'] for r in group]
   line,=ax.plot(x,mean,'o-',ms=3,lw=1.3,color=color,label=label)
   ax.fill_between(x,[r[metric]['min'] for r in group],[r[metric]['max'] for r in group],color=color,alpha=.10)
   if axis=='rx':handles.append(line)
  ax.set_title('RX rates; TX fixed at 4 MS/s' if axis=='rx' else 'TX rates; RX fixed at 4 MS/s')
  ax.set(xlim=(-12.7,14.7),ylim=limits,xlabel='Requested TX power (dBm)');ax.set_xticks(list(range(-12,15,2)));ax.grid(alpha=.2)
  ax.text(.98,.98,'PA code 31 at +10/+12/+14',ha='right',va='top',transform=ax.transAxes,fontsize=8,color='.35')
 axes[0].set_ylabel(ylabel)
 fig.suptitle(title+' after continuity fixes',fontsize=15,y=.98)
 fig.text(.5,.925,'430.100 MHz | 90 dB nominal attenuation | 1 kHz NBFM tone, amplitude 0.4, deviation setting 2500 Hz',ha='center',fontsize=9)
 if handles:fig.legend(handles,[h.get_label() for h in handles],loc='lower center',bbox_to_anchor=(.5,.045),ncol=4,frameon=False)
 fig.text(.5,.015,'Bands: observed min–max, not uncertainty. RF power and RSSI are not calibrated. '+('PARTIAL RUN' if args.partial_log else ''),ha='center',fontsize=8,color='.35')
 fig.subplots_adjust(top=.83,bottom=.24,wspace=.12,left=.075,right=.985)
 name='sinad' if metric=='sinad_db' else 'rssi';fig.savefig(out/(name+'.svg'));fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig)
print('Analyzed',len(rows),'points')
for axis in ['rx','tx']:
 for rate,label in zip(rates,labels):
  group=[r for r in rows if r['axis']==axis and r[axis+'_rate_hz']==rate and r['tx_power_dbm'] is not None]
  if group:print(axis,label,len(group),'points, SINAD range',min(r['sinad_db']['min'] for r in group if r['sinad_db']),max(r['sinad_db']['max'] for r in group if r['sinad_db']),flush=True)
