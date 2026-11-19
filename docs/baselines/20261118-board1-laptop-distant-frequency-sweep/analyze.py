"""Verify snapshots against plan and emit report, CSV and static frequency plots."""
import csv,hashlib,json,math,pathlib,statistics,sys,collections
root=pathlib.Path(sys.argv[1]);dest=pathlib.Path(sys.argv[2]);dest.mkdir(parents=True,exist_ok=True)
selection=json.loads((root/'source-selection.json').read_text()) if (root/'source-selection.json').exists() else {}
plan=json.loads((root/'plan.json').read_text());rows=json.loads((root/'results.json').read_text());byid={r['point']:r for r in rows};raw=collections.defaultdict(list);last_sequences={};last_times={}
for line in (root/'points.jsonl').open():
 x=json.loads(line);point=x['point'];hz=x['frequency_hz'];r=x['rx'];t=x['tx'];raw[point].append(x)
 assert plan['points'][point-1]['frequency_hz']==hz
 assert r['state']=='RX' and t['state']=='TX'
 if selection:
  choice=selection[str(point)];assert r['rx']['gate_open'] is True
  assert r['rx']['noise_enabled'] is choice['noise_squelch_enabled']
  assert r['rx']['carrier_enabled'] is False
 for role,p in [('rx',r),('tx',t)]:
  # Per-point sequences/timestamps; a resumed helper may restart its counter.
  key=(point,role)
  assert p['sequence']>last_sequences.get(key,-1) and p['monotonic_s']>last_times.get(key,-1)
  last_sequences[key]=p['sequence'];last_times[key]=p['monotonic_s']
 assert r['config']['requested_frequency_hz']==hz and t['config']['tx_frequency_hz']==hz
 assert r['config']['sample_rate_hz']==t['config']['tx_sample_rate_hz']==1000000
 assert t['config']['tx_power_dbm']==0 and t['config']['tx_actual_power_dbm']==0
 assert t['config']['tx_source']=='tone' and t['config']['tx_mode']=='nbfm'
 assert t['config']['tx_tone_hz']==1000 and t['config']['tx_tone_amp']==.4 and t['config']['tx_deviation_hz']==2500
 assert t['hardware']['rf24']['registers']['PAC']&31==12
 for role,p,bw in [('rx',r,160000),('tx',t,80000)]:
  obs=p['config']['observed'][role]
  assert obs['sample_rate_hz']==1000000 and obs['analog_bandwidth_hz']==bw and obs['digital_cutoff_hz']==125000
 assert r['config']['rx_deviation_hz']==2500 and not r['config']['am_offset_tuning_enabled']

def get_stats(v):return dict(mean=statistics.mean(v),median=statistics.median(v),min=min(v),max=max(v),stdev=statistics.stdev(v) if len(v)>1 else 0,n=len(v)) if v else None
for row in rows:
 xs=raw[row['point']];assert len(xs)==row['packets'] and len(xs)>=20
 rss=[];sinad=[]
 for x in xs:
  v=x['rx']['hardware']['rf24']['registers']['RSSI']
  if isinstance(v,int) and v!=127:rss.append(v-256 if v>127 else v)
  v=x['rx']['rx']['sinad']['db']
  if isinstance(v,(int,float)):assert math.isfinite(v);sinad.append(v)
 for key,values in [('rssi_dbm',rss),('sinad_db',sinad)]:
  expected=get_stats(values);actual=row[key]
  if expected is None:assert actual is None
  else:
   for k,v in expected.items():assert abs(actual[k]-v)<1e-8,(row['point'],key,k)
 assert row['sinad_valid_fraction']==len(sinad)/len(xs)
 for role in ['rx','tx']:
  for name,e in row[role+'_errors'].items():
   a,b=name.split('.');v=[x[role][role][a][b] for x in xs]
   assert e==dict(first=v[0],last=v[-1],max=max(v),delta=v[-1]-v[0])
 assert row['transport_clean']==(not any(v['max'] for v in [*row['rx_errors'].values(),*row['tx_errors'].values()]))
assert set(raw)==set(byid), 'Measurement is still running or summary has missing points'
failed=json.loads((root/'failures.json').read_text()) if (root/'failures.json').exists() else []
coverage=[]
for b in plan['band_coverage']:
 measured=[r for r in rows if b['low_hz']<=r['frequency_hz']<=b['high_hz']]
 coverage.append(dict(**b,measured_points=len(measured),fully_valid_sinad_points=sum(r['sinad_valid_fraction']==1 for r in measured),transport_clean_points=sum(r['transport_clean'] for r in measured)))
summary=dict(planned_points=len(plan['points']),measured_points=len(rows),failed_points=len(failed),snapshots=sum(map(len,raw.values())),all_points_measured=set(byid)=={p['point'] for p in plan['points']},fully_valid_sinad_points=sum(r['sinad_valid_fraction']==1 for r in rows),partially_valid_sinad_points=sum(0<r['sinad_valid_fraction']<1 for r in rows),unreported_sinad_points=sum(r['sinad_db'] is None for r in rows),points_with_transport_counters=sum(not r['transport_clean'] for r in rows),points_with_new_transport_errors=sum(any(e['delta'] for e in [*r['rx_errors'].values(),*r['tx_errors'].values()]) for r in rows),all_bands_have_three_measured_points=all(b['measured_points']>=3 for b in coverage),statistics_reconciled=True,settings_and_observed_filters_verified=True,band_coverage=coverage,raw_points_sha256=hashlib.sha256((root/'points.jsonl').read_bytes()).hexdigest())
(dest/'verification.json').write_text(json.dumps(summary,indent=2)+'\n')
with (dest/'results.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['point','frequency_hz','frequency_mhz','bands','rssi_mean_dbm','rssi_min_dbm','rssi_max_dbm','sinad_mean_db','sinad_min_db','sinad_max_db','sinad_valid_fraction','sinad_status','snapshots','transport_counters_zero','new_transport_errors_in_window','tx_pa_code','rx_actual_frequency_hz','tx_actual_frequency_hz'])
 for r in rows:
  a=r['rssi_dbm'];b=r['sinad_db'] or {};w.writerow([r['point'],r['frequency_hz'],r['frequency_hz']/1e6,';'.join(r['bands']),a['mean'],a['min'],a['max'],b.get('mean'),b.get('min'),b.get('max'),r['sinad_valid_fraction'],'reported' if r['sinad_valid_fraction']==1 else 'partial' if r['sinad_db'] else 'not_reported_1dB_tone_criterion',r['packets'],r['transport_clean'],sum(e['delta'] for e in [*r['rx_errors'].values(),*r['tx_errors'].values()]),r['tx_pa_code'],r['rx_config']['rx_frequency_hz'],r['tx_config']['tx_actual_frequency_hz']])
if '--no-plots' not in sys.argv:
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.ticker import FixedLocator, FuncFormatter, LogLocator, MultipleLocator, NullFormatter
 for scale in ['linear','log']:
  fig,axes=plt.subplots(2,1,figsize=(14,8),sharex=True,constrained_layout=True)
  for ax,key,label,color in [(axes[0],'rssi_dbm','Raw RSSI (dBm)','#27618c'),(axes[1],'sinad_db','Estimated audio SINAD (dB)','#aa4528')]:
   xs=[r['frequency_hz']/1e6 for r in rows];ys=[r[key]['mean'] if r[key] else float('nan') for r in rows]
   ax.plot(xs,ys,'o-',ms=3,lw=.8,color=color)
   ax.fill_between(xs,[r[key]['min'] if r[key] else float('nan') for r in rows],[r[key]['max'] if r[key] else float('nan') for r in rows],alpha=.15,color=color,label='Observed min–max')
   if key=='sinad_db':
    partial=[r for r in rows if 0<r['sinad_valid_fraction']<1]
    if partial:ax.scatter([r['frequency_hz']/1e6 for r in partial],[r[key]['mean'] for r in partial],marker='D',s=32,facecolors='white',edgecolors=color,label='Partial reporting',zorder=5)
    missing=[r['frequency_hz']/1e6 for r in rows if not r['sinad_db']]
    ax.scatter(missing,[1]*len(missing),marker='v',s=24,color='black',label='Not reported (1 dB tone criterion)',zorder=4)
    ax.axhline(1,color='black',alpha=.3,lw=.6,ls=':')
   ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend(loc='best')
   for lo,hi in [(2385,2495)]:ax.axvspan(lo,hi,color='grey',alpha=.1)
   for x in [2385,2495]:ax.axvline(x,color='grey',lw=.7,ls='--')
  axes[-1].set_xscale(scale);axes[-1].set_xlim(30,6000);axes[-1].set_xlabel('Requested RF frequency (MHz)')
  if scale=='linear':
   axes[-1].xaxis.set_major_locator(MultipleLocator(500))
   axes[-1].xaxis.set_minor_locator(MultipleLocator(100))
  else:
   axes[-1].xaxis.set_major_locator(FixedLocator([30,50,100,200,300,500,1000,2000,3000,4000,6000]))
   axes[-1].xaxis.set_major_formatter(FuncFormatter(lambda value, position: f'{value:g}'))
   axes[-1].xaxis.set_minor_locator(LogLocator(base=10,subs=range(1,10)))
  axes[-1].xaxis.set_minor_formatter(NullFormatter())
  for ax in axes:
   ax.grid(axis='x',which='major',alpha=.3,linewidth=.7)
   ax.grid(axis='x',which='minor',alpha=.13,linewidth=.5)
  fig.suptitle('Board #1 RX / original TX · laptop distant · nominal 90 dB coax\n0 dBm requested TX · both 1 MS/s · 1 kHz NBFM tone · LED1=0\nUncalibrated end-to-end link; shaded region is modem bypass')
  fig.savefig(dest/f'frequency-{scale}.svg');fig.savefig(dest/f'frequency-{scale}.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k!='band_coverage'},indent=2))
