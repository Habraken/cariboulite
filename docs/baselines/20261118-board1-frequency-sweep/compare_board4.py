"""Compare two independently verified frequency sweeps at matched settings."""
import json,csv,math,pathlib,statistics,sys
base=pathlib.Path(sys.argv[1]);reference=pathlib.Path(sys.argv[2]);a=json.loads((base/'summary.json').read_text());b=json.loads((reference/'summary.json').read_text());aa={r['frequency_hz']:r for r in a};bb={r['frequency_hz']:r for r in b}
assert set(aa)==set(bb) and len(aa)==172
assert (base/'plan.json').read_bytes()==(reference/'plan.json').read_bytes(),'plans differ'
old_followup=set(json.loads((reference/'retry-points.json').read_text()));new_followup=set(json.loads((base/'retry-points.json').read_text()));assert old_followup<=new_followup
def normalize_args(args):
 out=list(args);out[out.index('--socket')+1]='<run-socket>';return out
for role in ['rx','tx']:
 p=json.loads((base/f'primary-{role}-ready.json').read_text())['metadata'];q=json.loads((reference/f'primary-{role}-ready.json').read_text())['metadata']
 assert p['binary_sha256']==q['binary_sha256'] and p['library_sha256']==q['library_sha256']
 assert p['host']==q['host'] and p['kernel']==q['kernel']
 assert normalize_args(p['args'])==normalize_args(q['args'])
 if role=='tx':assert p['board_uuid']==q['board_uuid']=='c132d0e7-d7dd-457a-b105-4a5cd97e7e80'
 else:assert p['board_uuid']=='d8810044-c444-46f4-a548-703412cbf971' and q['board_uuid']=='c2440a8d-3852-4c44-b96f-1b64a962e7dc'
rows=[]
for hz in sorted(aa):
 x,y=aa[hz],bb[hz]
 for z in [x,y]:
  assert z['rx_config']['sample_rate_hz']==z['tx_config']['tx_sample_rate_hz']==1000000
  assert z['tx_config']['tx_power_dbm']==0 and z['tx_pa_code']==12
  assert z['tx_config']['tx_tone_hz']==1000 and z['tx_config']['tx_tone_amp']==.4 and z['tx_config']['tx_deviation_hz']==2500
 xs=x['sinad_db']['mean'] if x['sinad_db'] else None;ys=y['sinad_db']['mean'] if y['sinad_db'] else None
 rows.append(dict(frequency_hz=hz,bands=';'.join(x['bands']),board1_rssi_dbm=x['rssi_dbm']['mean'],board4_rssi_dbm=y['rssi_dbm']['mean'],rssi_difference_db=x['rssi_dbm']['mean']-y['rssi_dbm']['mean'],board1_sinad_db=xs,board4_sinad_db=ys,sinad_difference_db=xs-ys if xs is not None and ys is not None else None))
with (base/'comparison-board4.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def stats(v):return dict(n=len(v),mean=statistics.mean(v),median=statistics.median(v),min=min(v),max=max(v)) if v else None
summary=dict(matched_frequencies=len(rows),identical_plan_and_software_verified=True,backend_arguments_match_except_socket=True,matched_board4_followup_points=len(old_followup),additional_board1_followup_points=sorted(new_followup-old_followup),rssi_board1_minus_board4_db=stats([r['rssi_difference_db'] for r in rows]),sinad_board1_minus_board4_db=stats([r['sinad_difference_db'] for r in rows if r['sinad_difference_db'] is not None]),unreported_on_both_hz=[r['frequency_hz'] for r in rows if r['board1_sinad_db'] is None and r['board4_sinad_db'] is None],unreported_only_board1_hz=[r['frequency_hz'] for r in rows if r['board1_sinad_db'] is None and r['board4_sinad_db'] is not None],unreported_only_board4_hz=[r['frequency_hz'] for r in rows if r['board4_sinad_db'] is None and r['board1_sinad_db'] is not None])
zoom=[r for r in rows if 2e9<=r['frequency_hz']<=3e9];summary['two_to_three_ghz']=dict(points=len(zoom),rssi_difference_db=stats([r['rssi_difference_db'] for r in zoom]),sinad_difference_db=stats([r['sinad_difference_db'] for r in zoom if r['sinad_difference_db'] is not None]),unreported_board1=sum(r['board1_sinad_db'] is None for r in zoom),unreported_board4=sum(r['board4_sinad_db'] is None for r in zoom))
(base/'comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if '--no-plots' not in sys.argv:
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.ticker import FixedLocator,FuncFormatter,LogLocator,MultipleLocator,NullFormatter
 for view in ['linear','log','2-to-3GHz']:
  fig,axes=plt.subplots(2,1,figsize=(14,8),sharex=True,constrained_layout=True)
  for data,label,color in [(b,'Board #4','#326aab'),(a,'Board #1','#20845d')]:
   for ax,key,ylabel in [(axes[0],'rssi_dbm','Raw RSSI (dBm)'),(axes[1],'sinad_db','Estimated audio SINAD (dB)')]:
    xs=[r['frequency_hz']/1e6 for r in data];ys=[r[key]['mean'] if r[key] else math.nan for r in data]
    ax.plot(xs,ys,'o-',ms=3,lw=1,label=label,color=color)
    if key=='sinad_db':
     missing=[r['frequency_hz']/1e6 for r in data if not r[key]]
     ax.scatter(missing,[1]*len(missing),marker='v',s=35 if label=='Board #4' else 12,facecolors='none' if label=='Board #4' else color,edgecolors=color,label=label+' unreported',zorder=4)
    ax.set_ylabel(ylabel)
  for ax in axes:
   ax.axvspan(2385,2495,color='grey',alpha=.1)
   for hz in [2385,2495]:ax.axvline(hz,color='grey',ls='--',lw=.7)
   ax.grid(axis='y',alpha=.25);ax.legend(loc='best',ncol=2)
  axes[-1].set_xlabel('Requested RF frequency (MHz)')
  if view=='log':
   axes[-1].set_xscale('log');axes[-1].set_xlim(30,6000)
   axes[-1].xaxis.set_major_locator(FixedLocator([30,50,100,200,300,500,1000,2000,3000,4000,6000]));axes[-1].xaxis.set_major_formatter(FuncFormatter(lambda x,pos:f'{x:g}'));axes[-1].xaxis.set_minor_locator(LogLocator(base=10,subs=range(1,10)))
  else:
   axes[-1].set_xlim((2000,3000) if view=='2-to-3GHz' else (30,6000))
   axes[-1].xaxis.set_major_locator(MultipleLocator(100 if view=='2-to-3GHz' else 500));axes[-1].xaxis.set_minor_locator(MultipleLocator(20 if view=='2-to-3GHz' else 100))
  axes[-1].xaxis.set_minor_formatter(NullFormatter())
  for ax in axes:
   ax.grid(axis='x',which='major',alpha=.3,lw=.7);ax.grid(axis='x',which='minor',alpha=.13,lw=.5)
  fig.suptitle('Receiver comparison: boards #1 and #4 on pi4f · same original pi4g TX\n0 dBm requested · both 1 MS/s · nominal 90 dB coax · LED1=0\nUncalibrated link; triangles denote unreported SINAD, not measured 1 dB')
  fig.savefig(base/f'comparison-{view}.png',dpi=160);fig.savefig(base/f'comparison-{view}.svg');plt.close(fig)
print(json.dumps(summary,indent=2))
