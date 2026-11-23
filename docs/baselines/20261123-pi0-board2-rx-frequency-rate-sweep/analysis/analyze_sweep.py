"""Reproducible analysis of the completed, locally archived Pi Zero sweep."""
import collections,csv,gzip,hashlib,json,math,pathlib,statistics,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np
BASE=pathlib.Path(__file__).resolve().parent
ROOT=pathlib.Path(sys.argv[1]) if len(sys.argv)>1 else BASE.parent
DEST=ROOT/'analysis';DEST.mkdir(exist_ok=True)
rows=[json.loads(line) for line in (ROOT/'results.jsonl').read_text().splitlines()]
rs=[r for r in rows if r['phase']=='matrix'];pre=[r for r in rows if r['phase']=='preflight']
RATES=sorted({r['sample_rate_hz'] for r in rs},reverse=True);FREQS=sorted({r['frequency_hz'] for r in rs});lookup={(r['frequency_hz'],r['sample_rate_hz']):r for r in rs}
assert len(rs)==1376 and len(FREQS)==172 and len(RATES)==8 and len(lookup)==len(rs)
assert all((f,s) in lookup for f in FREQS for s in RATES)
assert all(r['tx_power_dbm']==0 for r in rs)
assert json.loads((ROOT/'matrix-verification.json').read_text())['passed']
verification=json.loads((ROOT/'verification.json').read_text());assert verification['passed']
labels={4000000:'4',2000000:'2',1333333:'1.333333',1000000:'1',800000:'0.8',666667:'0.666667',500000:'0.5',400000:'0.4'}
colors=plt.get_cmap('tab10').colors
plt.rcParams.update({'font.size':10,'axes.titlesize':12,'axes.labelsize':11,'figure.facecolor':'white','axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})
def vals(rate,key):return np.array([lookup[(f,rate)][key]['mean'] if lookup[(f,rate)][key] else np.nan for f in FREQS])
def save(fig,name):
    fig.savefig(DEST/(name+'.png'),dpi=180,bbox_inches='tight');fig.savefig(DEST/(name+'.svg'),bbox_inches='tight');plt.close(fig)
    svg=DEST/(name+'.svg');svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
def decorate(ax,ylabel):ax.set_ylabel(ylabel);ax.grid(alpha=.2);ax.set_axisbelow(True)
# Overview: gaps remain gaps where the estimator reported no number.
fig,axs=plt.subplots(2,1,figsize=(13.5,8),sharex=True,layout='constrained')
for i,rate in enumerate(RATES):
 for ax,key in zip(axs,['rssi_dbm','sinad_db']):ax.plot(np.array(FREQS)/1e9,vals(rate,key),'.-',lw=.85,ms=2.5,color=colors[i],alpha=.85,label=labels[rate])
for ax,key in zip(axs,['rssi_dbm','sinad_db']):
 flagged=[r for r in rs if not r['transport_clean'] and r[key]]
 if flagged:ax.scatter([r['frequency_hz']/1e9 for r in flagged],[r[key]['mean'] for r in flagged],marker='x',s=45,c='black',lw=1.5,zorder=5)
for ax in axs:
 for boundary in [2.385,2.495]:ax.axvline(boundary,color='#777777',ls=':',lw=.8)
 ax.set_xlim(.03,6)
decorate(axs[0],'Mean RSSI (dBm)');decorate(axs[1],'Mean estimated SINAD (dB)')
axs[0].set_title('Conducted NBFM link response — 0 dBm requested TX, nominal 90 dB attenuation')
axs[0].legend(title='Matched TX/RX rate (MS/s)',ncol=4,loc='upper left',fontsize=9,title_fontsize=9)
axs[1].set_xlabel('Requested frequency (GHz)');axs[1].set_xticks(np.arange(0,6.1,.5))
axs[1].set_ylim(bottom=0)
save(fig,'frequency-overview')
# Heatmaps: frequency cells use midpoint edges on the actual nonuniform grid.
f=np.array(FREQS)/1e9;edges=np.r_[f[0],(f[:-1]+f[1:])/2,f[-1]]
fig,axs=plt.subplots(2,1,figsize=(13.5,8),sharex=True,layout='constrained')
for ax,key,cmap_name,title in zip(axs,['rssi_dbm','sinad_db'],['viridis','magma'],['RSSI (dBm)','Estimated SINAD (dB)']):
 data=np.vstack([vals(rate,key) for rate in RATES]);cmap=plt.get_cmap(cmap_name).copy();cmap.set_bad('#c7cbd1')
 im=ax.pcolormesh(edges,np.arange(9),np.ma.masked_invalid(data),cmap=cmap,shading='flat',vmin=np.floor(np.nanmin(data)) if key=='rssi_dbm' else 0,vmax=np.ceil(np.nanmax(data)),rasterized=True)
 ax.set_yticks(np.arange(8)+.5,[labels[r] for r in RATES]);ax.invert_yaxis();ax.set_ylabel('Matched rate (MS/s)');ax.set_title(title);fig.colorbar(im,ax=ax,label=title,pad=.015)
axs[0].set_title('RSSI (dBm) — actual frequency spacing');axs[1].set_title('Estimated SINAD (dB) — gray means unavailable')
axs[1].set_xlabel('Requested frequency (GHz)');axs[1].set_xticks(np.arange(0,6.1,.5));save(fig,'frequency-rate-heatmaps')
# Rate profiles at selected anchors. Fractional rate labels are rounded command values.
anchors=[30000000,430100000,1350000000,2399900000,2494900000,5999999000]
asc=sorted(RATES);fig,axs=plt.subplots(2,1,figsize=(12.5,8),sharex=True,layout='constrained')
for i,hz in enumerate(anchors):
 for ax,key in zip(axs,['rssi_dbm','sinad_db']):
  y=[lookup[(hz,rate)][key]['mean'] if lookup[(hz,rate)][key] else np.nan for rate in asc]
  ax.plot(np.array(asc)/1e6,y,'o-',ms=4,lw=1.2,color=colors[i],label=f'{hz/1e6:.3f}'.rstrip('0').rstrip('.')+' MHz')
for ax in axs:ax.set_xscale('log');ax.grid(alpha=.2)
axs[0].set_title('Sample-rate dependence at selected measured frequencies');axs[0].legend(ncol=3,fontsize=9)
axs[0].set_ylabel('Mean RSSI (dBm)');axs[1].set_ylabel('Mean estimated SINAD (dB)');axs[1].set_ylim(bottom=0)
axs[1].set_xticks(np.array(asc)/1e6,[labels[r] for r in asc]);axs[1].minorticks_off();axs[1].set_xlabel('Matched TX/RX sample rate (MS/s; logarithmic axis)');save(fig,'rate-profiles')
# Native modem path boundaries, where 100 kHz spacing is otherwise invisible.
fig,axs=plt.subplots(2,2,figsize=(13.5,8),layout='constrained')
for col,(lo,hi,bound) in enumerate([(2384.75,2385.25,2385),(2494.75,2495.25,2495)]):
 for i,rate in enumerate(RATES):
  for row,key in enumerate(['rssi_dbm','sinad_db']):
   ff=[h for h in FREQS if lo*1e6<=h<=hi*1e6];yy=[lookup[(h,rate)][key]['mean'] if lookup[(h,rate)][key] else np.nan for h in ff]
   axs[row,col].plot(np.array(ff)/1e6,yy,'o-',lw=1,ms=3,color=colors[i],label=labels[rate])
 for row in range(2):axs[row,col].axvline(bound,color='#666',ls=':');axs[row,col].grid(alpha=.2);axs[row,col].set_xlim(lo,hi)
 axs[0,col].set_title(f'{bound} MHz path boundary');axs[1,col].set_xlabel('Requested frequency (MHz)')
axs[0,0].set_ylabel('Mean RSSI (dBm)');axs[1,0].set_ylabel('Mean estimated SINAD (dB)')
handles,legend_labels=axs[0,0].get_legend_handles_labels()
fig.legend(handles,legend_labels,title='Matched rate (MS/s)',ncol=8,fontsize=9,title_fontsize=9,loc='upper center',bbox_to_anchor=(.5,1.08));save(fig,'path-boundary-zooms')
# Summaries and paired comparisons exclude partially valid windows when comparing SINAD.
summary=[]
for rate in RATES:
 rr=[r for r in rs if r['sample_rate_hz']==rate];sn=[r['sinad_db']['mean'] for r in rr if r['sinad_db']]
 pairs=[(r,lookup[(r['frequency_hz'],1000000)]) for r in rr]
 ds=[r['sinad_db']['mean']-ref['sinad_db']['mean'] for r,ref in pairs if r['sinad_valid_fraction']==ref['sinad_valid_fraction']==1 and r['transport_clean'] and ref['transport_clean']]
 summary.append(dict(rate_hz=rate,points=len(rr),transport_flagged=sum(not r['transport_clean'] for r in rr),numeric_sinad=len(sn),full_sinad=sum(r['sinad_valid_fraction']==1 for r in rr),median_rssi_dbm=statistics.median(r['rssi_dbm']['mean'] for r in rr),median_sinad_db=statistics.median(sn),sinad_ge_12_db=sum(s>=12 for s in sn),paired_full_points=len(ds),median_sinad_difference_vs_1m_db=statistics.median(ds),rssi_min_dbm=min(r['rssi_dbm']['mean'] for r in rr),rssi_max_dbm=max(r['rssi_dbm']['mean'] for r in rr)))
with (DEST/'rate-summary.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(summary[0]),lineterminator='\n');w.writeheader();w.writerows(summary)
common=[h for h in FREQS if all(lookup[(h,r)]['sinad_valid_fraction']==1 and lookup[(h,r)]['transport_clean'] for r in RATES)]
spreads=[max(lookup[(h,r)]['sinad_db']['mean'] for r in RATES)-min(lookup[(h,r)]['sinad_db']['mean'] for r in RATES) for h in common]
missing=[(h,[r for r in RATES if lookup[(h,r)]['sinad_db'] is None]) for h in FREQS];missing=[x for x in missing if x[1]]
partial=[r for r in rs if 0<r['sinad_valid_fraction']<1]
# Scan complete host histories to separate out-of-window events from measured windows.
history={}
import tarfile
for role in ('rx','tx'):
 keys=[('alignment','invalid_frames')] if role=='rx' else [('driver','underrun_periods')]
 peaks={a+'.'+b:0 for a,b in keys};increases={a+'.'+b:0 for a,b in keys};previous={};latest=None
 with tarfile.open(ROOT/(role+'-host-artifacts.tar.gz')) as archive:
  for mem in archive.getmembers():
   if mem.name.endswith('/status.jsonl.gz'):
    previous={}
    with gzip.GzipFile(fileobj=archive.extractfile(mem)) as stream:
     for line in stream:
      d=json.loads(line)
      if latest is None or d['monotonic_s']>latest['monotonic_s']:
       latest=dict(monotonic_s=d['monotonic_s'],state=d['state'],tx_running=d['tx']['running'])
      for a,b in keys:
       k=a+'.'+b;v=d.get(role,{}).get(a,{}).get(b)
       if v is None:continue
       peaks[k]=max(peaks[k],v)
       if k in previous and v>previous[k]:increases[k]+=v-previous[k]
       previous[k]=v
 history[role]={'peak':peaks,'positive_increments':increases,'final_snapshot':latest}
 assert latest['state']=='IDLE' and not latest['tx_running'], (role,latest)
metrics=dict(transport_flagged=sum(not r['transport_clean'] for r in rs),matrix_points=len(rs),matrix_snapshots=sum(r['snapshots'] for r in rs),numeric_sinad=sum(r['sinad_db'] is not None for r in rs),full_sinad=sum(r['sinad_valid_fraction']==1 for r in rs),common_full_frequencies=len(common),median_sinad_rate_span_db=statistics.median(spreads),max_sinad_rate_span_db=max(spreads),history=history,rate_summary=summary)
(DEST/'analysis-summary.json').write_text(json.dumps(metrics,indent=2)+'\n')
widest_rssi_hz=max(FREQS,key=lambda hz:max(lookup[(hz,r)]['rssi_dbm']['mean'] for r in RATES)-min(lookup[(hz,r)]['rssi_dbm']['mean'] for r in RATES))
widest_rssi_values=[lookup[(widest_rssi_hz,r)]['rssi_dbm']['mean'] for r in RATES]
widest_rssi_sinad=[lookup[(widest_rssi_hz,r)]['sinad_db']['mean'] for r in RATES if lookup[(widest_rssi_hz,r)]['sinad_db']]
metrics['largest_rssi_rate_span']=dict(frequency_hz=widest_rssi_hz,min_dbm=min(widest_rssi_values),max_dbm=max(widest_rssi_values),span_db=max(widest_rssi_values)-min(widest_rssi_values),numeric_sinad_min_db=min(widest_rssi_sinad) if widest_rssi_sinad else None,numeric_sinad_max_db=max(widest_rssi_sinad) if widest_rssi_sinad else None)
(DEST/'analysis-summary.json').write_text(json.dumps(metrics,indent=2)+'\n')
from datetime import datetime
from zoneinfo import ZoneInfo
progress=json.loads((ROOT/'progress.json').read_text())
plan=json.loads((ROOT/'plan.json').read_text())
matrix_verification=json.loads((ROOT/'matrix-verification.json').read_text())
start=datetime.strptime(plan['run'].split('rx-')[1],'%Y%m%dT%H%M%SZ').replace(tzinfo=ZoneInfo('UTC'))
finish=datetime.fromisoformat(progress['updated_utc'].replace('Z','+00:00'))
local=ZoneInfo('Europe/Brussels')
fmt=lambda x:'Unavailable' if x is None else f'{x:.2f}'
best=max((r for r in rs if r['sinad_db'] and r['transport_clean']),key=lambda r:r['sinad_db']['mean'])
numeric=metrics['numeric_sinad']
lines=[
 '# Repaired CaribouLite device #2 RX frequency and sample-rate sweep',
 '', f'**{start.astimezone(local):%d %B %Y · %H:%M}–{finish.astimezone(local):%H:%M} Europe/Brussels · pi0a TX → pi0b RX**',
 '', '## Results',
 '', f'All **{len(rs):,} planned measurements** completed: **172 frequencies × eight matched TX/RX sample rates**, at requested and hardware-reported **0 dBm TX** through the user-confirmed nominal **90 dB attenuated RF24/HiF link**. Independent verification passed against **{verification["raw_paired_snapshots"]:,} paired snapshots**, including eight preflight measurements. The main matrix contains **{metrics["matrix_snapshots"]:,} pairs**.',
 '', f'Numeric SINAD is available at **{numeric:,}/{len(rs):,} points**; **{len(rs)-numeric} points have no numeric estimate**, and **{len(partial)} additional windows are partially valid**. Missing SINAD remains blank in the CSV. **{metrics["transport_flagged"]} matrix windows** showed increases or resets in the selected transport counters.',
 '', f'The best transport-clean point mean is **{best["sinad_db"]["mean"]:.2f} dB SINAD at {best["frequency_hz"]/1e6:g} MHz and {labels[best["sample_rate_hz"]]} MS/s**. This describes the complete link and the backend estimator; it does not establish calibrated receiver sensitivity.',
 '', '## Setup',
 '', '| Item | Setting |', '|---|---|',
 '| TX | Original reference board on `pi0a-trixie`, `192.168.6.37` |',
 '| TX board UUID | `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` |',
 '| RX | Repaired device #2 on `pi0b-trixie`, `192.168.6.69`; RX only |',
 '| RX board UUID | `36b67ac4-b313-4369-99bd-4b3e09caabfa` |',
 '| RX oscillator | Correct 125 MHz oscillator fitted, as reported by the operator; no oscillator timing workaround |',
 '| RF path | HiF/RF24 ports, nominal 90 dB attenuated conducted link |',
 '| Frequency grid | 172 frequencies, 30 MHz–5,999.999 MHz; 50 MHz grid plus band anchors and path boundaries |',
 '| Rates | 4, 2, 1.333333, 1, 0.8, 0.666667, 0.5, 0.4 MS/s; TX and RX changed together |',
 '| TX signal | NBFM; 1 kHz sine, amplitude 0.4; deviation setting 2,500 Hz |',
 '| Power | Requested and reported 0 dBm at every retained sample; no independent RF power calibration |',
 '| Signalling | TX/RX CTCSS and CDCSS off |',
 '| RX gain profile | `matched`, with `am=-7`, `usb=0`, `lsb=0` |',
 '| RX controls | Carrier/noise squelch disabled; LED1 held low, original LED state restored on release |',
 '| RX playback | Same Logitech H650e USB headset moved from pi0a to pi0b by the operator |',
 '| Reference clock drive | Standard 8 mA setting, no drive override or preload |',
 '| Acquisition | 2 s settling, 5 s measurement per point; fresh sequences and monotonic timestamps on each host |',
 '| SINAD | Backend NBFM 1 kHz estimate over 300–3,000 Hz |',
 '', 'Exactly 6 GHz is excluded by the radio API, so the highest setting is 5,999,999,000 Hz. The fractional native rates are 4 MHz divided by 3 and 6, with rounded integer command labels. RX analog bandwidth remains 160 kHz and TX analog bandwidth 80 kHz; digital cutoff changes with rate from 500 kHz to 50 kHz. Thus each sample-rate setting changes a complete profile on both boards.',
 '', 'The oscillator replacement was reported by the operator. This sweep checks functional operation with the standard driver, including 4 MS/s RX; it does not independently measure oscillator frequency or phase noise. The microphone compressor is enabled in configuration but is microphone-only, so it does not process this synthetic tone source.',
 '', '## Frequency response',
 '', '![RSSI and SINAD versus frequency](analysis/frequency-overview.png)',
 '', '[Vector figure](analysis/frequency-overview.svg). Points are discrete measured settings. Connecting lines guide the eye; missing SINAD interrupts the curves. Dotted lines mark the 2,385 and 2,495 MHz RF path boundaries. Black crosses identify transport-flagged windows.',
 '', 'Representative means at 1 MS/s:',
 '', '| Frequency MHz | RSSI dBm | Estimated SINAD dB |', '|---:|---:|---:|',
]
for hz in [30000000,430100000,1000000000,1350000000,1750000000,2000000000,2384900000,2385000000,2399900000,2400000000,2494900000,2495000000,2650000000,4650000000,5999999000]:
 r=lookup[(hz,1000000)]
 lines.append(f'| {hz/1e6:.3f} | {r["rssi_dbm"]["mean"]:.2f} | {fmt(r["sinad_db"]["mean"] if r["sinad_db"] else None)} |')
lines += [
 '', '## Frequency and rate maps',
 '', '![Frequency and rate heatmaps](analysis/frequency-rate-heatmaps.png)',
 '', '[Vector figure](analysis/frequency-rate-heatmaps.svg). Frequency cells follow the actual nonuniform grid, with edges at neighboring midpoints. Rates are discrete rows. Gray SINAD cells indicate missing estimates; colors are not interpolated.',
 '', '## Sample-rate summary',
 '', '| Rate MS/s | Numeric SINAD / 172 | Transport flags | Median RSSI dBm | Median numeric SINAD dB | Points ≥12 dB | Median paired SINAD Δ vs 1 MS/s dB |',
 '|---:|---:|---:|---:|---:|---:|---:|',
]
for s in summary:
 lines.append(f'| {labels[s["rate_hz"]]} | {s["numeric_sinad"]} | {s["transport_flagged"]} | {s["median_rssi_dbm"]:.2f} | {s["median_sinad_db"]:.2f} | {s["sinad_ge_12_db"]} | {s["median_sinad_difference_vs_1m_db"]:+.2f} |')
lines += [
 '', f'Among **{len(common)} frequencies** with fully valid SINAD and clean transport at every rate, the median range between rate-specific means is **{statistics.median(spreads):.2f} dB**, with a maximum of **{max(spreads):.2f} dB**. The sweep measured one full frequency pass per rate, sequentially; time and temperature are therefore not separated from rate effects.',
 '', 'Medians weight every measured frequency equally, including clustered anchors. Numeric-SINAD medians exclude missing estimates. Paired differences retain only fully valid, transport-clean windows at both rates; counts are in [rate-summary.csv](analysis/rate-summary.csv). The ≥12 dB count is descriptive, not a sensitivity specification.',
 '', '![Rate profiles at selected frequencies](analysis/rate-profiles.png)',
 '', '[Vector figure](analysis/rate-profiles.svg). TX and RX rates change together; these profiles cannot assign a change to one board.',
 '', f'The largest rate-dependent RSSI range is **{max(widest_rssi_values)-min(widest_rssi_values):.2f} dB at {widest_rssi_hz/1e6:g} MHz**, spanning **{min(widest_rssi_values):.2f} to {max(widest_rssi_values):.2f} dBm**. '+(f'Numeric SINAD means at this frequency span **{min(widest_rssi_sinad):.2f}–{max(widest_rssi_sinad):.2f} dB**.' if widest_rssi_sinad else 'No rate yields a numeric SINAD estimate at this frequency.')+' A change in reported RSSI alone does not establish an equal change in wanted-signal input power.',
 '', '## RF path boundaries',
 '', '![Path boundary detail](analysis/path-boundary-zooms.png)',
 '', '[Vector figure](analysis/path-boundary-zooms.svg). These zooms show the closely spaced measurements around the native-modem/mixer boundaries. Any coincident step remains a combined TX/RX link observation.',
 '', '## Missing and partially valid SINAD',
 '', 'An unavailable estimate is not zero SINAD and does not establish that no RF signal was received. RSSI can include interference and conversion artifacts. Only finite numeric estimates contribute to the reported SINAD statistics.',
]
if missing:
 lines += ['', '| Frequency MHz | Rates with no numeric SINAD, MS/s |', '|---:|---|']
 for h,rates in missing:
  lines.append(f'| {h/1e6:g} | '+('All eight' if len(rates)==8 else ', '.join(labels[r] for r in rates))+' |')
else:
 lines += ['', 'Every matrix point yielded at least one numeric SINAD estimate.']
if partial:
 lines += ['', '| Frequency MHz | Rate MS/s | Numeric snapshots / total |', '|---:|---:|---:|']
 for r in partial:
  lines.append(f'| {r["frequency_hz"]/1e6:g} | {labels[r["sample_rate_hz"]]} | {r["sinad_db"]["n"]} / {r["snapshots"]} |')
lines += [
 '', '## Acquisition integrity',
 '', f'The collector recorded **{len(progress["failures"])} failed setup attempts**. A failed setup triggers a clean helper restart and one retry; failed partial windows are not retained as completed points. The matrix verifier checks every planned point is present exactly once.',
 '', f'**{metrics["transport_flagged"]} matrix windows** have transport flags. Flags are recomputed from every retained raw counter value, including checks for resets. They cover RX queue/audio drops, playback underruns/write errors, reader errors, alignment failures/invalid frames, and TX driver underruns, writer errors and queue drops.',
 '', 'Whole-session histories include startup, tuning and settling, so their cumulative counters can differ from the clean-window result:',
 '', f'- RX `alignment.invalid_frames`: peak **{history["rx"]["peak"]["alignment.invalid_frames"]}**, total positive increments **{history["rx"]["positive_increments"]["alignment.invalid_frames"]}**.',
 f'- TX `driver.underrun_periods`: peak **{history["tx"]["peak"]["driver.underrun_periods"]}**, total positive increments **{history["tx"]["positive_increments"]["driver.underrun_periods"]}**.',
 '', 'Independent checks verify host/board identities, RX-only operation, requested frequency/rate, requested and reported power, signalling off, tone settings, fresh sample sequences, RSSI/SINAD summary statistics, CSV consistency, and the complete 172 × 8 matrix. The receiver command audit found **zero TX commands**, and both backend release checks passed. Details are in [verification.json](verification.json) and [matrix-verification.json](matrix-verification.json).',
 '', 'All eight 430.1 MHz preflight measurements passed:',
 '', '| Rate MS/s | Mean RSSI dBm | Mean SINAD dB |', '|---:|---:|---:|',
]
for r in pre:
 lines.append(f'| {labels[r["sample_rate_hz"]]} | {r["rssi_dbm"]["mean"]:.2f} | {r["sinad_db"]["mean"]:.2f} |')
flagged=[r for r in rs if not r['transport_clean']]
if flagged:
 lines += ['', 'Transport-flagged measurement windows:', '', '| Point | Frequency MHz | Rate MS/s | Counter changes |', '|---:|---:|---:|---|']
 for r in flagged:
  changes=[]
  for role in ('rx','tx'):
   for key,value in r[role+'_errors'].items():
    if value['delta'] or value['reset']:
     changes.append(f'{role}.{key}: Δ={value["delta"]}, reset={value["reset"]}')
  lines.append(f'| {r["point"]} | {r["frequency_hz"]/1e6:g} | {labels[r["sample_rate_hz"]]} | '+('; '.join(changes))+' |')
if progress['failures']:
 lines += ['', 'Recovered setup attempts:', '', '| Matrix point | Frequency MHz | Rate MS/s | Failed attempt |', '|---:|---:|---:|---:|']
 for failure in progress['failures']:
  lines.append(f'| {failure["point"]} | {failure["frequency_hz"]/1e6:g} | {labels[failure["sample_rate_hz"]]} | {failure["attempt"]} |')
 lines += ['', 'Exact fault messages are retained in [progress.json](progress.json) and [controller.log](controller.log), with each backend log in the host archives. Retried points contribute only their successful complete measurement window. A recovered startup failure remains an acquisition issue even when that window is clean; it is not classified as a measured SINAD or transport failure.']
lines += [
 '', '## Interpretation limits',
 '', 'These values characterize the complete transmitter–attenuator–receiver link. Requested power and nominal attenuation do not establish actual receiver input power over frequency. This run therefore does not measure receiver sensitivity, noise figure, or calibrated standalone TX/RX frequency response.',
 '', 'This run reverses today’s earlier pairing of repaired device #2 TX on pi0b and original RX on pi0a, using the same boards, software, headset and nominal RF link. A difference between directions cannot by itself separate transmitter purity, actual RF output power, receiver behavior or external interference. The 21 November board #1 RX baseline used the same original TX board but a different receiver and software version; it is not an isolated oscillator-repair comparison.',
 '', '## Data and regeneration',
 '', '- [Point measurements CSV](results.csv), [full point summaries](results.jsonl), and [test plan](plan.json).',
 '- [Acquisition report](REPORT.txt), [host/snapshot verification](verification.json), and [matrix verification](matrix-verification.json).',
 '- [Paired raw snapshots](paired-snapshots.jsonl.gz), [TX host archive](tx-host-artifacts.tar.gz), and [RX host archive](rx-host-artifacts.tar.gz).',
 '- [Analysis summary](analysis/analysis-summary.json), [rate summary](analysis/rate-summary.csv), and [analysis source](analysis/analyze_sweep.py).',
 '', 'With Python, NumPy and Matplotlib installed, run from the baseline directory:',
 '', '```sh', 'python3 verify_matrix.py .', 'MPLCONFIGDIR=/tmp/cariboulite-matplotlib python3 analysis/analyze_sweep.py .', '```',
 '', 'The acquisition sources preserve original deployment addresses and key paths as historical records. Review them before reuse; running acquisition code transmits RF. Offline verification and analysis do not access hardware.',
 '', 'The source manifest preserves the original completion checksums. The package manifest covers the archived evidence, and the analysis manifest covers the generated report and figures.', '',
]
(ROOT/'REPORT.md').write_text('\n'.join(lines))
(DEST/'analyze_sweep.py').write_bytes(pathlib.Path(__file__).read_bytes())
(DEST/'environment.txt').write_text(sys.version+'\nmatplotlib '+matplotlib.__version__+'\nnumpy '+np.__version__+'\n')
(DEST/'requirements.txt').write_text(f'matplotlib=={matplotlib.__version__}\nnumpy=={np.__version__}\n')
(DEST/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(DEST.iterdir()) if p.is_file() and p.name!='SHA256SUMS')+hashlib.sha256((ROOT/'REPORT.md').read_bytes()).hexdigest()+'  ../REPORT.md\n')
print(json.dumps(metrics,indent=2));print('REPORT',ROOT/'REPORT.md')
