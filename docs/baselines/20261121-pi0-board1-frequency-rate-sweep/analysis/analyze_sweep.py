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
assert all(r['tx_power_dbm']==0 and r['transport_clean'] for r in rs)
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
for ax in axs:
 for boundary in [2.385,2.495]:ax.axvline(boundary,color='#777777',ls=':',lw=.8)
 ax.set_xlim(.03,6)
decorate(axs[0],'Mean RSSI (dBm)');decorate(axs[1],'Mean estimated SINAD (dB)')
axs[0].set_title('Conducted NBFM link response — 0 dBm requested TX, nominal 90 dB attenuation')
axs[0].legend(title='Matched TX/RX rate (MS/s)',ncol=4,loc='upper left',fontsize=9,title_fontsize=9)
axs[1].set_xlabel('Requested frequency (GHz)');axs[1].set_xticks(np.arange(0,6.1,.5))
axs[1].set_ylim(0,25);axs[0].annotate('2.65 GHz: strong RSSI, no numeric SINAD',xy=(2.65,-58.2),xytext=(3.1,-66),arrowprops={'arrowstyle':'->','color':'#555'},fontsize=9)
save(fig,'frequency-overview')
# Heatmaps: frequency cells use midpoint edges on the actual nonuniform grid.
f=np.array(FREQS)/1e9;edges=np.r_[f[0],(f[:-1]+f[1:])/2,f[-1]]
fig,axs=plt.subplots(2,1,figsize=(13.5,8),sharex=True,layout='constrained')
for ax,key,cmap_name,title in zip(axs,['rssi_dbm','sinad_db'],['viridis','magma'],['RSSI (dBm)','Estimated SINAD (dB)']):
 data=np.vstack([vals(rate,key) for rate in RATES]);cmap=plt.get_cmap(cmap_name).copy();cmap.set_bad('#c7cbd1')
 im=ax.pcolormesh(edges,np.arange(9),np.ma.masked_invalid(data),cmap=cmap,shading='flat',vmin=-115 if key=='rssi_dbm' else 0,vmax=-58 if key=='rssi_dbm' else 24,rasterized=True)
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
axs[0].set_ylabel('Mean RSSI (dBm)');axs[1].set_ylabel('Mean estimated SINAD (dB)');axs[1].set_ylim(0,25)
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
 ds=[r['sinad_db']['mean']-ref['sinad_db']['mean'] for r,ref in pairs if r['sinad_valid_fraction']==ref['sinad_valid_fraction']==1]
 summary.append(dict(rate_hz=rate,points=len(rr),numeric_sinad=len(sn),full_sinad=sum(r['sinad_valid_fraction']==1 for r in rr),median_rssi_dbm=statistics.median(r['rssi_dbm']['mean'] for r in rr),median_sinad_db=statistics.median(sn),sinad_ge_12_db=sum(s>=12 for s in sn),paired_full_points=len(ds),median_sinad_difference_vs_1m_db=statistics.median(ds),rssi_min_dbm=min(r['rssi_dbm']['mean'] for r in rr),rssi_max_dbm=max(r['rssi_dbm']['mean'] for r in rr)))
with (DEST/'rate-summary.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(summary[0]),lineterminator='\n');w.writeheader();w.writerows(summary)
common=[h for h in FREQS if all(lookup[(h,r)]['sinad_valid_fraction']==1 for r in RATES)]
spreads=[max(lookup[(h,r)]['sinad_db']['mean'] for r in RATES)-min(lookup[(h,r)]['sinad_db']['mean'] for r in RATES) for h in common]
missing=[(h,[r for r in RATES if lookup[(h,r)]['sinad_db'] is None]) for h in FREQS];missing=[x for x in missing if x[1]]
partial=[r for r in rs if 0<r['sinad_valid_fraction']<1]
# Scan complete host histories to separate out-of-window events from measured windows.
history={}
import tarfile
for role in ('rx','tx'):
 keys=[('alignment','invalid_frames')] if role=='rx' else [('driver','underrun_periods')]
 peaks={a+'.'+b:0 for a,b in keys};increases={a+'.'+b:0 for a,b in keys};previous={}
 with tarfile.open(ROOT/(role+'-host-artifacts.tar.gz')) as archive:
  for mem in archive.getmembers():
   if mem.name.endswith('/status.jsonl.gz'):
    with gzip.GzipFile(fileobj=archive.extractfile(mem)) as stream:
     for line in stream:
      d=json.loads(line)
      for a,b in keys:
       k=a+'.'+b;v=d.get(role,{}).get(a,{}).get(b)
       if v is None:continue
       peaks[k]=max(peaks[k],v)
       if k in previous and v>previous[k]:increases[k]+=v-previous[k]
       previous[k]=v
 history[role]={'peak':peaks,'positive_increments':increases}
metrics=dict(matrix_points=len(rs),matrix_snapshots=sum(r['snapshots'] for r in rs),numeric_sinad=sum(r['sinad_db'] is not None for r in rs),full_sinad=sum(r['sinad_valid_fraction']==1 for r in rs),common_full_frequencies=len(common),median_sinad_rate_span_db=statistics.median(spreads),max_sinad_rate_span_db=max(spreads),history=history,rate_summary=summary)
(DEST/'analysis-summary.json').write_text(json.dumps(metrics,indent=2)+'\n')
# Markdown intentionally retained in the ignored run directory with all figures and evidence.
lines=['# Pi Zero / CaribouLite board #1 frequency and sample-rate sweep',
'', '**21 November 2026 · 15:16–18:48 Europe/Brussels · 0 dBm requested TX · nominal 90 dB link attenuation**',
'', '## Findings',
'', f'All **1,376 planned measurements** completed: 172 frequencies at eight matched TX/RX sample rates. Independent verification passed against 32,400 paired snapshots, including eight preflight measurements; the main matrix contains **{metrics["matrix_snapshots"]:,} paired snapshots**. Both backends stopped at completion, and the receiver command audit found no TX commands.',
'', f'Frequency dominates the measured response. Among **{len(common)} frequencies** with fully valid SINAD at every rate, the median difference between the highest and lowest rate-specific SINAD is **{statistics.median(spreads):.2f} dB**; the largest is **{max(spreads):.2f} dB**. No single rate is uniformly best. The small median advantages of lower rates are descriptive and do not establish a causal rate improvement: the sweep ran each rate sequentially over approximately 3½ hours.',
'', 'RSSI and recovered tone quality can disagree substantially. At **2.65 GHz**, mean RSSI is approximately **−58 dBm** across rates, yet the estimator reports no numeric SINAD. Around **2.4 GHz**, changing rate changes reported RSSI by up to approximately **18 dB**, while SINAD at nearby frequencies changes much less. Strong RSSI alone therefore does not establish a usable wanted signal.',
'', '## Measurement setup',
'', '| Item | Setting |','|---|---|',
'| TX | Original board on `pi0a-trixie` (`192.168.6.37`) |',
'| RX | Reconstructed board #1 on `pi0b-trixie` (`192.168.6.69`), RX only |',
'| RF path | RF24/HiF, user-confirmed nominal 90 dB attenuation between boards |',
'| TX power | 0 dBm requested at every point; no calibrated RF power measurement |',
'| Modulation | NBFM, 1 kHz sine, amplitude 0.4, deviation setting 2500 Hz |',
'| Frequency grid | 172 points; 50 MHz base grid plus band anchors and path-boundary points |',
'| Frequency endpoints | 30 MHz to 5,999.999 MHz; exactly 6 GHz is outside the supported exclusive upper limit |',
'| Rates | 4, 2, 1.333333, 1, 0.8, 0.666667, 0.5 and 0.4 MS/s; TX and RX changed together |',
'| Acquisition | 2 s settling, then 5 s measurement; fresh per-host sequences and monotonic timestamps |',
'| RX setup | Both squelches disabled; LED1 held low with LED0 preserved |',
'| SINAD | Backend 1 kHz NBFM estimate over 300–3000 Hz; no numeric estimate retained as missing |',
'', 'The fractional native rates are 4 MHz divided by 3 and 6; their integer command labels are rounded. The RX analog bandwidth remains 160 kHz and TX analog bandwidth 80 kHz, while the digital cutoff changes with sample rate (500 kHz at 4 MS/s down to 50 kHz at 0.4 MS/s). Thus the experiment varies a complete rate profile on both boards, rather than isolating one receiver parameter.',
'', 'These measurements characterize the **complete transmitter–attenuator–receiver link**. Requested TX power and nominal attenuation do not establish actual input power at every frequency. No receiver sensitivity threshold, noise figure, calibrated SINAD instrument comparison, or other-board comparison was measured in this run.',
'', '## RSSI and SINAD versus frequency',
'', '![Mean RSSI and estimated SINAD against frequency for all eight matched rates](analysis/frequency-overview.png)',
'', '[Vector plot](analysis/frequency-overview.svg). Points are measured frequency settings; connecting lines guide the eye and do not imply continuous-frequency coverage. Missing SINAD values interrupt the lines. Dotted vertical lines mark 2385 and 2495 MHz RF path boundaries.',
'', 'At 1 MS/s, representative point means are:',
'', '| Frequency MHz | RSSI dBm | Estimated SINAD dB |','|---:|---:|---:|']
for hz in [30000000,430100000,1000000000,1350000000,2384900000,2385000000,2399900000,2400000000,2494900000,2495000000,2650000000,4650000000,5999999000]:
 r=lookup[(hz,1000000)];sn=f'{r["sinad_db"]["mean"]:.2f}' if r['sinad_db'] else 'Unavailable';lines.append(f'| {hz/1e6:.3f} | {r["rssi_dbm"]["mean"]:.2f} | {sn} |')
lines += ['', 'The best point mean in the full matrix is **22.38 dB SINAD at 2494.9 MHz and 0.4 MS/s**. At 1 MS/s the same frequency gives 22.20 dB. At the high endpoint, all rates yield only about 3.1–3.3 dB estimated SINAD. These are link observations; the data do not identify whether TX response, RX response, local interference, conversion artifacts, or the RF path dominates each feature.',
'', '## Frequency and rate maps',
'', '![RSSI and estimated SINAD heatmaps by frequency and matched sample rate](analysis/frequency-rate-heatmaps.png)',
'', '[Vector plot](analysis/frequency-rate-heatmaps.svg). Rows are discrete rate settings, ordered from 4 down to 0.4 MS/s. Cell boundaries fall halfway between neighboring measured frequencies on the actual nonuniform frequency axis; colors are not interpolated. Gray SINAD cells mean unavailable estimates. Very closely spaced boundary measurements are easier to see in the zoom plots below.',
'', '## Sample-rate comparison',
'', '| Matched rate MS/s | Numeric SINAD / 172 | Median RSSI dBm | Median numeric SINAD dB | Points ≥12 dB SINAD | Median paired SINAD Δ vs 1 MS/s dB |',
'|---:|---:|---:|---:|---:|---:|']
for s in summary:lines.append(f'| {labels[s["rate_hz"]]} | {s["numeric_sinad"]} | {s["median_rssi_dbm"]:.2f} | {s["median_sinad_db"]:.2f} | {s["sinad_ge_12_db"]} | {s["median_sinad_difference_vs_1m_db"]:+.2f} |')
lines += ['', 'Medians weight each measured frequency equally, so clustered band anchors carry more weight than a uniform sweep would. Numeric-SINAD medians exclude missing points. Paired differences use only frequencies with fully valid windows in both compared rates; sample counts are in [rate-summary.csv](analysis/rate-summary.csv). The ≥12 dB column is a descriptive count, not a measured sensitivity threshold or pass/fail specification.',
'', '![RSSI and SINAD versus matched sample rate at selected frequencies](analysis/rate-profiles.png)',
'', '[Vector plot](analysis/rate-profiles.svg). Selected frequency profiles show that RSSI can change greatly with rate without a comparable change in recovered SINAD. At 2399.9 MHz, mean RSSI spans −94.96 to −113.21 dBm (18.25 dB), while the rate-specific SINAD means span only 11.30–11.55 dB. This pattern warrants a targeted RSSI/register/AGC investigation; it is not evidence that received wanted-signal power changed by 18 dB.',
'', '## RF path boundaries',
'', '![RSSI and estimated SINAD around the 2385 MHz and 2495 MHz path boundaries](analysis/path-boundary-zooms.png)',
'', '[Vector plot](analysis/path-boundary-zooms.svg). At 1 MS/s, SINAD increases from 3.31 dB at 2384.9 MHz to 12.04 dB at 2385 MHz, and decreases from 22.20 dB at 2494.9 MHz to 14.44 dB at 2495 MHz. These steps align with the native-modem/mixer path boundaries. The alignment supports investigating path-dependent behavior, but this combined-link sweep cannot assign the change to the TX or RX board.',
'', '## Missing and partially valid SINAD',
'', f'Numeric estimates are available at **{metrics["numeric_sinad"]:,} / 1,376 points**. **35 points have no numeric estimate**, and three additional points have partially valid windows. Missing values are not zero SINAD and are not proof of absent RF.',
'', '| Frequency MHz | Rates with no numeric SINAD (MS/s) |','|---:|---|']
for h,rates in missing:lines.append(f'| {h/1e6:g} | '+(', '.join(labels[r] for r in rates) if len(rates)!=8 else 'All eight rates')+' |')
lines += ['', '| Frequency MHz | Rate MS/s | Valid snapshots / total |','|---:|---:|---:|']
for r in partial:lines.append(f'| {r["frequency_hz"]/1e6:g} | {labels[r["sample_rate_hz"]]} | {r["sinad_db"]["n"]} / {r["snapshots"]} |')
lines += ['', 'The unavailable estimates recur at a small number of frequency settings, suggesting frequency-specific effects worth repeating. A source of interference, a conversion spur, tuning behavior, or inadequate wanted-tone quality are hypotheses; none is established by these data. In particular, the strong 2.65 GHz RSSI reading should be checked with TX off and with a frequency offset before interpreting it as the desired signal.',
'', '## Transport integrity and verification',
'', 'All 1,376 matrix measurement windows have zero increases or resets in the selected RX/TX transport counters. This does **not** mean every counter was zero throughout startup, tuning, and settling. The archived histories show:',
'', f'- RX `alignment.invalid_frames`: peak cumulative count **{history["rx"]["peak"]["alignment.invalid_frames"]}**, with **{history["rx"]["positive_increments"]["alignment.invalid_frames"]} positive increments** across the full recorded history.',
 f'- TX `driver.underrun_periods`: peak **{history["tx"]["peak"]["driver.underrun_periods"]}**, with **{history["tx"]["positive_increments"]["driver.underrun_periods"]} positive increments** across the full recorded history; this counter resets across some operations.',
'', 'Those increases occurred outside the retained matrix measurement windows. Nonzero counters inherited by later points are therefore distinct from new errors during measurement. The main matrix has no RX queue/audio drops, playback underruns/write errors, reader errors, alignment failures, TX writer errors, or TX queue drops in its recorded windows.',
'', 'The existing verifier independently recomputed point counts and RSSI/SINAD means from raw snapshots, checked role identities and requested frequency/rate/power settings, audited receiver commands, and checked final backend release. It passed. This analysis additionally checks the full 172 × 8 matrix and reads both hosts’ archived status histories. The original 18 artifact checksums were verified before this report was created.',
'', '## Interpretation and next measurements',
'', 'The Pi Zero pair sustained the planned acquisition at every supported rate without new transport errors in measurement windows. For this link and tone, choosing a rate alone does not resolve the large frequency-dependent SINAD variations. At 430.1 MHz, all eight preflight rates were usable and gave approximately 14.1–14.4 dB SINAD.',
'', 'A useful follow-up would repeat the low-SINAD and missing-estimate frequencies with TX off/on and small frequency offsets, then repeat a few anchors while alternating rates rather than sweeping each rate in a separate time block. To isolate receiver behavior, hold TX rate fixed while varying RX rate; a separate controlled run can isolate TX rate. Calibrated frequency-dependent input power or a known signal generator would be needed for a receiver sensitivity claim. No additional transmission was performed for this analysis.',
'', '## Files and reproducibility',
'', '- [Point measurements CSV](results.csv) and [full point summaries](results.jsonl).',
'- [Original completion report](REPORT.txt), [verification](verification.json), and [test plan](plan.json).',
'- [Analysis summary JSON](analysis/analysis-summary.json) and [rate-summary CSV](analysis/rate-summary.csv).',
'- [Raw paired snapshots](paired-snapshots.jsonl.gz), [TX host archive](tx-host-artifacts.tar.gz), and [RX host archive](rx-host-artifacts.tar.gz).',
'- [Analysis source](analysis/analyze_sweep.py) and [analysis artifact checksums](analysis/SHA256SUMS).',
'', 'Regenerate the report and PNG/SVG figures from the repository root:',
'', '```sh', 'python3 -m venv /tmp/cariboulite-sweep-analysis', '/tmp/cariboulite-sweep-analysis/bin/pip install -r \\', '  docs/baselines/20261121-pi0-board1-frequency-rate-sweep/analysis/requirements.txt', 'MPLCONFIGDIR=/tmp/cariboulite-matplotlib \\', '/tmp/cariboulite-sweep-analysis/bin/python \\', '  docs/baselines/20261121-pi0-board1-frequency-rate-sweep/analysis/analyze_sweep.py \\', '  docs/baselines/20261121-pi0-board1-frequency-rate-sweep', '```',
'', 'This baseline retains the report, plots, acquisition and analysis sources, point results, and compressed raw evidence in Git. The original acquisition directory and analysis virtual environment remain under ignored `build/`. Acquisition sources preserve the original host addresses and paths and must be reviewed before reuse; running them can transmit RF.', '']
(ROOT/'REPORT.md').write_text('\n'.join(lines))
(DEST/'analyze_sweep.py').write_bytes(pathlib.Path(__file__).read_bytes())
(DEST/'environment.txt').write_text(sys.version+'\nmatplotlib '+matplotlib.__version__+'\nnumpy '+np.__version__+'\n')
(DEST/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(DEST.iterdir()) if p.is_file() and p.name!='SHA256SUMS')+hashlib.sha256((ROOT/'REPORT.md').read_bytes()).hexdigest()+'  ../REPORT.md\n')
print(json.dumps(metrics,indent=2));print('REPORT',ROOT/'REPORT.md')
