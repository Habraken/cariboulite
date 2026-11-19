"""Offline assembly of the verified board #1 measurement and comparison reports.
Arguments: PRIMARY_RUN REPEAT_RUN BUILD_DIRECTORY. Both runs must include RX logs.
"""
from pathlib import Path
import subprocess,json,shutil,csv,hashlib,sys
base=Path(__file__).resolve().parent;primary=Path(sys.argv[1]);repeat=Path(sys.argv[2]);build=Path(sys.argv[3]);selected=build/'selected';reference=base.with_name('20261118-board4-frequency-sweep')
def run(script,*args,output=None):
 cmd=['python3',str(base/script),*[str(x) for x in args]]
 if output:
  with Path(output).open('w') as f:subprocess.run(cmd,stdout=f,check=True)
 else:subprocess.run(cmd,check=True)
run('merge_runs.py',primary,repeat,selected)
run('analyze.py',selected,base)
for src,dst in [('results.json','summary.json'),('metadata.json','metadata.json'),('source-selection.json','source-selection.json')]:shutil.copyfile(selected/src,base/dst)
for label,folder in [('primary',primary),('repeat',repeat)]:
 run('verify_roles.py',folder,output=base/(label+'-role-verification.json'))
 run('analyze.py',folder,build/(label+'-analysis'),'--no-plots',output=build/(label+'-verification-output.txt'))
 shutil.copyfile(build/(label+'-analysis')/'verification.json',base/(label+'-verification.json'))
 for role in ['rx','tx']:shutil.copyfile(folder/(role+'-session1-ready.json'),base/(label+'-'+role+'-ready.json'))
a={r['point']:r for r in json.loads((primary/'results.json').read_text())};b=json.loads((repeat/'results.json').read_text())
with (base/'repeat-comparison.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['point','frequency_hz','original_rssi_dbm','repeat_rssi_dbm','original_sinad_db','repeat_sinad_db','original_sinad_valid_fraction','repeat_sinad_valid_fraction'])
 for r in b:
  o=a[r['point']];w.writerow([r['point'],r['frequency_hz'],o['rssi_dbm']['mean'],r['rssi_dbm']['mean'],o['sinad_db']['mean'] if o['sinad_db'] else None,r['sinad_db']['mean'] if r['sinad_db'] else None,o['sinad_valid_fraction'],r['sinad_valid_fraction']])
history=[]
for folder in [primary,repeat]:
 for session in sorted(folder.glob('*/rx')):
  previous=None;events=[];commands=[json.loads(s) for s in (session/'commands.jsonl').read_text().splitlines()]
  for line in (session/'status.jsonl').open():
   p=json.loads(line);v=p['rx']['alignment']['invalid_frames'];old=previous['rx']['alignment']['invalid_frames'] if previous else 0
   if v>old:events.append(dict(count_increase=v-old,previous_status_monotonic_s=previous['monotonic_s'] if previous else None,first_observed_monotonic_s=p['monotonic_s'],state=p['state'],requested_frequency_hz=p['config']['requested_frequency_hz'],recent_commands=[c for c in commands if c['monotonic_s']<=p['monotonic_s']][-3:]))
   previous=p
  history.append(dict(run=folder.name,session=str(session.relative_to(folder)),observed_invalid_frame_events=sum(e['count_increase'] for e in events),events=events))
(base/'transport-history.json').write_text(json.dumps(history,indent=2)+'\n')
run('compare_board4.py',base,reference)
report=base.parents[1]/'board1-frequency-sweep-2026-11-18.md';run('write_report.py',base,report)
summary=json.loads((base/'comparison-summary.json').read_text());verification=json.loads((base/'verification.json').read_text());r=summary['rssi_board1_minus_board4_db'];s=summary['sinad_board1_minus_board4_db'];events=sum(x['observed_invalid_frame_events'] for x in history)
lines=['','## Direct comparison with board #4','',
'![Boards #1 and #4 overlaid, linear frequency](baselines/20261118-board1-frequency-sweep/comparison-linear.png)','',
'![Boards #1 and #4, 2–3 GHz detail](baselines/20261118-board1-frequency-sweep/comparison-2-to-3GHz.png)','',
'[Logarithmic comparison](baselines/20261118-board1-frequency-sweep/comparison-log.png) · [Paired CSV](baselines/20261118-board1-frequency-sweep/comparison-board4.csv) · [Comparison summary](baselines/20261118-board1-frequency-sweep/comparison-summary.json)','',
f"All **{summary['matched_frequencies']} frequencies** match exactly. Plan bytes, both hosts' backend/shared-library hashes, kernel versions, TX identity, tone, requested power/PA code and sample rates were verified. LED1 was low on both receivers. Wi-Fi/Bluetooth disable overlays and absence of exposed wireless devices were checked after the board #1 reboot; no wireless settings were changed.",'',
f"Across matched frequencies, mean raw RSSI **board #1 minus board #4** is **{r['mean']:+.2f} dB** (range {r['min']:+.2f} to {r['max']:+.2f}). Across the **{s['n']} pairs with numeric SINAD on both boards**, mean SINAD difference is **{s['mean']:+.2f} dB** (range {s['min']:+.2f} to {s['max']:+.2f}). Missing estimates are excluded from this difference, not assigned zero.",'',
f"SINAD is unreported on **both boards at {len(summary['unreported_on_both_hz'])} matched frequencies**, only on board #1 at **{len(summary['unreported_only_board1_hz'])}**, and only on board #4 at **{len(summary['unreported_only_board4_hz'])}**. Exact frequencies and the 2–3 GHz subset are recorded in the summary. These are time-separated, uncalibrated end-to-end comparisons; external RF conditions, connector power and attenuation versus frequency were not independently measured. Shared features alone do not distinguish the common TX, conversion paths, cable/attenuator response, receiver design or estimator.",'','## Receiver transport and final disposition','',
f"Full receiver status histories record **{events} invalid-frame events** across the two board #1 runs. Selected measurement windows with new transport errors: **{verification['points_with_new_transport_errors']}**. Absolute and per-window counters remain in the data, and [event details](baselines/20261118-board1-frequency-sweep/transport-history.json) identify any startup observations. Final release checks cover both hosts. Board #1 remains on pi4f with LED1 low and the radios stopped.",'']
paired={r['frequency_hz']:r for r in json.loads((base/'summary.json').read_text())};prior={r['frequency_hz']:r for r in json.loads((reference/'summary.json').read_text())}
fmt=lambda r,key: f"{r[key]['mean']:.2f}" if r[key] else 'Unreported'
lines+=['','## Matched frequencies of interest','', '| MHz | #1 RSSI dBm | #4 RSSI dBm | #1 SINAD dB | #4 SINAD dB |','|---:|---:|---:|---:|---:|']
for hz in [144100000,430100000,1240100000,2250000000,2300100000,2385000000,2400100000,2450000000,2495000000,2650000000,3000000000,3405000000,5800000000,5999999000]:
 x,y=paired[hz],prior[hz];lines.append(f"| {format(hz/1e6,'.3f').rstrip('0').rstrip('.')} | {fmt(x,'rssi_dbm')} | {fmt(y,'rssi_dbm')} | {fmt(x,'sinad_db')} | {fmt(y,'sinad_db')} |")
lines+=['', 'The 2650 MHz RSSI peak and absence of a reported SINAD occur on both boards. The bypass-region RSSI levels differ substantially; shared SINAD suppression does not imply identical received spectra or identical causes. The tables and overlays retain these differences without treating the response as unique to one receiver.', '']
report.write_text(report.read_text()+'\n'.join(lines))
print('REPORT '+str(report))
