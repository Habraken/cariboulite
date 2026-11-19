"""Offline assembly of the verified board #1 measurement and comparison reports.
Arguments: PRIMARY_RUN REPEAT_RUN BUILD_DIRECTORY. Both runs must include RX logs.
"""
from pathlib import Path
import subprocess,json,shutil,csv,hashlib,sys
base=Path(__file__).resolve().parent;primary=Path(sys.argv[1]);repeat=Path(sys.argv[2]);build=Path(sys.argv[3]);selected=build/'selected';reference=base.with_name('20261118-board1-frequency-sweep')
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
run('compare_nearby.py',base,reference)
report=base.parents[1]/'board1-laptop-distant-frequency-sweep-2026-11-18.md';run('write_report.py',base,report)
c=json.loads((base/'comparison-summary.json').read_text());v=json.loads((base/'verification.json').read_text());r=c['rssi_distant_minus_nearby_db'];s=c['sinad_distant_minus_nearby_db'];events=sum(x['observed_invalid_frame_events'] for x in history)
folder='baselines/20261118-board1-laptop-distant-frequency-sweep'
lines=['','## Laptop distant versus nearby','',
f'![Distant and nearby laptop runs overlaid]({folder}/comparison-linear.png)','',
f'![Distant and nearby laptop runs, 2–3 GHz detail]({folder}/comparison-2-to-3GHz.png)','',
f'[Logarithmic overlay]({folder}/comparison-log.png) · [Paired CSV]({folder}/comparison-nearby.csv) · [Comparison summary]({folder}/comparison-summary.json)','',
f"The same board #1 receiver and original TX were used at **{c['matched_frequencies']} identical frequencies**. Plan bytes, backend/shared-library hashes, kernel versions and backend arguments apart from socket paths match. The {c['matched_nearby_followup_points']} original follow-up frequencies are included; additional distant-run follow-ups: {len(c['additional_distant_followup_points'])}. Laptop placement is user-reported, without a measured distance or independently measured emissions.",'',
f"Mean raw RSSI **distant minus nearby** is **{r['mean']:+.2f} dB** (range {r['min']:+.2f} to {r['max']:+.2f}). For **{s['n']} pairs with numeric SINAD in both runs**, mean SINAD difference is **{s['mean']:+.2f} dB** (range {s['min']:+.2f} to {s['max']:+.2f}). Missing estimates are not assigned numeric values.",'',
f"Unreported SINAD occurs at **{len(c['unreported_on_both_hz'])} frequencies in both runs**, **{len(c['unreported_only_distant_hz'])} only with the laptop distant**, and **{len(c['unreported_only_nearby_hz'])} only with it nearby**. Exact frequencies and the 2–3 GHz subset are in the comparison summary.",'',f"Partially reported SINAD occurs at {len(c['partial_distant_hz'])} distant-run frequencies and {len(c['partial_nearby_hz'])} nearby-run frequencies. Means at these frequencies use only reported estimator windows above its criterion; reporting fractions are retained in the CSV and should not be interpreted as fully restored reception.", '', '## Selected paired measurements','',
'| MHz | Distant RSSI dBm | Nearby RSSI dBm | Distant SINAD dB | Nearby SINAD dB |','|---:|---:|---:|---:|---:|']
a={x['frequency_hz']:x for x in json.loads((base/'summary.json').read_text())};b={x['frequency_hz']:x for x in json.loads((reference/'summary.json').read_text())};fmt=lambda x,k:(f"{x[k]['mean']:.2f}"+(' (partial)' if k=='sinad_db' and x['sinad_valid_fraction']<1 else '')) if x[k] else 'Unreported'
for hz in [144100000,430100000,1240100000,2250000000,2300100000,2385000000,2400100000,2450000000,2495000000,2650000000,3000000000,3405000000,5800000000,5999999000]:
 x,y=a[hz],b[hz];lines.append(f"| {format(hz/1e6,'.3f').rstrip('0').rstrip('.')} | {fmt(x,'rssi_dbm')} | {fmt(y,'rssi_dbm')} | {fmt(x,'sinad_db')} | {fmt(y,'sinad_db')} |")
lines+=['','## Scope and final disposition','',
f"Full receiver histories record **{events} invalid-frame events** in this distant-laptop experiment. Measurement windows with new transport errors: **{v['points_with_new_transport_errors']}**. Both radios were stopped and released; board #1 remains on pi4f with LED1 low. The original nearby-laptop and board #4 measurements remain separate and unchanged.",'',
'These are time-separated, uncalibrated end-to-end measurements. Moving the laptop is the intended experimental change; laptop emissions, exact distance, other external RF sources, connector power, cable/attenuator response and thermal drift were not independently measured. Persistence or change of a feature in this comparison does not by itself identify its physical source.', '']
peak_difference=a[2650000000]['rssi_dbm']['mean']-b[2650000000]['rssi_dbm']['mean']
lead=f"At 2650 MHz, distant-minus-nearby mean RSSI is **{peak_difference:+.2f} dB**."
if a[2650000000]['sinad_db'] is None and b[2650000000]['sinad_db'] is None and abs(peak_difference)<1:
 lead='**The 2650 MHz peak persisted after moving the laptop away**, with no reported SINAD in either run. '+lead
if all(a[hz]['sinad_db'] is None and b[hz]['sinad_db'] is None for hz in a if 2385000000<=hz<2495000000):
 lead+=' SINAD also remains unreported at the sampled bypass frequencies (`2385 ≤ f < 2495 MHz`).'
existing=report.read_text();title,body=existing.split('\n',1)
report.write_text(title+'\n\n'+lead+'\n'+body+'\n'.join(lines))
print('REPORT '+str(report))
