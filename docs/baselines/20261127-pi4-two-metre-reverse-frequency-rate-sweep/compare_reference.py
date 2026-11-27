import csv, json, pathlib, statistics, sys, hashlib
root=pathlib.Path(sys.argv[1]); here=pathlib.Path(__file__).resolve().parent
assert json.loads((root/'matrix-verification.json').read_text())['passed']
rows=[json.loads(s) for s in (root/'results.jsonl').read_text().splitlines()]
reference=here/'reference-results.jsonl'
if not reference.exists():reference=here.parent/'20261127-pi4-two-metre-frequency-rate-sweep/pi4-two-metre-20261127T081042Z/results.jsonl'
ref=[json.loads(s) for s in reference.read_text().splitlines()]
old={(r['frequency_hz'],r['sample_rate_hz']):r for r in ref if r['phase']=='matrix'}
new=[r for r in rows if r['phase']=='matrix']; pairs=[]
for r in new:
    o=old[(r['frequency_hz'],r['sample_rate_hz'])]
    valid=r['transport_clean'] and o['transport_clean'] and r['sinad_valid_fraction']==o['sinad_valid_fraction']==1
    pairs.append(dict(frequency_hz=r['frequency_hz'],sample_rate_hz=r['sample_rate_hz'],rssi_difference_db=r['rssi_dbm']['mean']-o['rssi_dbm']['mean'],sinad_difference_db=(r['sinad_db']['mean']-o['sinad_db']['mean']) if valid else None,sinad_pair_fully_valid=valid))
with (root/'reference-comparison.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(pairs[0]),lineterminator='\n');w.writeheader();w.writerows(pairs)
lines=['# Pi 4 reciprocal sweep with approximately two-metre separation','',
'Repaired board #2 pi4f TX (192.168.6.163) to original pi4g RX (192.168.6.77).',
'Both hosts updated to GitHub dev-fpga-pa-control a997ec4 before this run.',
'Operator reports approximately 2 m separation, added 2 m RG179 and nominal 90 dB attenuators.',
'Added cable loss is additional and uncalibrated. These are complete TX/link/RX results.',
'',f'Completed {len(new)} matrix points; independently verified 172 frequencies × eight matched rates.',
f'Numeric SINAD: {sum(r["sinad_db"] is not None for r in new)}/{len(new)}; transport-flagged windows: {sum(not r["transport_clean"] for r in new)}.',
'', '## Comparison with the 27 November forward direction','',
'Reference is pi4g TX to pi4f RX, measured earlier the same day with the same software, nominal attenuation and cable setup. The operator moved the headset and attempted to minimize disturbance. Differences are reverse minus forward; they combine both boards\' TX/RX responses and the link, and cannot isolate either board or calibrated sensitivity. SINAD differences include only fully valid transport-clean pairs.',
'', '| Rate MS/s | Numeric SINAD | Median numeric SINAD dB | Median RSSI difference dB | Valid SINAD pairs | Median SINAD difference dB |', '|---:|---:|---:|---:|---:|---:|']
for rate in sorted({r['sample_rate_hz'] for r in new},reverse=True):
    rr=[r for r in new if r['sample_rate_hz']==rate]; pp=[r for r in pairs if r['sample_rate_hz']==rate]
    sn=[r['sinad_db']['mean'] for r in rr if r['sinad_db']]; ds=[r['sinad_difference_db'] for r in pp if r['sinad_difference_db'] is not None]
    fmt=lambda x:'Unavailable' if not x else f'{statistics.median(x):.2f}'
    lines.append(f'| {rate/1e6:g} | {len(sn)} | {fmt(sn)} | {fmt([r["rssi_difference_db"] for r in pp])} | {len(ds)} | {fmt(ds)} |')
lines+=['','See REPORT.txt, verification.json, matrix-verification.json, results.csv and reference-comparison.csv for evidence.']
(root/'REPORT.md').write_text('\n'.join(lines)+'\n')
(root/'SHA256SUMS').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+f.name+'\n' for f in sorted(root.iterdir()) if f.is_file() and f.name!='SHA256SUMS'))
