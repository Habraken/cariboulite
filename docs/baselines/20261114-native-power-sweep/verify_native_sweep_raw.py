"""Independently reconcile every saved snapshot with its point summary."""
import collections,hashlib,json,pathlib,statistics,sys
root=pathlib.Path(sys.argv[1]);dest=pathlib.Path(sys.argv[2]);dest.mkdir(parents=True,exist_ok=True)
rows={r['point']:r for r in json.loads((root/'results.json').read_text())};groups=collections.defaultdict(list)
previous=-1
with (root/'points.jsonl').open() as f:
 for line in f:
  r=json.loads(line);p=rows[r['point']];rx=r['rx'];tx=r['tx']
  assert rx['monotonic_s']>previous;previous=rx['monotonic_s']
  assert r['axis']==p['axis'] and r['rx_rate_hz']==p['rx_rate_hz'] and r['tx_rate_hz']==p['tx_rate_hz'] and r['tx_power_dbm']==p['tx_power_dbm']
  assert rx['state']=='RX' and tx['state']==('TX' if p['tx_power_dbm'] is not None else 'IDLE')
  assert not rx['control']['transition'] and not tx['control']['transition']
  assert rx['config']['sample_rate_hz']==p['rx_rate_hz'] and tx['config']['tx_sample_rate_hz']==p['tx_rate_hz']
  assert tx['config']['tx_tone_hz']==1000 and abs(tx['config']['tx_tone_amp']-.4)<1e-6
  groups[p['point']].append(r)
for point,packets in groups.items():
 row=rows[point];assert len(packets)==row['packets']
 assert packets[0]['rx']['monotonic_s']==row['first_rx_monotonic_s'] and packets[-1]['rx']['monotonic_s']==row['last_rx_monotonic_s']
 vals={'rssi_dbm':[],'sinad_db':[],'noise_rms':[]}
 for r in packets:
  rx=r['rx'];v=rx['hardware']['rf24']['registers']['RSSI']
  if isinstance(v,int) and v!=127:vals['rssi_dbm'].append(v-256 if v>127 else v)
  for metric,path,key in [('sinad_db','sinad','db'),('noise_rms','noise','rms')]:
   v=rx['rx'][path][key]
   if isinstance(v,(int,float)):vals[metric].append(v)
 for metric,v in vals.items():
  if not v:assert row[metric] is None;continue
  expected={'n':len(v),'mean':statistics.mean(v),'median':statistics.median(v),'min':min(v),'max':max(v),'stdev':statistics.stdev(v) if len(v)>1 else 0}
  for key,value in expected.items():assert abs(value-row[metric][key])<1e-9,(point,metric,key)
assert set(groups)==set(rows) and len(rows)==240
result={'points':len(rows),'snapshots':sum(len(p) for p in groups.values()),'statistics_reconciled':True,'states_profiles_tone_and_monotonic_order_verified':True,'points_sha256':hashlib.sha256((root/'points.jsonl').read_bytes()).hexdigest()}
(dest/'raw-verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
