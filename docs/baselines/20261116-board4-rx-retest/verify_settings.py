"""Verify active settings and transport counters in every retained snapshot."""
import json,pathlib,sys
root=pathlib.Path(sys.argv[1]);count=0; transport_failures=[]
dividers={4000000:1,2000000:2,1333333:3,1000000:4,800000:5,666667:6,500000:8,400000:10}
for line in (root/'points.jsonl').open():
 r=json.loads(line);rx=r['rx'];tx=r['tx'];c=tx['config'];q=rx['config']
 assert c['tx_mode']=='nbfm' and c['tx_am_dac_shaping'] is False
 assert c['tx_source']=='tone' and c['tx_deviation_hz']==2500 and c['tx_tone_hz']==1000 and c['tx_tone_amp']==.4
 assert q['rx_deviation_hz']==2500 and q['deemphasis_us']==50 and q['pcm_gain']==8000
 assert q['am_offset_tuning_enabled'] is False and q['requested_frequency_hz']==430100000
 assert c['tx_frequency_hz']==430100000
 assert q['sample_rate_hz']==r['rx_rate_hz'] and c['tx_sample_rate_hz']==r['tx_rate_hz']
 assert abs(q['observed']['rx']['sample_rate_hz']-4000000/dividers[r['rx_rate_hz']])<.01
 for path,key in [('queue','drops'),('audio_queue','drops'),('playback','underruns'),('playback','write_errors'),('reader','errors'),('alignment','failures'),('alignment','invalid_frames')]:
  if rx['rx'][path][key]!=0:transport_failures.append({'point':r['point'],'role':'rx','counter':path+'.'+key,'value':rx['rx'][path][key]})
 if r['tx_power_dbm'] is not None:
  assert c['tx_power_dbm']==r['tx_power_dbm']
  assert abs(c['observed']['tx']['sample_rate_hz']-4000000/dividers[r['tx_rate_hz']])<.01
  assert q['observed']['rx']['analog_bandwidth_hz']==160000
  assert c['observed']['tx']['analog_bandwidth_hz']==80000
  assert abs(q['observed']['rx']['digital_cutoff_hz']-500000/dividers[r['rx_rate_hz']])<1
  assert abs(c['observed']['tx']['digital_cutoff_hz']-500000/dividers[r['tx_rate_hz']])<1
  for path,key in [('driver','underrun_periods'),('writer','errors'),('queue','drops')]:
   if tx['tx'][path][key]!=0:transport_failures.append({'point':r['point'],'role':'tx','counter':path+'.'+key,'value':tx['tx'][path][key]})
 count+=1
print(json.dumps({'verified_snapshots':count,'nbfm_mode_tone_no_am_shaping_verified':True,'zero_transport_errors':not transport_failures,'transport_failure_snapshots':transport_failures}))

assert count > 0
for p in root.glob('*/rx-ready.json'):
 m=json.loads(p.read_text())['metadata'];assert m['board_uuid']=='c2440a8d-3852-4c44-b96f-1b64a962e7dc' and m['host']=='pi4f-trixie'
for p in root.glob('*/rx/status.jsonl'):
 for line in p.open():assert json.loads(line)['state'] not in ['TX','TX_STARTING','TX_STOPPING']
for p in root.glob('*/rx/commands.jsonl'):
 for line in p.open():assert 'tx_start' not in json.loads(line)['request']
print(json.dumps({'receiver_uuid_checked':True,'no_rx_backend_tx_states_or_tx_start_commands':True}))

for p in root.glob('*/tx-ready.json'):
 m=json.loads(p.read_text())['metadata'];assert m['board_uuid']=='c132d0e7-d7dd-457a-b105-4a5cd97e7e80' and m['host']=='pi4g-trixie'
print(json.dumps({'tx_reference_uuid_checked':True}))
