"""Offline verification of complete captures, settings, errors and release."""
from pathlib import Path
import hashlib,json,math,tarfile
root=Path(__file__).resolve().parent
runs=[p for p in root.glob('[ab]tx_[ab]rx_*') if p.is_dir()]
rows=[r for p in runs for r in json.loads((p/'measurements.json').read_text())]
assert len(rows)==18 and len({r['case'] for r in rows})==18
bycase={r['case']:r for r in rows};capture_hashes={};metadata=[];capture_infos={}
for archive in sorted(root.glob('*-host.tar.gz')):
 with tarfile.open(archive) as t:
  for m in t:
   assert not Path(m.name).is_absolute() and '..' not in Path(m.name).parts and not m.issym() and not m.islnk()
   if not m.isfile():continue
   if m.name.endswith('/metadata.json'):
    d=json.load(t.extractfile(m));metadata.append(d)
    assert d['uid']==0 and d['backend_exit']==0 and not d.get('forced_kill') and not d.get('cleanup_warning')
    assert d['capture_test']['passed']
    assert d['build']['installed_backend_sha256']=='0fb395c1a95ff810b1cfa7e8909a5a995532c341ddf656823d670cbbb79bc859'
    assert d['build']['library_sha256']=='cfeb3a7788165213bd7f1d2f3bcb4cc6c436596f5ef6b292cb27f439e045e06b'
    assert int(d['led_restore_reply'].split()[-1])&3==d['initial_fpga_pins']&3
   if m.name.endswith('/rx/capture.json'):
    d=json.load(t.extractfile(m));case=Path(m.name).parts[-3];capture_infos[case]=d
    assert d['iq_samples']==8000000 and d['raw_frames']==d['filtered_frames']==d['pcm_frames']==384000
   if '/rx/capture.' in m.name:
    f=t.extractfile(m);digest=hashlib.file_digest(f,'sha256').hexdigest()
    capture_hashes[m.name]={'bytes':m.size,'sha256':digest}
assert set(capture_infos)==set(bycase)
assert len(metadata)==24 # 18 RX cases and six TX sessions.
assert len({d['build']['diagnostic_backend_sha256'] for d in metadata})==1
assert len([d for d in metadata if d.get('tx_if_override_hz')==2385000000])==1
for case,row in bycase.items():
 d=capture_infos[case];prefix=case+'/rx/capture'
 for suffix,size in (('.iq',32000000),('.raw.f32',1536000),('.filtered.f32',1536000),('.pcm',384000*2*d['channels'])):
  assert capture_hashes[prefix+suffix]['bytes']==size
 for role in ('rx','tx'):
  before=row['before'][role];after=row['after'][role];c=after['config']
  assert after['channel']=='RF24' and c['sample_rate_hz']==1000000 and c['tx_sample_rate_hz']==1000000
  assert c['ctcss']['rx_frequency_hz']==c['ctcss']['tx_frequency_hz']==0
  assert c['cdcss']['rx_code']==c['cdcss']['tx_code']=='off'
  assert after['hardware']['common']['registers']['CLKO']==26
  assert after['sequence']>before['sequence'] and after['monotonic_s']>before['monotonic_s']
  expected='RX' if role=='rx' else 'TX' if row['tx_on'] else 'IDLE'
  assert before['state']==after['state']==expected and not after['control']['transition']
  for w in after[role]['workers'].values():
   if w['created']:
    assert w['alive'] and not w['last_error']
    assert w['scheduling']['policy_name']=='SCHED_FIFO' and not w['scheduling']['rt_error']
 assert row['after']['rx']['config']['requested_frequency_hz']==430100000+row.get('rx_offset_hz',0)
 if row['tx_on']:
  c=row['after']['tx']['config'];assert c['tx_frequency_hz']==430100000 and abs(c['tx_actual_frequency_hz']-430100000)<10
  assert c['tx_power_dbm']==c['tx_actual_power_dbm']==0
  assert abs(c['tx_tone_hz']-1000)<1e-6 and abs(c['tx_tone_amp']-row['amplitude'])<1e-6
 assert not row['start_retries']
releases=json.loads((root/'release.json').read_text());assert len(releases)==2
assert all(not r['owners'] and r['fuser_exit']==1 and not r['fuser_stdout'] and not r['fuser_stderr'] for r in releases)
analysis=json.loads((root/'analysis/summary.json').read_text());assert {r['case'] for r in analysis}==set(bycase)
assert all(all(v==0 for v in r['health_deltas'].values()) for r in analysis)
for r in analysis:
 assert all(math.isfinite(v) and abs(v)<=500.000001 for v in r['raw_correction_ppm_min_mean_max'])
 assert abs(r['production_replay_frames']-384000)<=1
 if r['raw_tone_frequency_hz'] is not None:
  assert abs(r['raw_tone_frequency_hz']-999.5)<.02
  assert abs(r['replay_tone_frequency_hz']-1000)<.02
assert json.loads((root/'decoder-test.json').read_text())['passed']
controls=[]
for run in sorted(runs):
 plan=json.loads((run/'plan.json').read_text())
 if plan.get('tx_if_control_hz'):
  r=next(r for r in json.loads((run/'measurements.json').read_text()) if r['name']=='tone_probe')
  keys=('CS','CCF0L','CCF0H','CNL','CNM')
  controls.append({'run':run.name,'tx_if_hz':plan['tx_if_control_hz'],
   'rx_registers':[r['after']['rx']['hardware']['rf24']['registers'][k] for k in keys],
   'tx_registers':[r['after']['tx']['hardware']['rf24']['registers'][k] for k in keys]})
assert [c['tx_if_hz'] for c in controls]==[2495000000,2385000000,2495000000]
assert controls[0]['rx_registers']==controls[1]['rx_registers']==controls[2]['rx_registers']
assert controls[0]['tx_registers']==controls[2]['tx_registers']!=controls[1]['tx_registers']
(root/'if-control-verification.json').write_text(json.dumps(controls,indent=2)+'\n')
(root/'capture-sha256.json').write_text(json.dumps(capture_hashes,indent=2)+'\n')
v={'passed':True,'recordings':18,'receiver_sessions':18,'transmitter_sessions':6,'decoded_iq_samples':144000000,
 'frames_per_audio_tap':6912000,'zero_capture_window_error_deltas':True,'all_rt_scheduling_applied':True,
 'ctcss_cdcss_off':True,'tx_start_retries':0,'release_verified':True,'production_software_unchanged':True}
(root/'verification.json').write_text(json.dumps(v,indent=2)+'\n');print(json.dumps(v,indent=2))
