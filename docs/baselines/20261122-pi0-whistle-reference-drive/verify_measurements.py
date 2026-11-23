"""Validate the complete drive-sweep settings and zero-error capture windows."""
from pathlib import Path
import json
HERE=Path(__file__).resolve().parent
plan=json.loads((HERE/'plan.json').read_text());rows=json.loads((HERE/'measurements.json').read_text())
assert len(rows)==len(plan['cases'])==36
assert [m['case'] for m in rows]==plan['cases']
confirmation=json.loads((HERE/'confirmation-measurements.json').read_text())
confirmation_plan=json.loads((HERE/'confirmation-plan.json').read_text())
assert len(confirmation)==len(confirmation_plan['cases'])==4
assert [m['case'] for m in confirmation]==confirmation_plan['cases']
rows+=confirmation
health=[]
for m in rows:
 cfg=m['case'];h={'case':cfg['name']}
 for role in ('rx','tx'):
  v=m['verification'][role]
  drive=cfg['drive_ma'] if role=='rx' else 8
  assert v['drive_ma']==drive and v['clock_hz']==32000000
  assert v['clko_register']==(((drive//2-1)<<3)|2)
  assert v['pll_startup_locked'] and v['pll_startup_raw']&0x8000
  assert v['route']==(2 if role=='rx' else 0 if cfg.get('no_tx') else 4)
  for stage in ('before','after'):
   s=m[stage][role]
   assert s['hardware']['common']['registers']['CLKO']==v['clko_register']
   assert s['channel']=='RF24' and s['config']['tx_power_dbm']==0
   assert s['config']['cdcss']['rx_code']=='off' and s['config']['cdcss']['tx_code']=='off'
   assert s['config']['sample_rate_hz']==(cfg['rate'] if role=='rx' else 1000000)
   assert s['control']['state']==('RX' if role=='rx' else 'IDLE' if cfg.get('no_tx') else 'TX')
   assert abs(s['config']['requested_frequency_hz']-(430100000+(cfg['offset'] if role=='rx' else 0)))<1
 for role,section,field in [('rx','queue','drops'),('rx','audio_queue','drops'),('rx','playback','underruns'),('rx','playback','write_errors'),('rx','reader','errors'),('tx','writer','errors'),('tx','driver','underrun_periods')]:
  before=m['before'][role][role][section];after=m['after'][role][role][section]
  h[f'{role}.{section}.{field}']=after[field]-before[field] if after is not None and field in after else None
 h['rx_clipped_samples']=m['after']['rx']['rx']['audio_output']['clipped_samples']-m['before']['rx']['rx']['audio_output']['clipped_samples']
 assert all(v==0 or v is None for k,v in h.items() if k!='case'),h
 health.append(h)
(HERE/'verified-settings.json').write_text(json.dumps({'case_count':len(rows),'main_case_count':36,'confirmation_case_count':4,'rx_drive_ma_sequence':[8,4,2,8],'reference_hz':32000000,'tx_drive_ma':8,'pll_check':'existing startup readback, not continuous lock monitoring','capture_health':health},indent=2)+'\n')
print('Verified 40 cases: RX drive, fixed TX drive, clock frequency, PLL startup lock, frontend route, operating states and zero health deltas.')
