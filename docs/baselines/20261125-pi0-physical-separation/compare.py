"""Offline comparison against the original, same-direction standard-IF run."""
from pathlib import Path
import json,hashlib
root=Path(__file__).resolve().parent
old=json.loads((root.parent/'20261125-pi0-demodulated-audio/analysis/summary.json').read_text())
new=json.loads((root/'analysis/summary.json').read_text())
measurements=json.loads((root/'atx_brx_195343/measurements.json').read_text())
assert len(new)==len(measurements)==5
rows=[]; hashes={}
for r in new:
 b=next(x for x in old if x['case']=='atx_brx_183933_'+r['name'])
 m=next(x for x in measurements if x['case']==r['case'])
 assert not m['start_retries'] and all(v==0 for v in r['health_deltas'].values())
 cap=root/'data'/r['case']/'rx/capture'
 info=json.loads(cap.with_suffix('.json').read_text())
 assert info['iq_samples']==8000000
 assert info['raw_frames']==info['filtered_frames']==info['pcm_frames']==384000
 for suffix,size in (('.iq',32000000),('.raw.f32',1536000),('.filtered.f32',1536000),('.pcm',768000)):
  p=cap.with_suffix(suffix);assert p.stat().st_size==size
  hashes[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
 rows.append({'name':r['name'],'before_live_sinad_db':b['live_sinad_after_db'],'after_live_sinad_db':r['live_sinad_after_db'],
  'before_rssi_dbm':b['rssi_after_dbm'],'after_rssi_dbm':r['rssi_after_dbm'],
  'whistle_band_3000_3800hz_reduction_db':b['raw_3000_3800hz_power_db_normalized']-r['raw_3000_3800hz_power_db_normalized']})
for p in (root/'data').glob('*/*/metadata.json'):
 m=json.loads(p.read_text());assert m['backend_exit']==0 and not m.get('cleanup_warning') and not m.get('forced_kill')
 assert m['tx_if_override_hz'] is None
for r in json.loads((root/'release.json').read_text()): assert not r['owners']
(root/'comparison.json').write_text(json.dumps(rows,indent=2)+'\n')
(root/'capture-sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
(root/'verification.json').write_text(json.dumps({'passed':True,'captures':5,'complete_iq_samples':40000000,'complete_frames_per_audio_tap':1920000,'capture_health_deltas_all_zero':True,'no_start_retries':True,'standard_if_only':True,'clean_shutdown_all_sessions':True},indent=2)+'\n')
print(json.dumps(rows,indent=2))
