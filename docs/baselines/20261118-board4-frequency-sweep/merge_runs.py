"""Select open-squelch repeats where original audio gating invalidated measurement.
Original runs remain unchanged; every final row records its source run.
"""
import json,pathlib,sys
primary=pathlib.Path(sys.argv[1]);repeat=pathlib.Path(sys.argv[2]);dest=pathlib.Path(sys.argv[3]);dest.mkdir(parents=True,exist_ok=True)
a=json.loads((primary/'results.json').read_text());b=json.loads((repeat/'results.json').read_text());selected={r['point']:r for r in a};replaced={r['point'] for r in b};orig={r['point']:r for r in a}
for r in b:selected[r['point']]=r
expected=json.loads((primary/'plan.json').read_text());assert set(selected)=={p['point'] for p in expected['points']}
assert replaced==set(json.loads(pathlib.Path(__file__).with_name('retry-points.json').read_text())),'repeat points missing'
provenance={str(i):dict(run=repeat.name if i in replaced else primary.name,noise_squelch_enabled=i not in replaced,original_sinad_valid_fraction=orig[i]['sinad_valid_fraction'],reason='Open-squelch diagnostic repeat; estimator itself is pre-gate' if i in replaced else 'Original point retained') for i in selected}
with (dest/'points.jsonl').open('w') as out:
 for folder,is_repeat in [(primary,False),(repeat,True)]:
  for line in (folder/'points.jsonl').open():
   x=json.loads(line)
   if (x['point'] in replaced)==is_repeat:out.write(line)
(dest/'results.json').write_text(json.dumps([selected[i] for i in sorted(selected)],indent=2)+'\n')
(dest/'plan.json').write_text(json.dumps(expected,indent=2)+'\n')
meta=json.loads((primary/'metadata.json').read_text());meta['repeat_run']=repeat.name;meta['repeated_point_count']=len(replaced);meta['repeat_reason']='Check unavailable SINAD with both squelches disabled; estimator runs pre-gate';meta['selected_data_note']='Use source-selection.json to distinguish original open-gate measurements from open-squelch repeats. Both complete raw runs retained.'
(dest/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n');(dest/'source-selection.json').write_text(json.dumps(provenance,indent=2)+'\n')
print(json.dumps(dict(final_points=len(selected),repeated_points=len(replaced),primary_run=primary.name,repeat_run=repeat.name),indent=2))
