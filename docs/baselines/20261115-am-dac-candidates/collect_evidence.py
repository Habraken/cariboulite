import pathlib,json,csv,hashlib,shutil
out=pathlib.Path('/tmp/am-candidate/evidence');out.mkdir(exist_ok=True)
runs={
 'initial':'am-candidates-initial-20261115-224301',
 'continuation':'am-candidates-continuation-20261115-224510',
 'calibration':'am-candidates-calibration-20261115-224656',
 'shaper-clock':'am-candidates-shaper-clock-20261115-225128',
 'comparison':'am-candidates-comparison-20261115-225509',
 'meter-gain42':'am-candidates-meter-gain42-20261115-230708',
 'meter-if37p5':'am-candidates-meter-if37p5-20261115-230854',
 'wide':'am-candidates-meter-wide-20261115-233541',
 'stair':'am-candidates-stair-20261115-230922',
 'production-unweighted':'am-shaping-production-20261115-231625',
 'production-weighted':'am-shaping-weighted-20261115-232836',
 'silence-off':'am-shaping-silence-off-20261115-233116',
 'silence-on':'am-shaping-silence-on-20261115-233131'}
manifest={}
for name,run in runs.items():
 p=pathlib.Path('/tmp')/run;records=json.loads((p/'results.json').read_text());rows=[]
 if name=='initial':records=[r for r in records if r['parameters'].get('mode')!=6]
 for r in records:
  tx,rx=r['tx_config'],r['rx_config'];row={'case':r['case'],'parameters_json':json.dumps(r['parameters'],sort_keys=True,separators=(',',':')),'tx_rate_hz':tx['tx_sample_rate_hz'],'rx_rate_hz':rx['sample_rate_hz'],'tx_power_setting_dbm':tx['tx_power_dbm'],'am_backoff_db':tx['tx_am_backoff_db'],'am_depth':tx['tx_am_depth'],'amplitude':tx['tx_tone_amp'],'production_shaping_flag':tx.get('tx_am_dac_shaping'),'tx_tuning_hz':tx['tx_frequency_hz'],'rx_wanted_hz':rx['requested_frequency_hz'],'rx_gain_db':rx['am_rf_gain_db'],'TXCI':r['tx_registers']['TXCI'],'TXCQ':r['tx_registers']['TXCQ'],'error_counter_deltas':sum(r['error_counter_deltas'].values()),'iq_sha256':hashlib.sha256(pathlib.Path(r['iq']).read_bytes()).hexdigest()}
  for k,v in r['metrics'].items():
   if isinstance(v,dict):
    for a,b in v.items():row[k+'_'+a]=b
   else:row[k]=v
  assert row['error_counter_deltas']==0 and row['adc_rail_samples']==0,(name,row)
  rows.append(row)
 fields=list(dict.fromkeys(k for row in rows for k in row))
 with (out/(name+'.csv')).open('w') as f:
  w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader();w.writerows(rows)
 shutil.copyfile(p/'metadata.json',out/(name+'-metadata.json'))
 manifest[name]={'run':run,'captures':len(rows),'error_counter_deltas':0,'adc_rail_samples':0}
shutil.copyfile('/tmp/am-candidate/calibration.json',out/'calibration.json')
for kind,run in [('raw','am-candidates-speech-raw-20261115-230050'),('if','am-candidates-speech-if-20261115-230148'),('ns','am-candidates-speech-ns-20261115-230239')]:
 p=pathlib.Path('/tmp')/run
 shutil.copyfile(p/'metadata.json',out/('speech-'+kind+'-metadata.json'));shutil.copyfile(p/'parameters.json',out/('speech-'+kind+'-parameters.json'))
manifest['listening']={'runs':['am-candidates-speech-raw-20261115-230050','am-candidates-speech-if-20261115-230148','am-candidates-speech-ns-20261115-230239'],'operator_comparison':'Both improve similarly','operator_residual_roughness':'No distinctive roughness noticed','source':'Approved private H650e recording; not included'}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest))
