"""Independently validate the stopped LED/register comparison and RX outcomes."""
import pathlib,json,re,sys
root=pathlib.Path(sys.argv[1]);dest=pathlib.Path(sys.argv[2]);dest.mkdir(exist_ok=True)
rows=json.loads((root/'snapshots.json').read_text());events=json.loads((root/'events.json').read_text());rx=json.loads((root/'rx-checks.json').read_text())
names=['SYS.module_version','SYS.system_version','SYS.manufacturer','SYS.gap_sync_source','IO.module_version','IO.mode','IO.digital','IO.rf_state','SMI.module_version','SMI.fifo_channel_direction'];defined=[255,255,255,255,255,31,251,255,255,255];dynamic=[0,0,0,0,0,0,248,0,0,3]
assert len(rows)==27 and all(r['valid_mask']==1023 and r['ok'] and r['state']=='IDLE' for r in rows)
stopped=[r for r in rows if r['phase']=='stopped'];assert len(stopped)==20
baseline=stopped[0]['values'];assert all(baseline[i]==1 for i in [0,1,2,4,8]);assert baseline[3]==0 and baseline[5]&31==0 and baseline[7]==0x56
changes=[];unexpected=[]
for r in stopped:
 match=re.search(r'led1=(\d)',r['label'])
 if match:assert (r['values'][6]>>1)&1==int(match.group(1))
 for i,(a,b) in enumerate(zip(baseline,r['values'])):
  diff=(a^b)&defined[i]
  if diff:changes.append({'label':r['label'],'register':names[i],'baseline':a,'observed':b,'defined_xor':diff,'dynamic_xor':diff&dynamic[i]})
  non_led=diff & (0xfd if i==6 else 255)
  if non_led:unexpected.append({'label':r['label'],'register':names[i],'xor':non_led,'non_dynamic_xor':non_led&~dynamic[i]})
assert not unexpected
assert all(c['register']=='IO.digital' and c['defined_xor']==2 for c in changes)
assert [x['led1'] for x in rx]==[0,1,0]
checks=[]
for x in rx:
 expected='RX' if x['led1']==0 else 'FAULT';assert x['snapshots'] and all(s['state']==expected for s in x['snapshots'])
 assert x['start_reply'][2:4]==(['OK','OK'] if not x['led1'] else ['ERROR','HARDWARE'])
 first=x['snapshots'][0]['rx']['alignment'];last=x['snapshots'][-1]['rx']['alignment']
 checks.append({'led1':x['led1'],'state':expected,'alignment_first':first,'alignment_last':last,'failures_delta_during_observation':last['failures']-first['failures'],'emitted_samples_delta_during_observation':last['emitted_samples']-first['emitted_samples']})
allowed={'get','fpga_regs','fpga_led_set','stop','start'}
for e in events:
 if 'request' in e:assert e['request'].split()[2] in allowed
assert rows[-1]['phase']=='cleanup' and rows[-1]['values'][6]&3==baseline[6]&1
result={'snapshot_count':len(rows),'complete_successful_register_reads':len(rows)*10,'stopped_comparison_snapshots':20,'repeated_low_high_cycles':3,'baseline_values':dict(zip(names,baseline)),'baseline_expected_versions_and_idle_controls_match':True,'only_LED1_defined_bit_changed':True,'other_defined_bit_changes':unexpected,'changes':changes,'rx_checks':checks,'final_led0':rows[-1]['values'][6]&1,'final_led1':(rows[-1]['values'][6]>>1)&1,'no_reset_program_or_tx_start_commands':True}
(dest/'comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['changes','rx_checks','baseline_values']},indent=2))
