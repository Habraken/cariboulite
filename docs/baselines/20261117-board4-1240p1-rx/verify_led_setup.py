"""Verify explicit LED1-low setup and preserved LED0 before each RX start."""
import json,pathlib,sys
root=pathlib.Path(sys.argv[1]);cases=[];boots=set()
for p in sorted(root.glob('*/rx-ready.json')):
 meta=json.loads(p.read_text())['metadata'];setup=meta['led_setup'];boots.add(meta['boot_id'])
 assert meta['board_uuid']=='c2440a8d-3852-4c44-b96f-1b64a962e7dc'
 before=setup['before_reply'].split();after=setup['after_reply'].split()
 assert before[-2]==after[-2]=='fpga_io' and before[2:4]==after[2:4]==['OK','OK']
 pins_before=int(before[-1]);pins_after=int(after[-1]);assert not pins_after&2 and pins_before&1==pins_after&1==setup['preserved_led0']
 commands=[json.loads(s) for s in (p.parent/'rx/commands.jsonl').read_text().splitlines()]
 sets=[(i,c) for i,c in enumerate(commands) if ' fpga_led_set ' in c['request']];assert len(sets)==1
 i,write=sets[0];assert write['request'].split()[-2:]==[str(setup['preserved_led0']),'0']
 starts=[j for j,c in enumerate(commands) if c['request'].endswith(' start')];assert starts and i<min(starts)
 assert not any('fpga_program' in c['request'] or 'fpga_soft_reset' in c['request'] or 'fpga_hard_reset' in c['request'] for c in commands[i+1:])
 assert write['reply']==setup['after_reply']
 cases.append({'case':p.parent.name,'pins_before':pins_before,'pins_after':pins_after})
assert len(cases)==8 and len(boots)==1
print(json.dumps({'verified_rx_rate_cases':8,'boot_ids':sorted(boots),'led1_low_before_all_rx_starts':True,'led0_preserved':True,'cases':cases}))
