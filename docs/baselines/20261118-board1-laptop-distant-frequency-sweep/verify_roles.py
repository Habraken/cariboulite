"""Verify board identities, no RX-side TX commands, and LED1-low before RX starts."""
import json,pathlib,sys
root=pathlib.Path(sys.argv[1]);checks=[];starts=0
for readyfile in sorted(root.glob('*-ready.json')):
 meta=json.loads(readyfile.read_text())['metadata'];role=meta['role']
 assert meta['host']=={'rx':'pi4f-trixie','tx':'pi4g-trixie'}[role]
 assert meta['board_uuid']=={'rx':'d8810044-c444-46f4-a548-703412cbf971','tx':'c132d0e7-d7dd-457a-b105-4a5cd97e7e80'}[role]
for d in sorted(root.glob('*/rx')):
 observed=None;low_written=False;preserved=None;session_starts=0
 for line in (d/'commands.jsonl').open():
  e=json.loads(line);tokens=e['request'].split();cmd=tokens[2];reply=e['reply'].split()
  assert not cmd.startswith('tx_'),e
  if cmd=='frequency':low_written=False
  if cmd=='fpga_led_get':
   assert reply[2:4]==['OK','OK'] and reply[-2]=='fpga_io';observed=int(reply[-1]);preserved=observed&1
  if cmd=='fpga_led_set':
   assert preserved is not None and int(tokens[3])==preserved and tokens[4]=='0'
   assert reply[2:4]==['OK','OK'] and int(reply[-1])&3==preserved;low_written=True
  if cmd=='start':assert low_written;session_starts+=1;starts+=1
 for line in (d/'status.jsonl').open():assert json.loads(line)['state'] not in ['TX','TX_STARTING','TX_STOPPING']
 checks.append(dict(session=str(d.relative_to(root)),rx_starts=session_starts,led0_preserved=preserved,led1_low_before_every_start=True,no_rx_tx_commands_or_states=True))
assert checks and starts>0
print(json.dumps(dict(receiver_board_identity_verified=True,transmitter_reference_identity_verified=True,rx_starts=starts,sessions=checks),indent=2))
