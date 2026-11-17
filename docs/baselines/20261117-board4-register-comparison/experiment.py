"""Compare exposed FPGA reads around LED1 writes; RX checks never start TX."""
import pathlib,subprocess,socket,json,time,tempfile,hashlib,platform
root=pathlib.Path('/home/pi/src/cariboulite');out=root/'build/board4-register-comparison-20261117'/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());out.mkdir(parents=True)
uuid=pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode();assert uuid=='c2440a8d-3852-4c44-b96f-1b64a962e7dc' and platform.node()=='pi4f-trixie'
for f in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
 try:assert f.read_bytes().split(b'\0')[0].split(b'/')[-1] not in (b'cariboulite_radio_backend',b'cariboulite_test_app',b'test_app'),'Radio owned'
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
ipc=pathlib.Path(tempfile.mkdtemp(prefix='board4-reg-compare-'));binary=root/'build/radio-backend/cariboulite_radio_backend';events=[];snapshots=[];rx_checks=[];sequence=0;peer=None;status=None;led0=None;log=(out/'backend.log').open('w')
args=[str(binary),'--channel','RF24','--rx-frequency','430100000','--rx-sample-rate','4000000','--rx-playback','plughw:USB,0,0','--seconds','180','--socket',str(ipc/'status.sock')]
(out/'metadata.json').write_text(json.dumps({'host':platform.node(),'board_uuid':uuid,'boot_id':pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'args':args,'backend_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'library_sha256':hashlib.sha256((root/'build/libcariboulite.so').read_bytes()).hexdigest(),'tx_used':False,'test_app_used':False,'reset_or_program_commands':False},indent=2)+'\n')
p=subprocess.Popen(args,stdout=log,stderr=log)
def save():
 (out/'events.json').write_text(json.dumps(events,indent=2)+'\n');(out/'snapshots.json').write_text(json.dumps(snapshots,indent=2)+'\n');(out/'rx-checks.json').write_text(json.dumps(rx_checks,indent=2)+'\n')
def command(text):
 global sequence
 sequence+=1;wire=f'v2 {sequence} {text}';peer.send((wire+'\n').encode());reply=peer.recv(4096).decode().strip();fields=reply.split();assert fields[:2]==['v2',str(sequence)]
 result={'monotonic_s':time.monotonic(),'request':wire,'reply':reply};events.append(result);save();return fields

def read_registers(phase,label):
 fields=command('fpga_regs');assert len(fields)==45 and fields[33]=='fpga_regs',fields
 result={'phase':phase,'label':label,'state':fields[7],'ok':fields[2]=='OK','valid_mask':int(fields[34]),'values':list(map(int,fields[35:])),'monotonic_s':time.monotonic()};snapshots.append(result);save();return result

def set_led1(value):
 fields=command(f'fpga_led_set {led0} {value}');assert fields[2:4]==['OK','OK'] and fields[-2]=='fpga_io',fields
 assert int(fields[-1])&3==led0|(value<<1),fields

def observe(seconds):
 start=time.monotonic();end=start+seconds;records=[]
 while time.monotonic()<end:
  try:
   v=json.loads(status.recv(16384))
   if v['monotonic_s']>=start:records.append(v)
  except socket.timeout:pass
 return records
try:
 deadline=time.monotonic()+20
 while not (ipc/'control.sock').exists():
  assert p.poll() is None and time.monotonic()<deadline;time.sleep(.05)
 peer=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);peer.settimeout(15);peer.connect(str(ipc/'control.sock'))
 status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.settimeout(.25);status.connect(str(ipc/'status.sock'))
 command('get');initial=read_registers('stopped','initial');assert initial['state']=='IDLE' and initial['valid_mask']==1023
 led0=initial['values'][6]&1
 for cycle in range(3):
  for value in [0,1]:
   set_led1(value)
   for repeat in range(3):
    time.sleep(.05);read_registers('stopped',f'cycle{cycle+1}_led1={value}_read{repeat+1}')
 set_led1(0);read_registers('stopped','restored_low')
 print('STOPPED_COMPARISON_COMPLETE',flush=True)
 for n,value in enumerate([0,1,0],1):
  command('stop');set_led1(value);before=read_registers('rx_checks',f'trial{n}_led1={value}_before')
  reply=command('start');records=observe(3)
  check={'led1':value,'start_reply':reply,'snapshots':records};rx_checks.append(check);save()
  command('stop');after=read_registers('rx_checks',f'trial{n}_led1={value}_after')
  print(json.dumps({'led1':value,'start_result':reply[2:4],'states':list(dict.fromkeys(r['state'] for r in records)),'final_alignment':records[-1]['rx']['alignment'] if records else None}),flush=True)
finally:
 if peer:
  try:
   command('stop')
   if led0 is not None:set_led1(0);read_registers('cleanup','final_low')
  except Exception as e:events.append({'cleanup_error':str(e)})
  peer.close()
 if status:status.close()
 if p.poll() is None:p.terminate();p.wait(timeout=15)
 log.close();save();print('ARTIFACTS '+str(out),flush=True)
