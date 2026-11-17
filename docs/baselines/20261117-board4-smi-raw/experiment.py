"""One clean backend lifetime per LED1 state, no TX/reset/program commands."""
import hashlib,json,os,pathlib,platform,socket,subprocess,tempfile,time
root=pathlib.Path('/home/pi/src/cariboulite')
out=root/'build/board4-smi-raw'/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());out.mkdir(parents=True)
uuid=pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode()
assert platform.node()=='pi4f-trixie' and uuid=='c2440a8d-3852-4c44-b96f-1b64a962e7dc'
for f in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
 try:assert f.read_bytes().split(b'\0')[0].split(b'/')[-1] not in (b'cariboulite_radio_backend',b'cariboulite_test_app'),'radio owned'
 except (FileNotFoundError,PermissionError,ProcessLookupError):pass
binary=root/'build/radio-backend/cariboulite_radio_backend'
(out/'metadata.json').write_text(json.dumps(dict(host=platform.node(),board_uuid=uuid,boot_id=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),library_sha256=hashlib.sha256((root/'build/libcariboulite.so').read_bytes()).hexdigest(),backend_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),tx_used=False),indent=2))
trials=[]
for trial,value in enumerate([0,1,0],1):
 label=f'trial{trial}-led1-{value}';ipc=pathlib.Path(tempfile.mkdtemp(prefix='smi-raw-'));raw=out/(label+'.bin');events=[];records=[];peer=None;status=None;led0=None
 args=[str(binary),'--channel','RF24','--rx-frequency','430100000','--rx-sample-rate','4000000','--rx-playback','plughw:USB,0,0','--seconds','120','--socket',str(ipc/'status.sock')]
 log=(out/(label+'.log')).open('w');env=dict(os.environ,CARIBOULITE_SMI_RAW_CAPTURE=str(raw));p=subprocess.Popen(args,env=env,stdout=log,stderr=log)
 def command(text):
  wire=f'v2 {len(events)+1} {text}';peer.send((wire+'\n').encode());reply=peer.recv(8192).decode().strip();events.append(dict(request=wire,reply=reply));return reply.split()
 try:
  deadline=time.monotonic()+25
  while not (ipc/'control.sock').exists():
   assert p.poll() is None and time.monotonic()<deadline;time.sleep(.05)
  peer=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);peer.settimeout(15);peer.connect(str(ipc/'control.sock'))
  status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.settimeout(.3);status.connect(str(ipc/'status.sock'))
  fields=command('fpga_regs');assert fields[2:4]==['OK','OK'] and fields[7]=='IDLE';led0=int(fields[41])&1
  fields=command(f'fpga_led_set {led0} {value}');assert fields[2:4]==['OK','OK'] and int(fields[-1])&3==led0|(value<<1)
  before=command('fpga_regs');start=command('start');deadline=time.monotonic()+3
  while time.monotonic()<deadline:
   try:records.append(json.loads(status.recv(16384)))
   except socket.timeout:pass
  command('stop');after=command('fpga_regs')
 finally:
  if peer:
   try:
    command('stop')
    if led0 is not None:command(f'fpga_led_set {led0} 0')
   finally:peer.close()
  if status:status.close()
  if p.poll() is None:p.terminate();p.wait(timeout=20)
  log.close()
  (out/(label+'-events.json')).write_text(json.dumps(events,indent=2))
  (out/(label+'-status.json')).write_text(json.dumps(records,indent=2))
 assert raw.exists() and raw.stat().st_size>0,'capture missing'
 analysis=subprocess.check_output(['python3',str(root/'software/libcariboulite/tools/analyze_smi_raw.py'),str(raw)])
 (out/(label+'-analysis.json')).write_bytes(analysis)
 a=json.loads(analysis);summary=dict(label=label,led1=value,start_result=start[2:4],last_alignment=records[-1]['rx']['alignment'] if records else None,bytes=a['bytes'],phases=a['phases']);trials.append(summary)
 (out/'summary.json').write_text(json.dumps(trials,indent=2));print(json.dumps(summary),flush=True)
print('ARTIFACTS '+str(out),flush=True)
