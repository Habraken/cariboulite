import os,sys,json,socket,subprocess,time,signal,pathlib,hashlib,platform,select
root=pathlib.Path('/home/pi/src/cariboulite');role=sys.argv[1];case=sys.argv[2]
assert role in ('tx','rx');assert platform.node()=={'tx':'pi0b-trixie','rx':'pi0a-trixie'}[role]
uuid=pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode()
assert uuid=={'tx':'d8810044-c444-46f4-a548-703412cbf971','rx':'c132d0e7-d7dd-457a-b105-4a5cd97e7e80'}[role]
out=root/'build/whistle-reverse'/case/role;out.mkdir(parents=True,exist_ok=True)
ipc=pathlib.Path('/tmp')/('wh-'+hashlib.sha256((role+case).encode()).hexdigest()[:12]);ipc.mkdir(mode=0o700,exist_ok=True)
binary=root/('build/whistle-reverse/backend' if role=='tx' else 'build/radio-backend/cariboulite_radio_backend')
args=[str(binary),'--socket',str(ipc/'status.sock'),'--seconds','900','--rx-frequency','430100000','--tx-frequency','430100000','--tx-power','0','--tx-source','tone','--rx-playback',('plughw:CARD=H650e,DEV=0' if role=='rx' else 'plughw:CARD=USB,DEV=0'),'--rx-sinad','--rx-no-noise-squelch','--rx-gain-profile','matched','--rx-mode-gain','am=-7','--rx-mode-gain','usb=0','--rx-mode-gain','lsb=0']
meta={'args':args,'role':role,'host':platform.node(),'uuid':uuid,'boot_id':pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'kernel':platform.release(),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
(out/'metadata.json').write_text(json.dumps(meta,indent=2))
log=(out/'backend.log').open('w');commands=(out/'commands.jsonl').open('w');states=(out/'status.jsonl').open('w')
proc=control=status=None;last=None;seq=0;initial_pins=None

def send(x):print(json.dumps(x),flush=True)
def collect(wait=.2):
 global last
 end=time.monotonic()+wait
 while time.monotonic()<end:
  if select.select([status],[],[],max(0,end-time.monotonic()))[0]:
   data=status.recv(16384)
   if not data:raise RuntimeError('status disconnected')
   last=json.loads(data);states.write(data.decode()+'\n');states.flush()
 if proc.poll() is not None:raise RuntimeError('backend exited '+str(proc.returncode))
 return last

def command(text):
 global seq
 assert role=='tx' or not text.startswith('tx_'),'RX must not transmit'
 seq+=1;wire=f'v2 {seq} {text}\n';control.sendall(wire.encode());reply=control.recv(4096).decode().strip()
 commands.write(json.dumps({'time':time.time(),'request':wire.strip(),'reply':reply})+'\n');commands.flush()
 fields=reply.split();assert fields[1]==str(seq) and fields[2]=='OK',reply
 return fields

def waitstate(state):
 end=time.monotonic()+15
 while time.monotonic()<end:
  s=collect(.1)
  if s and s['control']['state']==state and not s['control']['transition']:return s
  if s and s['control']['state']=='FAULT':raise RuntimeError(str(s))
 raise RuntimeError('state timeout '+state+' last='+str(last and last['control']))

try:
 for p in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
  try:
   exe=p.read_bytes().split(b'\0')[0].split(b'/')[-1]
   assert exe not in (b'cariboulite_radio_backend',b'backend',b'cariboulite_test_app',b'test_app'),'Existing radio owner '+str(p)
  except (FileNotFoundError,ProcessLookupError,PermissionError):pass
 env=dict(os.environ)
 if role=='tx':env.update(LD_PRELOAD=str(root/'build/whistle-reverse/tx_timing_probe.so'),CARIBOULITE_TX_WRITE_TIMING='2,12,6')
 if role=='rx':env.update(LD_PRELOAD=str(root/'build/whistle-reverse/capture_tap.so'),WHISTLE_CAPTURE_PREFIX=str(out/'capture'),WHISTLE_CAPTURE_IQ_SAMPLES=str(int(sys.argv[3]) if len(sys.argv)>3 else 4000000))
 meta['diagnostic_environment']={k:env[k] for k in ('LD_PRELOAD','CARIBOULITE_TX_WRITE_TIMING','WHISTLE_CAPTURE_IQ_SAMPLES') if k in env};(out/'metadata.json').write_text(json.dumps(meta,indent=2))
 proc=subprocess.Popen(args,cwd=root,stdout=log,stderr=log,env=env)
 end=time.monotonic()+15
 while not (ipc/'control.sock').exists():
  assert proc.poll() is None,'backend exited';assert time.monotonic()<end,'socket timeout';time.sleep(.05)
 control=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);control.settimeout(20);control.connect(str(ipc/'control.sock'))
 status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.connect(str(ipc/'status.sock'))
 waitstate('IDLE')
 if role=='rx':
  fields=command('fpga_led_get');initial_pins=int(fields[-1]);command(f'fpga_led_set {initial_pins&1} 0')
  meta['initial_fpga_pins']=initial_pins;(out/'metadata.json').write_text(json.dumps(meta,indent=2))
 send({'ok':True,'ready':True,'metadata':meta})
 while True:
  if not select.select([sys.stdin],[],[],30)[0]:raise RuntimeError('heartbeat timeout')
  line=sys.stdin.readline()
  if not line:break
  req=json.loads(line);op=req['op']
  if op=='command':reply=command(req['text']);send({'ok':True,'reply':reply})
  elif op=='state':send({'ok':True,'snapshot':waitstate(req['state'])})
  elif op=='status':send({'ok':True,'snapshot':collect(req.get('wait',.2))})
  elif op=='capture':
   assert role=='rx';os.kill(proc.pid,signal.SIGUSR1);send({'ok':True,'armed_realtime_s':time.time()})
  elif op=='shutdown':
   command('tx_stop' if role=='tx' else 'stop');s=waitstate('IDLE');send({'ok':True,'stopped':True,'snapshot':s});break
  else:raise RuntimeError(op)
finally:
 if proc is not None and proc.poll() is None:
  if control is not None:
   try:
    command('tx_stop' if role=='tx' else 'stop');waitstate('IDLE')
    if role=='rx' and initial_pins is not None:command(f'fpga_led_set {initial_pins&1} {(initial_pins>>1)&1}')
   except Exception:pass
  proc.send_signal(signal.SIGTERM)
  try:proc.wait(timeout=12)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
 if control:control.close()
 if status:status.close()
 log.close();commands.close();states.close()
