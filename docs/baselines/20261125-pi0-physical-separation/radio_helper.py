"""One privileged, bounded radio session; controlled via JSON lines on stdin."""
import hashlib,json,os,pathlib,platform,select,signal,socket,subprocess,sys,time
root=pathlib.Path('/home/pi/src/cariboulite');tool=root/'build/demod-audio-20261125'
role,node,case=sys.argv[1:4];program='--program' in sys.argv[4:]
tx_if=int(sys.argv[sys.argv.index('--tx-if')+1]) if '--tx-if' in sys.argv else None
assert tx_if is None or (role=='tx' and tx_if==2385000000)
assert os.geteuid()==0 and role in ('tx','rx') and node in ('a','b')
assert platform.node()=='pi0'+node+'-trixie'
uuid=pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode()
assert uuid=={'a':'c132d0e7-d7dd-457a-b105-4a5cd97e7e80','b':'36b67ac4-b313-4369-99bd-4b3e09caabfa'}[node]
assert case and all(c.islower() or c.isdigit() or c in '-_' for c in case)
out=tool/'runs'/case/role;out.mkdir(parents=True,exist_ok=False)
ipc=pathlib.Path('/tmp')/('cl-demod-'+hashlib.sha256((role+case).encode()).hexdigest()[:15]);ipc.mkdir(mode=0o700)
binary=tool/'backend'
args=[str(binary),'--channel','RF24','--socket',str(ipc/'status.sock'),'--seconds','300' if role=='tx' else '90',
 '--rx-frequency','430100000','--tx-frequency','430100000','--rx-sample-rate','1000000','--tx-sample-rate','1000000',
 '--tx-power','0','--tx-source','tone','--rx-mode','nbfm','--tx-mode','nbfm','--rx-sinad',
 '--rx-playback','plughw:CARD=H650e,DEV=0' if role=='rx' else 'null',
 '--rx-gain-profile','matched','--rx-mode-gain','am=-7','--rx-mode-gain','usb=0','--rx-mode-gain','lsb=0',
 '--rx-ctcss','off','--tx-ctcss','off','--rx-cdcss','off','--tx-cdcss','off']
meta={'role':role,'node':node,'host':platform.node(),'uuid':uuid,'args':args,'fpga_reloaded':program,
 'started':time.time(),'boot_id':pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
 'kernel':platform.release(),'uid':os.geteuid(),'build':json.loads((tool/'build.json').read_text()),'tx_if_override_hz':tx_if,
 'capture_test':json.loads((tool/'capture-test.json').read_text()),'iq_samples_expected':8000000,'audio_frames_expected':384000}
(out/'metadata.json').write_text(json.dumps(meta,indent=2))
log=(out/'backend.log').open('w');commands=(out/'commands.jsonl').open('w');states=(out/'status.jsonl').open('w')
proc=control=status=None;last=None;seq=0;initial_pins=None
def send(x):print(json.dumps(x),flush=True)
def collect(seconds=.1):
 global last
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  if select.select([status],[],[],min(.08,max(0,until-time.monotonic())))[0]:
   raw=status.recv(1048576)
   if not raw:raise RuntimeError('status disconnected')
   last=json.loads(raw);states.write(raw.decode()+'\n');states.flush()
  if proc.poll() is not None:raise RuntimeError('backend exited '+str(proc.returncode))
 return last
def command(text):
 global seq
 assert role=='tx' or not text.startswith('tx_'),'RX role cannot issue TX commands'
 seq+=1;wire=f'v2 {seq} {text}\n';control.send(wire.encode());reply=control.recv(16384).decode().strip()
 commands.write(json.dumps({'time':time.time(),'request':wire.strip(),'reply':reply})+'\n');commands.flush()
 fields=reply.split();assert fields[1]==str(seq) and fields[2]=='OK',reply
 return reply
def waitstate(expected,timeout=15):
 until=time.monotonic()+timeout
 while time.monotonic()<until:
  p=collect(.1)
  if p and p['state']==expected and not p['control']['transition']:return p
  if p and p['state']=='FAULT' and not p['control']['transition']:raise RuntimeError('radio fault '+str(p['tx'].get('fault')))
 raise RuntimeError('state timeout '+expected)
try:
 for f in pathlib.Path('/proc').glob('[0-9]*/exe'):
  try:assert f.resolve().name not in ('cariboulite_radio_backend','backend','cariboulite_test_app','test_app'),'radio owner '+str(f)
  except (OSError,PermissionError):pass
 env=dict(os.environ)
 for k in ('LD_PRELOAD','WHISTLE_REF_DRIVE_MA','CL_CAPTURE_PREFIX','DEMOD_TX_IF_HZ'):env.pop(k,None)
 if tx_if:
  assert json.loads((tool/'if-test.json').read_text())['passed']
  env.update(LD_PRELOAD=str(tool/'if_probe.so'),DEMOD_TX_IF_HZ=str(tx_if))
  meta['if_probe_sha256']=hashlib.sha256((tool/'if_probe.so').read_bytes()).hexdigest()
 if role=='rx':env.update(CL_CAPTURE_PREFIX=str(out/'capture'),CL_CAPTURE_IQ_SAMPLES='8000000',CL_CAPTURE_AUDIO_FRAMES='384000')
 proc=subprocess.Popen(args,cwd=root,stdout=log,stderr=log,env=env)
 until=time.monotonic()+20
 while not (ipc/'control.sock').exists():
  assert proc.poll() is None,'backend exited during startup'
  assert time.monotonic()<until,'socket timeout'
  time.sleep(.05)
 control=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);control.settimeout(20);control.connect(str(ipc/'control.sock'))
 status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.connect(str(ipc/'status.sock'));status.setblocking(False)
 command('get');waitstate('IDLE')
 pins=command('fpga_led_get');initial_pins=int(pins.split()[-1]);meta['initial_fpga_pins']=initial_pins
 if program:command('fpga_program');waitstate('IDLE',30)
 if role=='rx':
  command(f'fpga_led_set {initial_pins&1} 0')
  command('squelch 0 0 0.2 0.3');meta['initial_fpga_pins']=initial_pins
 (out/'metadata.json').write_text(json.dumps(meta,indent=2));send({'ok':True,'ready':True,'out':str(out),'metadata':meta})
 while True:
  if not select.select([sys.stdin],[],[],35)[0]:raise RuntimeError('controller heartbeat expired')
  line=sys.stdin.readline()
  if not line:break
  request=json.loads(line)
  try:
   op=request['op']
   if op=='command':send({'ok':True,'reply':command(request['text'])})
   elif op=='state':send({'ok':True,'snapshot':waitstate(request['state'])})
   elif op=='status':send({'ok':True,'snapshot':collect(request.get('wait',.1))})
   elif op=='capture':
    assert role=='rx';os.kill(proc.pid,signal.SIGUSR1);send({'ok':True,'armed':time.time()})
   elif op=='shutdown':
    command('tx_stop' if role=='tx' else 'stop');waitstate('IDLE');send({'ok':True,'stopped':True});break
   else:raise RuntimeError('unknown operation')
  except Exception as e:send({'ok':False,'error':str(e)})
except Exception as e:send({'ok':False,'error':str(e)});raise
finally:
 if proc is not None and proc.poll() is None:
  if control is not None:
   try:
    command('tx_stop' if role=='tx' else 'stop');waitstate('IDLE')
    if initial_pins is not None:
     meta['led_restore_reply']=command(f'fpga_led_set {initial_pins&1} {(initial_pins>>1)&1}')
   except Exception as e:meta['cleanup_warning']=str(e)
  proc.send_signal(signal.SIGTERM)
  try:proc.wait(timeout=12)
  except subprocess.TimeoutExpired:proc.kill();proc.wait();meta['forced_kill']=True
 if control:control.close()
 if status:status.close()
 meta['backend_exit']=proc.returncode if proc else None;meta['finished']=time.time()
 (out/'metadata.json').write_text(json.dumps(meta,indent=2))
 log.close();commands.close();states.close()
