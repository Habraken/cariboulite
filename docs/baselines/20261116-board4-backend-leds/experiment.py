"""Controlled LED/RX experiment through the shipped controller; never starts TX."""
import pathlib,subprocess,socket,json,time,tempfile,hashlib
root=pathlib.Path('/home/pi/src/cariboulite');base=root/'build/board4-backend-leds-20261116';base.mkdir(exist_ok=True);out=base/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());out.mkdir()
assert pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode()=='c2440a8d-3852-4c44-b96f-1b64a962e7dc'
for f in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
 try:assert f.read_bytes().split(b'\0')[0].split(b'/')[-1] not in (b'cariboulite_radio_backend',b'cariboulite_test_app',b'test_app')
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
ipc=pathlib.Path(tempfile.mkdtemp(prefix='backend-led-test-'));binary=root/'build/radio-backend/cariboulite_radio_backend';controller=binary.with_name('cariboulite_radio_controller');events=[];log=(out/'backend.log').open('w');s=None
meta={'boot_id':pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'board_uuid':'c2440a8d-3852-4c44-b96f-1b64a962e7dc','backend_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'library_sha256':hashlib.sha256((root/'build/libcariboulite.so').read_bytes()).hexdigest(),'test_app_used':False,'tx_used':False}
(out/'metadata.json').write_text(json.dumps(meta,indent=2))
p=subprocess.Popen([str(binary),'--channel','RF24','--rx-frequency','430100000','--rx-sample-rate','4000000','--rx-sinad','--rx-playback','plughw:USB,0,0','--seconds','180','--socket',str(ipc/'status.sock')],stdout=log,stderr=log)
def save(): (out/'events.json').write_text(json.dumps(events,indent=2))
def cmd(text):
 r=subprocess.run([str(controller),'--socket',str(ipc/'control.sock'),'--timeout','15','--command',text],capture_output=True,text=True,timeout=20)
 v={'command':text,'exit_code':r.returncode,'stdout':r.stdout,'stderr':r.stderr};events.append(v);save();return v
try:
 deadline=time.monotonic()+20
 while not (ipc/'control.sock').exists():
  assert p.poll() is None and time.monotonic()<deadline;time.sleep(.05)
 s=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);s.settimeout(1);s.connect(str(ipc/'status.sock'))
 def observe(seconds):
  snapshots=[];end=time.monotonic()+seconds
  while time.monotonic()<end:
   try:snapshots.append(json.loads(s.recv(16384)))
   except socket.timeout:pass
  events.append({'snapshots':snapshots});save();return snapshots
 def trial(name):
  cmd('rx stop');observe(.3);r=cmd('rx start');snaps=observe(3);print(json.dumps({'trial':name,'start_exit':r['exit_code'],'state':snaps[-1]['state'] if snaps else None,'alignment':snaps[-1]['rx']['alignment'] if snaps else None}),flush=True);cmd('rx stop');observe(.3)
 observe(.3);trial('baseline')
 read=cmd('fpga leds');print(json.dumps(read),flush=True);assert read['exit_code']==0
 import re
 match=re.search(r'LED0=(\d) LED1=(\d)',read['stdout']);assert match
 a,b=map(int,match.groups());trial('read_only')
 for name,x,y in [('same_state_write',a,b),('led0_toggle',1-a,b),('restore',a,b)]:
  write=cmd(f'fpga leds {x} {y}');print(json.dumps(write),flush=True)
  trial(name)
 for x,y in [(1,0),(1,1),(0,1),(0,0)]:
  write=cmd(f'fpga leds {x} {y}');assert write['exit_code']==0,write
 trial('exact_testapp_write_pattern')
 cmd(f'fpga leds {a} {b}')
finally:
 if p.poll() is None:
  try:cmd('rx stop')
  except Exception:pass
  p.terminate();p.wait(timeout=15)
 if s:s.close()
 log.close();save();print('ARTIFACTS '+str(out),flush=True)
