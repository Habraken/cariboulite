#!/usr/bin/env python3
"""Owned AM link-test backend. Run only after the operator confirms attenuation.

JSON lines on stdin: wire (RX/TX owner commands), status, capture (RX loopback
WAV), shutdown. Each helper owns its backend/control socket; EOF/signals unkey
and terminate it. TX is permitted only for the explicit pi4g transmitter role.
"""
import hashlib,json,os,pathlib,platform,re,signal,socket,subprocess,sys,time
root=pathlib.Path('/home/pi/src/cariboulite')
os.environ['OPENBLAS_NUM_THREADS']='1'
sys.path.insert(0,'/tmp/am-candidate')
role,run=sys.argv[1:3]
assert role in ('tx','rx') and re.fullmatch(r'[a-zA-Z0-9_-]+',run)
assert platform.node()==('pi4g-trixie' if role=='tx' else 'pi4f-trixie')
playback=sys.argv[3] if len(sys.argv)>3 else 'hw:Loopback,0,0'
out=pathlib.Path('/tmp')/run/role;out.mkdir(parents=True,exist_ok=True)
ipc=out/'ipc';ipc.mkdir(mode=0o700,exist_ok=True)
binary=pathlib.Path(sys.argv[5]) if len(sys.argv)>5 else root/'build/radio-backend/cariboulite_radio_backend'
args=[str(binary),'--channel','RF24','--rx-frequency','430100000','--tx-frequency','430100000',
      '--rx-mode','am','--rx-am-offset','on','--tx-mode','am','--tx-am-depth','0.8','--tx-am-backoff','6','--tx-source','tone',
      '--tx-power','-12','--tx-sample-rate','1000000','--rx-sample-rate','1000000',
      '--rx-playback',playback,'--rx-am-rf-attenuation',sys.argv[4] if len(sys.argv)>4 else '0','--rx-volume','-12','--seconds','1800','--socket',str(ipc/'status.sock')]
if role=='tx':args += ['--tx-am-dac-shaping',os.environ.get('AM_TX_SHAPING','on')]
os.environ['AM_DIAG_CONTROL']=str(out/'diag-control')
os.environ['AM_IQ_CONTROL']=str(out/'iq-control')
log=(out/'backend.log').open('w');events=(out/'events.jsonl').open('w');statuses=(out/'status.jsonl').open('w')
proc=None;control=None;status=None;sequence=0;last=None;injector=None

def emit(value): print(json.dumps(value),flush=True)
def collect(seconds=0):
 global last
 end=time.monotonic()+max(.25,seconds)
 hard_end=end+2
 while True:
  try:
   raw=status.recv(32768)
   if not raw:raise RuntimeError('Status socket closed')
   last=json.loads(raw);statuses.write(raw.decode()+'\n');statuses.flush()
  except BlockingIOError:
   now=time.monotonic()
   if now>=end and last is not None and now-last['monotonic_s']<.5:break
   if now>hard_end:raise RuntimeError('Status did not refresh after draining queued snapshots')
   time.sleep(.01)
 if proc.poll() is not None:raise RuntimeError('Backend exited '+str(proc.returncode))
 return last

def command(text):
 global sequence
 if role=='rx' and text.startswith('tx_'):raise RuntimeError('RX helper cannot issue TX commands')
 sequence+=1;wire=f'v2 {sequence} {text}\n';control.sendall(wire.encode())
 reply=control.recv(4096).decode().strip();fields=reply.split()
 events.write(json.dumps({'monotonic_s':time.monotonic(),'request':wire.strip(),'reply':reply})+'\n');events.flush()
 if len(fields)!=33 or fields[1]!=str(sequence) or fields[2]!='OK':raise RuntimeError(reply)
 return reply

def waitstate(wanted):
 end=time.monotonic()+20
 while time.monotonic()<end:
  value=collect(.08)
  if value and value['state']==wanted and not value['control']['transition']:
   if wanted!='TX' or value['tx']['running']:return value
  if value and (value['state']=='FAULT' or value['tx'].get('fault')):raise RuntimeError(value)
 raise RuntimeError('State timeout '+wanted)

def interrupted(signum,frame):raise SystemExit('Signal '+str(signum))
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
try:
 for path in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
  try:
   exe=path.read_bytes().split(b'\0')[0].split(b'/')[-1]
   if exe in (b'cariboulite_radio_backend',b'cariboulite_test_app',b'test_app'):
    raise RuntimeError('Radio owner already running: '+str(path))
  except (FileNotFoundError,PermissionError,ProcessLookupError):pass
 proc=subprocess.Popen(args,cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
 end=time.monotonic()+20
 while time.monotonic()<end and proc.poll() is None:
  if (ipc/'control.sock').exists() and (ipc/'status.sock').exists():break
  time.sleep(.05)
 control=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);control.settimeout(25);control.connect(str(ipc/'control.sock'))
 status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.connect(str(ipc/'status.sock'));status.setblocking(False)
 command('get');waitstate('IDLE')
 if role=='tx':command('tx_tone 1000 0.5')
 metadata={'host':platform.node(),'role':role,'kernel':platform.release(),'pid':proc.pid,'args':args,
           'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
           'backend_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
           'driver_srcversion':pathlib.Path('/sys/module/smi_stream_dev/srcversion').read_text().strip()}
 (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
 emit({'ok':True,'ready':True,'out':str(out),'metadata':metadata})
 for line in sys.stdin:
  request=json.loads(line);op=request['op']
  if op=='wire':
   reply=command(request['command'])
   value=waitstate(request['state']) if request.get('state') else collect(.25)
   emit({'ok':True,'reply':reply,'snapshot':value})
  elif op=='inject_voice':
   assert role=='tx' and injector is None
   injector=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-stream_loop','-1','-i','/tmp/am-voice-reference.wav','-ac','1','-ar','48000','-f','alsa','plughw:Loopback,0,1'],stdin=subprocess.DEVNULL,stdout=log,stderr=log)
   collect(.25)
   if injector.poll() is not None:raise RuntimeError('Voice injector exited')
   emit({'ok':True,'voice_injector_pid':injector.pid})
  elif op=='status':emit({'ok':True,'snapshot':collect(request.get('wait',0))})
  elif op=='diag':
   assert role=='tx'
   epoch=time.monotonic_ns();values=[epoch,int(request['mode']),int(request.get('ci',-1)),int(request.get('cq',-1)),float(request.get('gain',1)),float(request.get('phase',0)),float(request.get('di',0)),float(request.get('dq',0)),float(request.get('offset',40000 if int(request['mode']) in (5,6) else 25200))]
   path=pathlib.Path(os.environ['AM_DIAG_CONTROL']);temp=path.with_suffix('.tmp');temp.write_text(' '.join(map(str,values))+'\n');temp.replace(path)
   end=time.monotonic()+5;ack=pathlib.Path(str(path)+'.ack')
   while time.monotonic()<end:
    tokens=ack.read_text().split() if ack.exists() else []
    if tokens and tokens[0]==str(epoch):break
    time.sleep(.02)
   else:raise RuntimeError('Diagnostic parameters not acknowledged')
   emit({'ok':True,'parameters':values,'ack':ack.read_text().strip(),'snapshot':collect(.25)})
  elif op=='iq':
   assert role=='rx'
   name=request['name'];assert re.fullmatch(r'[a-zA-Z0-9_-]+',name)
   rate=int(last['config']['sample_rate_hz']);count=int(request.get('count',rate//5));assert 10000<=count<=2000000
   path=out/(name+'.iq16');epoch=time.monotonic_ns();controlpath=pathlib.Path(os.environ['AM_IQ_CONTROL']);temp=controlpath.with_suffix('.tmp');temp.write_text(f'{epoch} {count} {path}\n');temp.replace(controlpath)
   end=time.monotonic()+10;done=pathlib.Path(str(path)+'.done')
   while time.monotonic()<end:
    if done.exists() and done.read_text().strip()==str(epoch):break
    time.sleep(.02)
   else:raise RuntimeError('IQ capture timeout')
   assert path.stat().st_size==count*4
   from metrics import measure
   metrics=measure(path,rate,request.get("if_center",12500))
   emit({'ok':True,'iq':str(path),'metrics':metrics,'snapshot':collect(.25)})
  elif op=='capture':
   assert role=='rx' and playback=='hw:Loopback,0,0'
   name=request['name'];assert re.fullmatch(r'[a-zA-Z0-9_-]+',name)
   path=out/(name+'.wav');seconds=int(request.get('seconds',3));assert 1<=seconds<=10
   collect(.5)
   p=subprocess.run(['arecord','-q','-D','hw:Loopback,1,0','-f','S16_LE','-r','48000','-c','1','-t','wav','-d',str(seconds),str(path)],stdout=log,stderr=log,timeout=seconds+10)
   if p.returncode:raise RuntimeError('Audio capture failed '+str(p.returncode))
   emit({'ok':True,'wav':str(path),'snapshot':collect(.25)})
  elif op=='shutdown':
   command('tx_stop' if role=='tx' else 'stop');value=waitstate('IDLE')
   emit({'ok':True,'snapshot':value,'stopped':True});break
  else:raise RuntimeError('Unknown operation '+op)
except Exception as exc:
 emit({'ok':False,'error':str(exc),'out':str(out)})
 sys.exit(1)
finally:
 if proc is not None and proc.poll() is None:
  if control:
   try:command('tx_stop' if role=='tx' else 'stop')
   except Exception:pass
  proc.terminate()
  try:proc.wait(timeout=15)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
 if injector is not None and injector.poll() is None:
  injector.terminate()
  try:injector.wait(timeout=5)
  except subprocess.TimeoutExpired:injector.kill();injector.wait()
 if control:control.close()
 if status:status.close()
 log.close();events.close();statuses.close()
