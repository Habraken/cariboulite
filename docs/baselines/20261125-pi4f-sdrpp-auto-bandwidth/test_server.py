from pathlib import Path
import json,subprocess,shutil,os,signal,time,socket,struct,re
repo=Path('/home/pi/src/SDRPlusPlus');out=repo/'build/auto-bandwidth-fix-20261125'
subprocess.run(['python3',str(out/'test_auto_bandwidth.py'),'--source',str(out/'main.cpp')],check=True)
ldd=subprocess.run(['ldd','-r',str(out/'soapy_source.so')],capture_output=True,text=True,check=True);(out/'link-check.txt').write_text(ldd.stdout+ldd.stderr);assert 'undefined symbol:' not in ldd.stdout+ldd.stderr
servers=[]
for p in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  args=p.read_bytes().split(b'\0')
  if args and Path(os.fsdecode(args[0])).name=='sdrpp':
   assert b'--server' in args
   servers.append({'pid':int(p.parent.name),'args':[os.fsdecode(x) for x in args if x]})
 except (FileNotFoundError,ProcessLookupError):pass
for row in servers:
 try:os.kill(row['pid'],signal.SIGTERM)
 except ProcessLookupError:pass
 until=time.monotonic()+4;p=Path('/proc',str(row['pid']),'cmdline')
 while p.exists() and p.read_bytes() and time.monotonic()<until:time.sleep(.1)
 row['forced_kill']=bool(p.exists() and p.read_bytes())
 if row['forced_kill']:os.kill(row['pid'],signal.SIGKILL)
(out/'previous-servers.json').write_text(json.dumps(servers,indent=2)+'\n')
config=out/'test-config';config.mkdir(exist_ok=True)
for p in Path('/home/pi/.config/sdrpp').glob('*.json'):shutil.copy2(p,config/p.name)
modules=out/'test-modules';modules.mkdir(exist_ok=True);link=modules/'soapy_source.so'
if not link.exists():link.symlink_to(out/'soapy_source.so')
p=config/'config.json';cfg=json.loads(p.read_text());cfg.update(source='SoapySDR',modulesDirectory=str(modules),modules=[],moduleInstances={'Soapy Source':{'module':'soapy_source','enabled':True}});p.write_text(json.dumps(cfg,indent=2))
p=config/'soapy_source_config.json';cfg=json.loads(p.read_text());cfg['device']='CaribouLite HiF[3831373b]';settings=cfg.setdefault('devices',{}).setdefault(cfg['device'],{});settings.update(bandwidth=0,sampleRate=4000000,agc=True,antenna=0);settings.setdefault('gains',{'Modem AGC':0});p.write_text(json.dumps(cfg,indent=2))
log=(out/'candidate-server.log').open('w');proc=subprocess.Popen(['stdbuf','-oL','-eL','sdrpp','--server','--addr','127.0.0.1','--port','5260','--root',str(config)],stdout=log,stderr=log);s=None;rows=[]
def recvn(n):
 b=b''
 while len(b)<n:
  c=s.recv(n-len(b))
  if not c:raise RuntimeError('connection closed')
  b+=c
 return b
def packet():
 typ,size=struct.unpack('<II',recvn(8));assert 8<=size<=20000000
 b=recvn(size-8);cmd=struct.unpack('<I',b[:4])[0] if typ in (0,1) else None
 return typ,cmd,b[4:] if cmd is not None else b
def send(cmd,b=b''):s.sendall(struct.pack('<III',0,12+len(b),cmd)+b)
def ack(cmd):
 until=time.monotonic()+10
 while time.monotonic()<until:
  typ,c,b=packet()
  if typ==1 and c==cmd:return b
 raise RuntimeError('no ack')
def elements(b):
 i=0;result=[]
 while i<len(b):
  typ=b[i];i+=1
  if typ==0:v=list(b[i:i+2]);i+=2
  elif typ==1:v=bool(b[i]);i+=1
  elif typ in (2,3):v=struct.unpack('<i' if typ==2 else '<f',b[i:i+4])[0];i+=4
  elif typ==4:n=struct.unpack('<H',b[i:i+2])[0];i+=2;v=b[i:i+n].decode('latin1');i+=n
  else:raise RuntimeError(('bad type',typ,i))
  result.append((typ,v))
 return result
def action(name,index):
 b=name.encode();send(1,b'\1'+struct.pack('<BH',4,len(b))+b+struct.pack('<Bi',2,index));return elements(ack(1))
def selection(ui,name):
 i=next(i for i,(t,v) in enumerate(ui) if t==4 and v==name);return ui[i+1][1]
try:
 for _ in range(120):
  assert proc.poll() is None,'server exited'
  try:s=socket.create_connection(('127.0.0.1',5260),.2);break
  except OSError:time.sleep(.1)
 else:raise RuntimeError('startup timeout')
 s.settimeout(4);assert packet()[:2]==(0,128)
 send(0);ui=elements(ack(0));assert selection(ui,'##_bw_select_Soapy Source')==0
 send(6,b'\0');send(7,b'\0');send(4,struct.pack('<d',430100000));ack(4)
 for index,rate,seconds in [(0,4000000,10),(1,2000000,3),(0,4000000,5)]:
  ui=action('##_sr_select_Soapy Source',index);assert selection(ui,'##_bw_select_Soapy Source')==0
  send(2);start=time.monotonic();first=None;prev=None;gaps=[];samples=packets=0
  while time.monotonic()-start<seconds:
   typ,c,b=packet();now=time.monotonic()
   if typ!=2:continue
   version,sampletype,scaler=struct.unpack('<HHf',b[:8]);assert sampletype==0
   if first is None:first=now
   else:samples+=(len(b)-8)//2;gaps.append(now-prev)
   prev=now;packets+=1
  elapsed=prev-first;observed=samples/elapsed
  send(3);send(0);ui=elements(ack(0));assert selection(ui,'##_bw_select_Soapy Source')==0
  row={'rate_hz':rate,'auto_selected':True,'samples':samples,'measurement_seconds':elapsed,'observed_samples_per_second':observed,'packets':packets,'maximum_packet_gap_seconds':max(gaps),'passed':abs(observed/rate-1)<.03 and max(gaps)<.5}
  rows.append(row);print(json.dumps(row),flush=True);assert row['passed']
 # The source logs its actual chosen bandwidth for every Auto start.
 text=(out/'candidate-server.log').read_text();matches=re.findall(r'Bandwidth for samplerate ([0-9.]+) is ([0-9.]+)',text)
 assert len(matches)>=3 and all(float(b)==2000000 for r,b in matches[-3:]),matches
 (out/'server-test.json').write_text(json.dumps({'passed':True,'rows':rows,'auto_bandwidth_log_results':matches},indent=2)+'\n')
finally:
 if s:
  try:send(3);send(0);ack(0)
  except Exception:pass
  s.close()
 proc.terminate()
 try:proc.wait(timeout=4)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 log.close();print('TEST_SERVER_EXIT',proc.returncode,flush=True)
