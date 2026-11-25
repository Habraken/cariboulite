import pathlib,subprocess,shutil,json,socket,struct,time,signal
root=pathlib.Path('/tmp/soapy-rate-investigation-20261125');root.mkdir(exist_ok=True)
config=root/'config';config.mkdir(exist_ok=True)
for p in pathlib.Path('/home/pi/.config/sdrpp').glob('*.json'):shutil.copy2(p,config/p.name)
p=config/'config.json';d=json.loads(p.read_text());d['source']='SoapySDR';p.write_text(json.dumps(d,indent=2))
log=(root/'server.log').open('w')
proc=subprocess.Popen(['sdrpp','--server','--addr','127.0.0.1','--port','5259','--root',str(config)],stdout=log,stderr=log)
rows=[]
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
def send(cmd,data=b''):s.sendall(struct.pack('<III',0,12+len(data),cmd)+data)
def ack(cmd):
 for _ in range(100):
  typ,c,b=packet();rows.append({'type':typ,'cmd':c,'bytes':len(b),'rate':struct.unpack('<d',b)[0] if typ==0 and c==128 else None})
  if typ==1 and c==cmd:return b
 raise RuntimeError('no ack')
def elements(b):
 i=0;out=[]
 while i<len(b):
  t=b[i];i+=1
  if t==0:v=list(b[i:i+2]);i+=2
  elif t==1:v=bool(b[i]);i+=1
  elif t in (2,3):v=struct.unpack('<i' if t==2 else '<f',b[i:i+4])[0];i+=4
  elif t==4:
   n=struct.unpack('<H',b[i:i+2])[0];i+=2;v=b[i:i+n].decode('latin1');i+=n
  else:raise RuntimeError(('type',t,i))
  out.append([t,v])
 return out
def action(name,index):
 b=name.encode();send(1,b'\1'+struct.pack('<BH',4,len(b))+b+struct.pack('<Bi',2,index))
 return elements(ack(1))
try:
 for _ in range(100):
  assert proc.poll() is None,'server exited'
  try:s=socket.create_connection(('127.0.0.1',5259),.2);break
  except OSError:time.sleep(.1)
 else:raise RuntimeError('server startup timeout')
 s.settimeout(12)
 print('HANDSHAKE',packet()[:2],flush=True)
 send(0);raw=ack(0);(root/'ui.bin').write_bytes(raw);ui=elements(raw);(root/'ui.json').write_text(json.dumps(ui,indent=2));print(json.dumps(ui),flush=True)
 ui=action('##_dev_select_Soapy Source',1)
 rates=[v for t,v in ui if t==4 and '_sr_select_' in v];assert len(rates)==1,rates
 for index in range(8):
  start=time.monotonic();ui=action(rates[0],index);print('RATE',index,'ACK_SECONDS',time.monotonic()-start,flush=True)
 send(4,struct.pack('<d',430100000.));ack(4)
 for index in (1,3,0,7,5,2,4,6,1):
  action(rates[0],index);send(2)
  start=time.monotonic();blocks=total=0
  while time.monotonic()-start<1.0:
   typ,c,b=packet()
   if typ==2:blocks+=1;total+=len(b)
  print('RX',index,'blocks',blocks,'bytes',total,flush=True)
  assert blocks>0
  send(3);send(0);ack(0)
  print('STOP_ACK',index,flush=True)
 s.close()
finally:
 (root/'packets.json').write_text(json.dumps(rows,indent=2))
 proc.terminate()
 try:proc.wait(timeout=15)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 log.close();print('SERVER_EXIT',proc.returncode,flush=True)
