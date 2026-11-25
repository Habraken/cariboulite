from pathlib import Path
import json,subprocess,shutil,os,signal,time,hashlib
out=Path('/home/pi/src/cariboulite/build/soapy-rate-fix-20261125')
assert json.loads((out/'mock-tests.json').read_text())['passed']
installed=Path('/usr/local/lib/SoapySDR/modules0.8-3/libSoapyCariboulite.so')
backup=out/'backup';backup.mkdir(exist_ok=True)
if not (backup/installed.name).exists():shutil.copy2(installed,backup/installed.name)
assert hashlib.sha256(installed.read_bytes()).hexdigest()==json.loads((out/'build.json').read_text())['original_installed_sha256']
assert Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode()=='36b67ac4-b313-4369-99bd-4b3e09caabfa'
servers=[]
for p in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  args=p.read_bytes().split(b'\0')
  if args and Path(os.fsdecode(args[0])).name=='sdrpp':
   assert args[:2]==[b'sdrpp',b'--server'],args
   servers.append({'pid':int(p.parent.name),'args':[os.fsdecode(a) for a in args if a]})
 except (FileNotFoundError,ProcessLookupError):pass
(out/'previous-servers.json').write_text(json.dumps(servers,indent=2)+'\n')
for row in servers:
 try:os.kill(row['pid'],signal.SIGTERM)
 except ProcessLookupError:pass
for row in servers:
 pid=row['pid'];p=Path('/proc',str(pid),'cmdline');until=time.monotonic()+5
 while p.exists() and p.read_bytes() and time.monotonic()<until:time.sleep(.1)
 row['forced_kill']=bool(p.exists() and p.read_bytes())
 if row['forced_kill']:os.kill(pid,signal.SIGKILL)
(out/'previous-servers.json').write_text(json.dumps(servers,indent=2)+'\n')
print('PRIOR_SERVER_STOPPED',json.dumps(servers),flush=True)
for name,path in [('before',installed),('candidate',out/'libSoapyCariboulite.so')]:
 print('CHECK_START',name,flush=True)
 with (out/(name+'-rates.log')).open('w') as f:
  proc=subprocess.Popen([str(out/'check-rates'),str(path)],stdout=f,stderr=f)
  try:code=proc.wait(timeout=60)
  except subprocess.TimeoutExpired:proc.kill();proc.wait();raise
 rows=[json.loads(x.split('RATE_RESULT ',1)[1]) for x in (out/(name+'-rates.log')).read_text().splitlines() if x.startswith('RATE_RESULT ')]
 result={'exit_code':code,'rates':rows};(out/(name+'-rates.json')).write_text(json.dumps(result,indent=2)+'\n')
 print(name,json.dumps(result),flush=True)
 assert len(rows)==8,(name,code)
 if name=='candidate':assert code==0 and all(r['passed'] for r in rows)
