"""Run one confirmed conducted-link direction, then release both radios."""
from pathlib import Path
import json,select,subprocess,sys,time,traceback
HERE=Path(__file__).resolve().parent
KEY='/Users/janhabraken/.ssh/id_ed25519_pi0a_trixie'
HOSTS={'a':'192.168.6.37','b':'192.168.6.69'}
txnode,rxnode=sys.argv[1:3];assert {txnode,rxnode}=={'a','b'}
phase=txnode+'tx_'+rxnode+'rx';run=phase+'_'+time.strftime('%H%M%S',time.gmtime())
out=HERE/run;out.mkdir();children={};errors=[];rows=[];sessions=[]
offsets='--offsets' in sys.argv[3:]
cases=[('offset_p5k',0,True,5000),('offset_m5k',0,True,-5000)] if offsets else [('noise',0,False,0),('carrier',0,True,0),('tone_a',.4,True,0),('tone_b',.4,True,0),('carrier_end',0,True,0)]
if_control=int(sys.argv[sys.argv.index('--if-control')+1]) if '--if-control' in sys.argv else None
assert if_control is None or if_control in (2385000000,2495000000)
if if_control is not None:cases=[('carrier_probe',0,True,0),('tone_probe',.4,True,0)]
plan={'run':run,'phase':phase,'tx_node':txnode,'rx_node':rxnode,'frequency_hz':430100000,'rate_hz':1000000,
      'tx_requested_dbm':0,'attenuation_db_nominal':90,'capture_seconds':8,'settle_seconds':3,'tx_if_control_hz':if_control,
      'cases':[{'name':n,'amplitude':a,'tx_on':on,'rx_offset_hz':offset} for n,a,on,offset in cases]}
(out/'plan.json').write_text(json.dumps(plan,indent=2))
def log(text):print(time.strftime('%H:%M:%S',time.gmtime())+' '+text,flush=True)
def receive(p,timeout=35):
 if not select.select([p.stdout],[],[],timeout)[0]:raise RuntimeError('helper response timeout')
 line=p.stdout.readline()
 if not line:raise RuntimeError('helper exited '+str(p.poll()))
 d=json.loads(line)
 if not d.get('ok'):raise RuntimeError(str(d))
 return d
def launch(role,node,case,program=False):
 f=(out/(role+'-'+case+'.log')).open('w');errors.append(f)
 remote=['sudo','-n','python3','-u','/home/pi/src/cariboulite/build/demod-audio-20261125/radio_helper.py',role,node,case]
 if program:remote.append('--program')
 if role=='tx' and if_control==2385000000:remote+=['--tx-if',str(if_control)]
 args=['ssh','-i',KEY,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2','pi@'+HOSTS[node],*remote]
 p=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=f,text=True,bufsize=1);children[role]=p
 d=receive(p);sessions.append({'role':role,'node':node,'case':case,'ready':d});(out/'sessions.json').write_text(json.dumps(sessions,indent=2));return d
def rpc(role,op,**kw):
 p=children[role];p.stdin.write(json.dumps(dict(op=op,**kw))+'\n');p.stdin.flush();return receive(p)
def cmd(role,text):return rpc(role,'command',text=text)
def stop(role):
 p=children.pop(role,None)
 if p is None:return
 if p.poll() is None:
  try:p.stdin.write('{"op":"shutdown"}\n');p.stdin.flush();receive(p)
  except Exception as e:log('SHUTDOWN_WARNING '+role+' '+str(e))
  finally:
   p.stdin.close()
   try:p.wait(timeout=20)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=20)
 assert p.returncode==0,(role,p.returncode)
def heartbeat(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  for role in list(children):rpc(role,'status',wait=min(.15,max(.01,until-time.monotonic())))
  time.sleep(.05)
def txstart():
 attempts=[]
 for attempt in (1,2):
  try:
   cmd('tx','tx_start');rpc('tx','state',state='TX');return attempts
  except Exception as e:
   attempts.append({'attempt':attempt,'error':str(e)});log('TX_START_RETRY '+str(e))
   if attempt==2:raise
   cmd('tx','stop');rpc('tx','state',state='IDLE')
   cmd('tx','fpga_program');rpc('tx','state',state='IDLE')
try:
 launch('tx',txnode,run+'_txsession',True)
 cmd('tx','tx_source tone')
 for index,(name,amplitude,on,offset) in enumerate(cases):
  case=run+'_'+name;log('START '+case)
  cmd('tx','tx_stop');rpc('tx','state',state='IDLE');cmd('tx','tx_tone 1000 '+str(amplitude))
  launch('rx',rxnode,case,index==0)
  if offset:cmd('rx','frequency '+str(430100000+offset))
  cmd('rx','start');rpc('rx','state',state='RX')
  attempts=txstart() if on else []
  heartbeat(3)
  before={r:rpc(r,'status')['snapshot'] for r in children}
  assert before['rx']['state']=='RX' and before['tx']['state']==('TX' if on else 'IDLE')
  rpc('rx','capture');heartbeat(8.6)
  after={r:rpc(r,'status')['snapshot'] for r in children}
  row={'case':case,'name':name,'amplitude':amplitude,'tx_on':on,'rx_offset_hz':offset,'before':before,'after':after,'start_retries':attempts}
  rows.append(row);(out/'measurements.json').write_text(json.dumps(rows,indent=2))
  stop('rx');cmd('tx','tx_stop');rpc('tx','state',state='IDLE')
  log('DONE '+case+' SINAD='+str(after['rx']['rx']['sinad']['db'])+' RSSI='+str(after['rx']['hardware']['rf24']['registers']['RSSI']))
except Exception:
 traceback.print_exc();raise
finally:
 for role in ('tx','rx'):
  try:stop(role)
  except Exception as e:log('CLEANUP_WARNING '+str(e))
 for f in errors:f.close()
 log('RADIOS RELEASED; '+str(out))
