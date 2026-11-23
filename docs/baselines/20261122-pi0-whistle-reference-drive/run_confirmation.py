from pathlib import Path
import json,subprocess,select,time,sys,traceback
HERE=Path(__file__).resolve().parent
KEY='/Users/janhabraken/.ssh/id_ed25519_pi0a_trixie'
HOSTS={'tx':'192.168.6.37','rx':'192.168.6.69'}
children={};files=[];results=[]
base_cases=[
 dict(name='carrier',tone_amp=0,tx_ctcss=0,rx_ctcss=0,offset=0,rate=1000000),
 dict(name='ctcss_only_rx_off',tone_amp=0,tx_ctcss=88.5,rx_ctcss=0,offset=0,rate=1000000),
 dict(name='ctcss_only_rx_on',tone_amp=0,tx_ctcss=88.5,rx_ctcss=88.5,offset=0,rate=1000000),
 dict(name='tone_1k',tone_amp=.4,tx_ctcss=0,rx_ctcss=0,offset=0,rate=1000000),
 dict(name='tone_1k_ctcss',tone_amp=.4,tx_ctcss=88.5,rx_ctcss=88.5,offset=0,rate=1000000),
 dict(name='carrier_rx_plus5k',tone_amp=0,tx_ctcss=0,rx_ctcss=0,offset=5000,rate=1000000),
 dict(name='carrier_rx_minus5k',tone_amp=0,tx_ctcss=0,rx_ctcss=0,offset=-5000,rate=1000000),
 dict(name='carrier_2m',tone_amp=0,tx_ctcss=0,rx_ctcss=0,offset=0,rate=2000000),
 dict(name='no_tx',tone_amp=0,tx_ctcss=0,rx_ctcss=0,offset=0,rate=1000000,no_tx=True),
]
cases=[dict(base_cases[0],name=block+'__carrier',base_case='carrier',drive_ma=ma,block=block,confirmation=True)
       for block,ma in [('e8',8),('f2',2),('g4',4),('h8',8)]]
(HERE/'confirmation-plan.json').write_text(json.dumps({'channel':'RF24','rf_hz':430100000,'tx_power_dbm':0,'attenuation_db_nominal':90,'capture_seconds_pcm':6,'capture_seconds_iq':4,'cases':cases},indent=2))
def log(s):print(time.strftime('%H:%M:%S',time.gmtime())+' '+s,flush=True)
def receive(p,timeout=25):
 if not select.select([p.stdout],[],[],timeout)[0]:raise RuntimeError('helper timeout')
 line=p.stdout.readline()
 if not line:raise RuntimeError('helper exited '+str(p.poll()))
 data=json.loads(line)
 if not data.get('ok'):raise RuntimeError(data)
 return data

def launch(role,case,rate=1000000,drive_ma=8):
 err=(HERE/(role+'-'+case+'-helper.log')).open('w');files.append(err)
 args=['ssh','-i',KEY,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2','pi@'+HOSTS[role],'python3','-u','/tmp/whistle-drive-tools/radio_helper.py',role,case,str(rate*4),str(drive_ma)]
 p=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,bufsize=1);children[role]=p
 data=receive(p);(HERE/(role+'-'+case+'-ready.json')).write_text(json.dumps(data,indent=2));return data

def rpc(role,op,**kw):
 p=children[role];p.stdin.write(json.dumps(dict(op=op,**kw))+'\n');p.stdin.flush();return receive(p)
def cmd(role,text):return rpc(role,'command',text=text)
def stop(role):
 p=children.pop(role,None)
 if p is None:return
 if p.poll() is None:
  try:
   p.stdin.write('{"op":"shutdown"}\n');p.stdin.flush();receive(p)
  except Exception as e:log('shutdown '+str(e))
  finally:
   p.stdin.close()
   try:p.wait(timeout=20)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=20)

def heartbeat(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  for role in children:rpc(role,'status',wait=min(.3,max(0,end-time.monotonic())))
  time.sleep(.1)

try:
 launch('tx','confirmation-tx-session');cmd('tx','tx_tone 1000 0');cmd('tx','tx_ctcss_deviation 200')
 for case in cases:
  name=case['name'];log('START '+name)
  cmd('tx','tx_stop');rpc('tx','state',state='IDLE')
  cmd('tx','tx_tone 1000 '+str(case['tone_amp']));cmd('tx','tx_ctcss '+str(case['tx_ctcss']))
  launch('rx',name,case['rate'],case['drive_ma']);rpc('tx','status')
  cmd('rx','frequency '+str(430100000+case['offset']));cmd('rx','sample_rate '+str(case['rate']))
  cmd('rx','rx_ctcss '+str(case['rx_ctcss']));cmd('rx','start');rpc('rx','state',state='RX')
  if not case.get('no_tx'):
   cmd('tx','tx_start');rpc('tx','state',state='TX')
  heartbeat(3)
  rx_verify=rpc('rx','verify',mode=2)
  tx_verify=rpc('tx','verify',mode=0 if case.get('no_tx') else 4)
  before={role:rpc(role,'status')['snapshot'] for role in children}
  for role,s in before.items():
   assert s['config']['tx_power_dbm']==0
   assert s['control']['state']==('RX' if role=='rx' else 'IDLE' if case.get('no_tx') else 'TX')
  rpc('rx','capture');heartbeat(6.5)
  after={role:rpc(role,'status')['snapshot'] for role in children}
  row=dict(case=case,before=before,after=after,verification={'rx':rx_verify,'tx':tx_verify});results.append(row)
  (HERE/'confirmation-measurements.json').write_text(json.dumps(results,indent=2))
  stop('rx');cmd('tx','tx_stop');rpc('tx','state',state='IDLE')
  log('DONE '+name+' rssi='+str(after['rx']['hardware']['rf24']['registers']['RSSI'])+' sinad='+str(after['rx']['rx']['sinad']['db']))
except Exception:
 traceback.print_exc();raise
finally:
 stop('tx');stop('rx')
 for f in files:f.close()
 log('RADIOS STOPPED')
