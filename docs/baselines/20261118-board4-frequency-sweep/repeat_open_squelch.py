"""Two-Pi coax frequency sweep. RX timestamps compared only within RX host."""
import json,pathlib,subprocess,time,select,statistics,os,sys,signal
HERE=pathlib.Path(__file__).resolve().parent
plan=json.loads((HERE/'plan.json').read_text());run='cariboulite-board4-frequency-open-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
out=pathlib.Path('/tmp')/run;out.mkdir();print('ARTIFACTS '+str(out),flush=True)
(out/'plan.json').write_text(json.dumps(plan,indent=2));children={};logs=[];results=[];failures=[]
(out/'metadata.json').write_text(json.dumps(dict(run=run,rx_host='pi4f-trixie',tx_host='pi4g-trixie',rx_board='#4',attenuation_db_nominal=90,rx_sample_rate_hz=1000000,tx_sample_rate_hz=1000000,tx_power_dbm=0,tone_hz=1000,tone_amplitude=.4,tx_deviation_hz=2500,settle_seconds=2,measure_seconds=5,source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True,cwd='/home/pi/src/cariboulite').strip(),squelch_disabled=True,timestamp_filter='per-host sequence freshness and RX-local monotonic time; clocks never compared across hosts'),indent=2))
raw=(out/'points.jsonl').open('w')
def receive(p):
 if not select.select([p.stdout],[],[],35)[0]:raise RuntimeError('Helper timeout')
 line=p.stdout.readline()
 if not line:raise RuntimeError('Helper exited '+str(p.poll()))
 obj=json.loads(line)
 if not obj.get('ok'):raise RuntimeError(str(obj))
 return obj

def rpc(role,**request):
 p=children[role];p.stdin.write(json.dumps(request)+'\n');p.stdin.flush();return receive(p)

def launch(role,tag):
 args=['sudo','-n','python3','-u','/tmp/board4-frequency-open-helper.py',role,run+'/'+tag]
 if role=='rx':args=['ssh','-F','/dev/null','-o','BatchMode=yes','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','pi@192.168.6.163',*args]
 log=(out/(role+'-'+tag+'-helper.log')).open('w');logs.append(log)
 children[role]=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
 ready=receive(children[role]);(out/(role+'-'+tag+'-ready.json')).write_text(json.dumps(ready,indent=2))
 print('READY '+role+' '+tag,flush=True)

def shutdown(role):
 p=children.get(role)
 if p and p.poll() is None:
  try:rpc(role,op='shutdown')
  except Exception as e:print('SHUTDOWN_WARNING '+role+' '+str(e),flush=True)
  finally:
   p.stdin.close()
   try:p.wait(timeout=25)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
 children.pop(role,None)

def stats(values):
 return dict(mean=statistics.mean(values),median=statistics.median(values),min=min(values),max=max(values),stdev=statistics.stdev(values) if len(values)>1 else 0,n=len(values)) if values else None

def errors(snapshots,role):
 fields=([('queue','drops'),('audio_queue','drops'),('playback','underruns'),('playback','write_errors'),('reader','errors'),('alignment','failures'),('alignment','invalid_frames')] if role=='rx' else [('driver','underrun_periods'),('writer','errors'),('queue','drops')])
 return {a+'.'+b:dict(first=snapshots[0][role][a][b],last=snapshots[-1][role][a][b],max=max(p[role][a][b] for p in snapshots),delta=snapshots[-1][role][a][b]-snapshots[0][role][a][b]) for a,b in fields}

def measure(point):
 hz=point['frequency_hz'];deadline=time.monotonic()+2
 while time.monotonic()<deadline:
  rs=rpc('rx',op='status',wait=.025)['snapshot'];ts=rpc('tx',op='status',wait=.025)['snapshot'];time.sleep(.08)
 rx_gate=rs['monotonic_s'];rx_seq=rs['sequence'];tx_seq=ts['sequence'];samples=[];txsamples=[];start=time.monotonic();nextpoll=start
 while time.monotonic()-start<5:
  nextpoll+=.2
  r=rpc('rx',op='status',wait=.01)['snapshot'];t=rpc('tx',op='status',wait=.01)['snapshot']
  assert r['state']=='RX' and t['state']=='TX' and not t['tx'].get('fault'),(r['state'],t['state'])
  assert r['config']['requested_frequency_hz']==hz and t['config']['tx_frequency_hz']==hz
  assert r['config']['sample_rate_hz']==1000000 and t['config']['tx_sample_rate_hz']==1000000 and t['config']['tx_power_dbm']==0
  if r['sequence']>rx_seq and r['monotonic_s']>rx_gate and t['sequence']>tx_seq:
   rx_seq=r['sequence'];tx_seq=t['sequence'];samples.append(r);txsamples.append(t)
   raw.write(json.dumps(dict(point=point['point'],frequency_hz=hz,rx=r,tx=t))+'\n');raw.flush()
  delay=nextpoll-time.monotonic()
  if delay>0:time.sleep(delay)
 assert len(samples)>=20,len(samples)
 rss=[];sinad=[]
 for r in samples:
  v=r['hardware']['rf24']['registers'].get('RSSI')
  if isinstance(v,int) and v!=127:rss.append(v-256 if v>127 else v)
  v=r['rx']['sinad'].get('db')
  if isinstance(v,(int,float)):sinad.append(v)
 assert len(rss)>=20,'RSSI unavailable'
 re=errors(samples,'rx');te=errors(txsamples,'tx');pac=txsamples[-1]['hardware']['rf24']['registers'].get('PAC')
 row=dict(**point,packets=len(samples),rssi_dbm=stats(rss),sinad_db=stats(sinad),sinad_valid_fraction=len(sinad)/len(samples),rx_errors=re,tx_errors=te,transport_clean=not any(v['max'] for v in [*re.values(),*te.values()]),tx_pa_code=pac&31 if isinstance(pac,int) else None,rx_config=samples[-1]['config'],tx_config=txsamples[-1]['config'],rx_first_monotonic_s=samples[0]['monotonic_s'],rx_last_monotonic_s=samples[-1]['monotonic_s'])
 results.append(row);(out/'results.json').write_text(json.dumps(results,indent=2))
 print('RESULT '+json.dumps(dict(point=row['point'],frequency_hz=hz,rssi_dbm=row['rssi_dbm']['mean'],sinad_db=row['sinad_db']['mean'] if row['sinad_db'] else None,valid_fraction=row['sinad_valid_fraction'],transport_clean=row['transport_clean'])),flush=True)

try:
 launch('tx','session1');launch('rx','session1')
 for point in plan['points']:
  if point['point'] not in json.loads((HERE/'retry-points.json').read_text()):continue
  try:
   if not children:
    tag='resume'+str(point['point']);launch('tx',tag);launch('rx',tag)
   rpc('tx',op='off');rpc('rx',op='tune',hz=point['frequency_hz']);rpc('tx',op='tune',hz=point['frequency_hz']);measure(point)
  except Exception as e:
   failure=dict(**point,error=str(e));failures.append(failure);(out/'failures.json').write_text(json.dumps(failures,indent=2));print('POINT_FAILURE '+json.dumps(failure),flush=True)
   for role in ('tx','rx'):shutdown(role)
 print('COMPLETE '+str(out),flush=True)
finally:
 for role in ('tx','rx'):
  try:shutdown(role)
  except Exception as e:print('CLEANUP_ERROR '+role+' '+str(e),flush=True)
 raw.close()
 for f in logs:f.close()
 print('RELEASED '+run,flush=True)
