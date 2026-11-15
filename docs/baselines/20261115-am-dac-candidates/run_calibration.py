import json,pathlib,select,subprocess,sys,time
root=pathlib.Path('/home/pi/src/cariboulite');helper='/tmp/am-candidate/helper.py'
run='am-candidates-calibration-'+time.strftime('%Y%m%d-%H%M%S');out=pathlib.Path('/tmp')/run;out.mkdir();clients={};records=[];log=(out/'orchestrator.log').open('w')
def receive(p):
 if not select.select([p.stdout],[],[],50)[0]:raise RuntimeError('helper timeout')
 line=p.stdout.readline()
 if not line:raise RuntimeError('helper exited')
 v=json.loads(line)
 if not v.get('ok'):raise RuntimeError(v)
 return v
def rpc(role,**request):
 p=clients[role];p.stdin.write(json.dumps(request)+'\n');p.stdin.flush();return receive(p)
def wire(role,text,state=None):return rpc(role,op='wire',command=text,state=state)
def counters(s,role):
 if role=='rx':return {n:s['rx'][g][k] for n,g,k in [('iq_drops','queue','drops'),('audio_drops','audio_queue','drops'),('reader_errors','reader','errors'),('alignment_failures','alignment','failures'),('invalid_iq','alignment','invalid_frames'),('audio_underruns','playback','underruns'),('audio_write_errors','playback','write_errors')]}
 return {'writer_errors':s['tx']['writer']['errors'],'driver_underruns':s['tx']['driver']['underrun_periods']}
def take(name,parameters):
 before={r:rpc(r,op='status')['snapshot'] for r in ('rx','tx')};iq=rpc('rx',op='iq',name=name);after={r:rpc(r,op='status')['snapshot'] for r in ('rx','tx')}
 deltas={r+'_'+k:counters(after[r],r)[k]-v for r in before for k,v in counters(before[r],r).items()};assert not any(deltas.values()),deltas;assert iq['metrics']['adc_rail_samples']==0
 row={'case':name,'parameters':parameters,'metrics':iq['metrics'],'iq':iq['iq'],'error_counter_deltas':deltas,'tx_config':after['tx']['config'],'rx_config':after['rx']['config'],'tx_registers':after['tx']['hardware']['rf24']['registers']};records.append(row);(out/'results.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps({'case':name,'metrics':iq['metrics']}),flush=True);return row
try:
 clients['rx']=subprocess.Popen(['python3',helper,'rx',run,'hw:Loopback,0,0','0','/tmp/cariboulite_am_candidate_rx_backend'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1);rxmeta=receive(clients['rx'])
 clients['tx']=subprocess.Popen(['ssh','-F','/dev/null','-o','BatchMode=yes','-o','UserKnownHostsFile=/tmp/cariboulite-power-sweep-hosts','pi@pi4g-trixie','python3',helper,'tx',run,'hw:Loopback,0,0','0','/tmp/cariboulite_am_candidate_backend'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1);txmeta=receive(clients['tx']);(out/'metadata.json').write_text(json.dumps({'rx':rxmeta,'tx':txmeta},indent=2)+'\n');print(json.dumps({'run':run,'ready':True}),flush=True)
 wire('rx','start','RX');wire('tx','tx_frequency 430125200');wire('tx','tx_power 6');wire('tx','tx_am_backoff 0');wire('tx','tx_tone 1000 0.1');wire('tx','tx_start','TX')
 params={'mode':1};rpc('tx',op='diag',**params);initial=take('initial',dict(params));regs=initial['tx_registers'];params.update(ci=regs['TXCI']&63,cq=regs['TXCQ']&63,gain=1.,phase=0.,di=0.,dq=0.)
 def evaluate(label):
  rpc('tx',op='diag',**params);return take(label+'_'+str(len(records)),dict(params))
 def coordinate(key,values,offset):
  best=None
  for value in sorted(set(values)):
   params[key]=value;r=evaluate(key);cost=r['metrics']['offset_200hz_band_dbc'][str(offset)]
   if best is None or cost<best[0]:best=(cost,value)
  params[key]=best[1];print(json.dumps({'selected':key,'value':best[1],'cost_dbc':best[0]}),flush=True)
 for span,step in [(8,4),(2,1)]:
  for key in ('ci','cq'):
   center=params[key];coordinate(key,[min(63,max(0,center+d)) for d in range(-span,span+1,step)],25200)
 for span,step in [(64,32),(16,8),(4,2)]:
  for key in ('di','dq'):
   center=params[key];coordinate(key,[center+d for d in range(-span,span+1,step)],25200)
 for span,step in [(.04,.02),(.01,.005),(.002,.001)]:
  for key in ('gain','phase'):
   center=params[key];coordinate(key,[center+k*step for k in (-2,-1,0,1,2)],50400)
 for key in ('di','dq'):
  center=params[key];coordinate(key,[center+d for d in (-4,-2,0,2,4)],25200)
 final=[evaluate('final') for _ in range(3)];cal={'parameters':params,'initial':initial['metrics'],'final':[v['metrics'] for v in final],'run':run};(out/'calibration.json').write_text(json.dumps(cal,indent=2)+'\n');pathlib.Path('/tmp/am-candidate/calibration.json').write_text(json.dumps(cal,indent=2)+'\n');print(json.dumps({'complete':True,'calibration':cal}),flush=True)
finally:
 for role in ('tx','rx'):
  p=clients.get(role)
  if p and p.poll() is None:
   try:rpc(role,op='shutdown')
   except Exception as e:print('cleanup '+str(e),flush=True)
   p.stdin.close()
   try:p.wait(timeout=25)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
 log.close()
