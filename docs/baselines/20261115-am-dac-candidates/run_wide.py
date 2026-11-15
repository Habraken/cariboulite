import json,pathlib,select,subprocess,sys,time
root=pathlib.Path('/home/pi/src/cariboulite');helper='/tmp/am-candidate/helper.py'
variant='wide'
iq_center=37500
run='am-candidates-meter-'+variant+'-'+time.strftime('%Y%m%d-%H%M%S');out=pathlib.Path('/tmp')/run;out.mkdir();clients={};records=[];log=(out/'orchestrator.log').open('w')
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
 before={r:rpc(r,op='status')['snapshot'] for r in ('rx','tx')};iq=rpc('rx',op='iq',name=name,if_center=iq_center);after={r:rpc(r,op='status')['snapshot'] for r in ('rx','tx')}
 deltas={r+'_'+k:counters(after[r],r)[k]-v for r in before for k,v in counters(before[r],r).items()};assert not any(deltas.values()),deltas;assert iq['metrics']['adc_rail_samples']==0
 row={'case':name,'parameters':parameters,'metrics':iq['metrics'],'iq':iq['iq'],'error_counter_deltas':deltas,'tx_config':after['tx']['config'],'rx_config':after['rx']['config'],'tx_registers':after['tx']['hardware']['rf24']['registers']};records.append(row);(out/'results.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps({'case':name,'metrics':iq['metrics']}),flush=True);return row
try:
 clients['rx']=subprocess.Popen(['python3',helper,'rx',run,'hw:Loopback,0,0','21' if variant=='gain42' else '0','/tmp/cariboulite_am_candidate_rx_wide_backend'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1);rxmeta=receive(clients['rx'])
 clients['tx']=subprocess.Popen(['ssh','-F','/dev/null','-o','BatchMode=yes','-o','UserKnownHostsFile=/tmp/cariboulite-power-sweep-hosts','pi@pi4g-trixie','python3',helper,'tx',run,'hw:Loopback,0,0','0','/tmp/cariboulite_am_candidate_backend'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1);txmeta=receive(clients['tx']);(out/'metadata.json').write_text(json.dumps({'rx':rxmeta,'tx':txmeta},indent=2)+'\n');print(json.dumps({'run':run,'ready':True}),flush=True)
 wire('rx','sample_rate 4000000');wire('rx','start','RX');wire('tx','tx_power 6');wire('tx','tx_am_backoff 0')
 calibrated=json.loads(pathlib.Path('/tmp/am-candidate/calibration.json').read_text())['parameters']
 for mode in (0,1,6):
  wire('tx','tx_frequency '+str(430125200 if mode==1 else 430100000))
  for rate in (1000000,):
   wire('tx','tx_sample_rate '+str(rate))
   for amplitude in (.03,.1):
    wire('tx','tx_tone 1000 '+str(amplitude));wire('tx','tx_start','TX')
    params=dict(calibrated) if mode==1 else {'mode':mode,'ci':calibrated['ci'],'cq':calibrated['cq'],'offset':40000}
    rpc('tx',op='diag',**params);take('mode_'+str(mode)+'_rate_'+str(rate)+'_amp_'+str(amplitude).replace('.','p'),params);wire('tx','tx_stop','IDLE')
 print(json.dumps({'complete':True,'cases':len(records),'run':run}),flush=True)
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
