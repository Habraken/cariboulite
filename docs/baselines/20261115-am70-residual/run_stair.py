#!/usr/bin/env python3
"""Run on pi4f after deployment and confirmed 70 dB RF24 link; owns both radios."""
import json,os,pathlib,select,subprocess,sys,time
root=pathlib.Path('/home/pi/src/cariboulite')
helper='/tmp/am-tx-link-helper.py'
kind='direct'
run='am70-stair-'+time.strftime('%Y%m%d-%H%M%S')
out=pathlib.Path('/tmp')/run;out.mkdir()
clients={};records=[]
log=(out/'orchestrator.log').open('w')

def receive(p,timeout=50):
 if not select.select([p.stdout],[],[],timeout)[0]:raise RuntimeError('Helper timeout')
 line=p.stdout.readline()
 if not line:raise RuntimeError('Helper disconnected')
 value=json.loads(line)
 if not value.get('ok'):raise RuntimeError(value)
 return value

def rpc(role,**request):
 p=clients[role];p.stdin.write(json.dumps(request)+'\n');p.stdin.flush();return receive(p)
def wire(role,text,state=None):return rpc(role,op='wire',command=text,state=state)
def emit(value):print(json.dumps(value),flush=True)
def counters(s,role):
 if role=='rx':
  return {name:s['rx'][group][key] for name,group,key in [
   ('iq_drops','queue','drops'),('audio_drops','audio_queue','drops'),('read_errors','reader','errors'),
   ('alignment_failures','alignment','failures'),('invalid_iq','alignment','invalid_frames'),
   ('audio_underruns','playback','underruns'),('audio_write_errors','playback','write_errors')]}
 return {'writer_errors':s['tx']['writer']['errors'],'underruns':s['tx']['driver']['underrun_periods']}
def capture(name):
 before_rx=rpc('rx',op='status',wait=.8)['snapshot'];before_tx=rpc('tx',op='status')['snapshot']
 assert before_tx['tx']['driver']['valid'] and before_tx['tx']['hardware_paced']
 result=rpc('rx',op='capture',name=name,seconds=3)
 after_tx=rpc('tx',op='status')['snapshot'];after_rx=result['snapshot']
 deltas={}
 for role,a,b in [('rx',before_rx,after_rx),('tx',before_tx,after_tx)]:
  ca,cb=counters(a,role),counters(b,role)
  deltas.update({role+'_'+k:cb[k]-ca[k] for k in ca})
 if any(deltas.values()):raise RuntimeError({'case':name,'error_deltas':deltas})
 assert after_rx['config']['am_offset_tuning_enabled']
 assert abs(after_rx['config']['rx_frequency_hz']-after_rx['config']['requested_frequency_hz'])<1000
 durations={'rx':after_rx['monotonic_s']-before_rx['monotonic_s'],'tx':after_tx['monotonic_s']-before_tx['monotonic_s']}
 assert min(durations.values())>=3,durations
 raw_rssi=after_rx['hardware']['rf24']['registers']['RSSI']
 row={'case':name,'wav':result['wav'],'tx_rate_hz':after_tx['config']['tx_sample_rate_hz'],
      'rx_rate_hz':after_rx['config']['sample_rate_hz'],'tx_power_setting_dbm':after_tx['config']['tx_power_dbm'],
      'tx_am_depth':after_tx['config']['tx_am_depth'],'tx_am_backoff_db':after_tx['config']['tx_am_backoff_db'],'tx_tone_amplitude':after_tx['config']['tx_tone_amp'],
      'rssi_dbm':raw_rssi if raw_rssi<128 else raw_rssi-256,
      'audio_output':after_rx['rx']['audio_output'],'error_counter_deltas':deltas,'counter_window_s':durations,
      'tx_driver':after_tx['tx']['driver'],'rx_snapshot':after_rx,'tx_snapshot':after_tx}
 records.append(row);(out/'results.json').write_text(json.dumps(records,indent=2)+'\n');emit({'case':name,'rssi_dbm':row['rssi_dbm'],'rms_dbfs':row['audio_output']['rms_dbfs'],'errors':deltas})
try:
 clients['rx']=subprocess.Popen(['python3',helper,'rx',run,'hw:Loopback,0,0','0','/tmp/cariboulite_am70_iq_backend'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1,env=dict(os.environ,AM_IQ_PATH='/tmp/am70-raw-iq.bin'))
 rxmeta=receive(clients['rx'])
 clients['tx']=subprocess.Popen(['ssh','-F','/dev/null','-o','BatchMode=yes','-o','UserKnownHostsFile=/tmp/cariboulite-power-sweep-hosts','pi@pi4g-trixie','python3',helper,'tx',run,'hw:Loopback,0,0','0','/tmp/cariboulite_am70_offset_backend' if kind=='offset' else '/tmp/cariboulite_am70_stair_backend'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
 txmeta=receive(clients['tx']);(out/'metadata.json').write_text(json.dumps({'rx':rxmeta,'tx':txmeta},indent=2)+'\n')
 emit({'ready':True,'run':run,'output':str(out)})
 wire('rx','frequency 430100000');wire('tx','tx_frequency '+str(430125200 if kind=='offset' else 430100000))
 for power,backoff in [(6,0)]:
  wire('tx','tx_power '+str(power));wire('tx','tx_am_backoff '+str(backoff))
  for amplitude in [.03,.03]:
   wire('tx','tx_tone 1000 '+str(amplitude));wire('tx','tx_start','TX')
   wire('rx','start','RX');rpc('rx',op='status',wait=2.5)
   raw=pathlib.Path('/tmp/am70-raw-iq.bin');assert raw.stat().st_size==8000000
   raw.rename(out/('repeat_'+str(len(records))+'.iq16'))
   capture('power_'+str(power)+'_backoff_'+str(backoff)+'_amp_'+str(amplitude).replace('.','p'))
   wire('rx','stop','IDLE');wire('tx','tx_stop','IDLE')
 emit({'ok':True,'complete':True,'cases':len(records),'run':run,'output':str(out)})
finally:
 for role in ('tx','rx'):
  p=clients.get(role)
  if p and p.poll() is None:
   try:rpc(role,op='shutdown')
   except Exception as exc:emit({'cleanup':role,'error':str(exc)})
   p.stdin.close()
   try:p.wait(timeout=25)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
 log.close()
