import os,sys,json,pathlib,subprocess,time,shlex,select,statistics,signal
helper=pathlib.Path('/tmp/cariboulite-diag-radio-helper.py').read_text()
run='cariboulite-native-power-sweep-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
out=pathlib.Path('/tmp')/run;out.mkdir(mode=0o755)
print('ARTIFACTS '+str(out),flush=True)
children={};stderr_files=[];results=[]
def receive(p,timeout=30):
    if not select.select([p.stdout],[],[],timeout)[0]:raise RuntimeError('Helper timeout')
    line=p.stdout.readline()
    if not line:raise RuntimeError('Helper exited '+str(p.poll()))
    obj=json.loads(line)
    if not obj.get('ok'):raise RuntimeError(str(obj))
    return obj

def rpc(role,**req):
    p=children[role];p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();return receive(p)

def launch(role,case,capture=False):
    args=['python3','-u','-c',helper,role,run+'/'+case]
    env=dict(os.environ)
    if role=='tx':args=['sudo','-u','pi','ssh','-F','/dev/null','-o','BatchMode=yes','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','-o','UserKnownHostsFile=/tmp/cariboulite-power-sweep-hosts','pi@pi4g-trixie',shlex.join(['sudo','-n',*args])]
    if capture:
        env['CARIBOULITE_CAPTURE_SO']='/tmp/cariboulite_capture_read.so'
        env['CARIBOULITE_CAPTURE_SAMPLES']='1000000'
    path=out/case;path.mkdir(exist_ok=True)
    log=(path/(role+'-helper.log')).open('w');stderr_files.append(log)
    p=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1,env=env);children[role]=p
    ready=receive(p);(path/(role+'-ready.json')).write_text(json.dumps(ready,indent=2));print('READY '+role+' '+case,flush=True)

def shutdown(role):
    p=children.get(role)
    if p and p.poll() is None:
        try:rpc(role,op='shutdown')
        finally:
            p.stdin.close()
            try:p.wait(timeout=60)
            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
    children.pop(role,None)

def stats(values):
    return {'mean':statistics.mean(values),'median':statistics.median(values),'min':min(values),'max':max(values),'stdev':statistics.stdev(values) if len(values)>1 else 0,'n':len(values)} if values else None

def measure(axis,rxrate,txrate,power):
    # Drain status throughout settling; measure only fresh RX timestamps.
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        rpc('rx',op='status',wait=.04);rpc('tx',op='status',wait=.02);time.sleep(.1)
    start=time.monotonic();nextpoll=start;ps=[];ts=[];seq=None;point=len(results)+1
    while time.monotonic()-start<5:
        nextpoll+=.2
        p=rpc('rx',op='status',wait=.02)['snapshot'];t=rpc('tx',op='status',wait=.01)['snapshot']
        if p and p['monotonic_s']>=start and p['sequence']!=seq:
            seq=p['sequence'];ps.append(p);ts.append(t)
            rawfile.write(json.dumps({'point':point,'axis':axis,'rx_rate_hz':rxrate,'tx_rate_hz':txrate,'tx_power_dbm':power,'rx':p,'tx':t})+'\n');rawfile.flush()
        if p['state']!='RX':raise RuntimeError('Receiver left RX')
        if not t or t['state']!=('TX' if power is not None else 'IDLE') or t['tx'].get('fault'):raise RuntimeError('Transmitter not steady')
        delay=nextpoll-time.monotonic()
        if delay>0:time.sleep(delay)
    assert len(ps)>=20,len(ps)
    rss=[];sinad=[];noise=[]
    for p in ps:
        v=p['hardware']['rf24']['registers'].get('RSSI')
        if isinstance(v,int) and v!=127:rss.append(v-256 if v>127 else v)
        v=p['rx']['sinad']['db']
        if isinstance(v,(int,float)):sinad.append(v)
        v=p['rx']['noise']['rms']
        if isinstance(v,(int,float)):noise.append(v)
    assert len(rss)>=20,(axis,rxrate,txrate,power,len(rss))
    errors={}
    for name,path,key in [('iq_drops','queue','drops'),('audio_drops','audio_queue','drops'),('alsa_underruns','playback','underruns'),('playback_errors','playback','write_errors'),('read_errors','reader','errors'),('alignment_failures','alignment','failures'),('invalid_frames','alignment','invalid_frames')]:
        vals=[p['rx'][path][key] for p in ps];errors[name]={'first':vals[0],'last':vals[-1],'max':max(vals),'delta':vals[-1]-vals[0]}
    txerrors={}
    if power is not None:
        for name,path,key in [('driver_underruns','driver','underrun_periods'),('writer_errors','writer','errors'),('queue_drops','queue','drops')]:
            vals=[p['tx'][path][key] for p in ts];txerrors[name]=max(vals)
    assert all(v['max']==0 for v in errors.values()),errors
    assert all(v==0 for v in txerrors.values()),txerrors
    pac=ts[-1]['hardware']['rf24']['registers'].get('PAC')
    row={'point':point,'axis':axis,'rx_rate_hz':rxrate,'tx_rate_hz':txrate,
         'rx_divider':dict((hz,d) for d,hz in rates)[rxrate],'tx_divider':dict((hz,d) for d,hz in rates)[txrate],
         'tx_power_dbm':power,'tx_pa_code':(pac&31) if power is not None and isinstance(pac,int) else None,
         'tx_model_power_dbm':ts[-1]['config']['tx_actual_power_dbm'] if power is not None else None,
         'nominal_input_dbm':power-90 if power is not None else None,'packets':len(ps),
         'rssi_dbm':stats(rss),'sinad_db':stats(sinad),'noise_rms':stats(noise),
         'sinad_valid_fraction':len(sinad)/len(ps),'gate_open_fraction':sum(p['rx']['gate_open'] is True for p in ps)/len(ps),
         'rx_errors':errors,'tx_errors':txerrors,
         'first_rx_monotonic_s':ps[0]['monotonic_s'],'last_rx_monotonic_s':ps[-1]['monotonic_s'],
         'rx_config':ps[-1]['config'],'tx_config':ts[-1]['config']}
    results.append(row);(out/'results.json').write_text(json.dumps(results,indent=2))
    print('RESULT '+json.dumps({k:v for k,v in row.items() if k not in ('rx_config','tx_config','rx_errors')}),flush=True)

rates=[(1,4000000),(2,2000000),(3,1333333),(4,1000000),(5,800000),(6,666667),(8,500000),(10,400000)]
powers=list(range(-12,15,2));rawfile=(out/'points.jsonl').open('w')
(out/'metadata.json').write_text(json.dumps({'source_commit':'1e4948c','driver_srcversion':'5AD373CAE4DBC118FE6E425','tx_host':'pi4g-trixie','rx_host':'pi4f-trixie','frequency_hz':430100000,'channel':'RF24','attenuation_db_nominal':90,'tone_hz':1000,'tone_amplitude':.4,'tx_deviation_hz':2500,'settle_seconds':2,'measure_seconds':5,'rates':rates,'tx_powers_dbm':powers,'axes':['rx','tx'],'timestamp_filter':'RX snapshots must be newer than measurement start; status drained throughout settling','shared_reference':'4M/4M measured independently in both axes'},indent=2))
try:
    launch('tx','rx_axis_source');rpc('tx',op='tx_rate',hz=4000000,dbm=-12);rpc('tx',op='off')
    for divider,rate in rates:
        case='rx_div'+format(divider,'X');launch('rx',case);rpc('rx',op='rx_rate',hz=rate)
        measure('rx',rate,4000000,None)
        for power in powers:
            rpc('tx',op='power',dbm=power);measure('rx',rate,4000000,power)
        rpc('tx',op='off');shutdown('rx');print('RATE_COMPLETE rx '+str(rate),flush=True)
    shutdown('tx')
    launch('rx','tx_axis_receiver');rpc('rx',op='rx_rate',hz=4000000)
    for divider,rate in rates:
        case='tx_div'+format(divider,'X');launch('tx',case)
        # Configure each native TX rate before taking its TX-off baseline.
        rpc('tx',op='tx_rate',hz=rate,dbm=-12);rpc('tx',op='off')
        measure('tx',4000000,rate,None)
        for power in powers:
            rpc('tx',op='power',dbm=power);measure('tx',4000000,rate,power)
        shutdown('tx');print('RATE_COMPLETE tx '+str(rate),flush=True)
    print('COMPLETE '+str(out),flush=True)
finally:
    for role in ('tx','rx'):
        try:shutdown(role)
        except Exception as exc:print('CLEANUP '+role+' '+str(exc),flush=True)
    rawfile.close()
    for f in stderr_files:f.close()
    print('RELEASED '+run,flush=True)
