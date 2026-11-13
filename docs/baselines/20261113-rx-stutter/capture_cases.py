import os,sys,json,pathlib,subprocess,time,shlex,select,statistics,signal
helper=pathlib.Path('/tmp/cariboulite-diag-radio-helper.py').read_text()
run='cariboulite-capture-cases-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
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
    if capture:env['CARIBOULITE_CAPTURE_SO']='/tmp/cariboulite_capture_read.so'
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

def record(case,label,seconds):
    ps=[];ts=[];seq=None;start=time.monotonic();nextpoll=start
    with (out/case/(label+'.jsonl')).open('w') as f:
        while time.monotonic()-start<seconds:
            nextpoll+=.2
            p=rpc('rx',op='status',wait=.02)['snapshot'];t=rpc('tx',op='status',wait=.01)['snapshot']
            if p and p['monotonic_s']>=start and p['sequence']!=seq:
                seq=p['sequence'];ps.append(p);ts.append(t);f.write(json.dumps({'rx':p,'tx':t})+'\n');f.flush()
            if not t or t['state']!='TX' or t['tx'].get('fault'):raise RuntimeError('TX fault')
            if p['state']!='RX':raise RuntimeError('RX state '+p['state'])
            delay=nextpoll-time.monotonic()
            if delay>0:time.sleep(delay)
    vals=[p['rx']['sinad']['db'] for p in ps if p['rx']['sinad']['db'] is not None];noise=[p['rx']['noise']['rms'] for p in ps]
    summary={'case':case,'label':label,'seconds':seconds,'packets':len(ps),
             'sinad_mean':statistics.mean(vals) if vals else None,'sinad_min':min(vals) if vals else None,'sinad_max':max(vals) if vals else None,
             'below_10_fraction':sum(v<10 for v in vals)/len(vals) if vals else None,
             'noise_mean':statistics.mean(noise),'noise_max':max(noise),'last_rx':ps[-1]['rx'],'last_tx':ts[-1]['tx']}
    results.append(summary);(out/'results.json').write_text(json.dumps(results,indent=2))
    print('RESULT '+json.dumps({k:v for k,v in summary.items() if k not in ('last_rx','last_tx')}),flush=True)

try:
    launch('tx','source');rpc('tx',op='power',dbm=0)
    cases=[('fresh_1m_a',1000000,None),('warm_4_to_1',1000000,'warm'),('fresh_1m_b',1000000,None),('soft_reset_1m',1000000,'fpga_soft_reset'),('hard_program_1m',1000000,'fpga_hard_reset'),('fresh_2m',2000000,None),('fresh_4m',4000000,None)]
    (out/'metadata.json').write_text(json.dumps({'tx_host':'pi4g-trixie','rx_host':'pi4f-trixie','frequency_hz':430100000,'tx_rate_hz':4000000,'tx_power_dbm':0,'tone_hz':1000,'amplitude':.4,'deviation_hz':2500,'cases':cases,'capture_samples':8000000},indent=2))
    for case,rate,action in cases:
        launch('rx',case,True)
        telem={role:rpc(role,op='telemetry')['telemetry'] for role in ('rx','tx')}
        (out/case/'host-before.json').write_text(json.dumps(telem,indent=2))
        if action=='warm':rpc('rx',op='rx_rate',hz=4000000);record(case,'warmup_4m',8)
        elif action:
            rpc('rx',op='rx_reset',kind=action)
            if action=='fpga_hard_reset':rpc('rx',op='rx_reset',kind='fpga_program')
        rpc('rx',op='rx_rate',hz=rate)
        record(case,'before_capture',12)
        armed=rpc('rx',op='capture');(out/case/'arm.json').write_text(json.dumps(armed,indent=2))
        record(case,'capture',10 if rate==1000000 else 6)
        telem={role:rpc(role,op='telemetry')['telemetry'] for role in ('rx','tx')}
        (out/case/'host-after.json').write_text(json.dumps(telem,indent=2))
        shutdown('rx')
        print('CASE_COMPLETE '+case,flush=True)
    print('COMPLETE '+str(out),flush=True)
finally:
    for role in ('tx','rx'):
        try:shutdown(role)
        except Exception as exc:print('CLEANUP '+role+' '+str(exc),flush=True)
    for f in stderr_files:f.close()
    print('RELEASED '+run,flush=True)
