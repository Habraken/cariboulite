import os,sys,json,pathlib,subprocess,time,shlex,select,statistics,signal
helper=pathlib.Path('/tmp/cariboulite-diag-radio-helper.py').read_text()
run='cariboulite-tx-stability-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
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
        env['CARIBOULITE_CAPTURE_SO']='/tmp/cariboulite_capture_read.so /tmp/pcm_probe.so'
        env['PROBE_LOG']=str(out/case/'write-times.txt')
        env['PROBE_PCM']=str(out/case/'playback.s16')
        env['CARIBOULITE_CAPTURE_SAMPLES']='32000000'
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
    vals=[p['rx']['sinad']['db'] for p in ps if p['rx']['sinad']['db'] is not None];noise=[p['rx']['noise']['rms'] for p in ps if p['rx']['noise']['rms'] is not None]
    summary={'case':case,'label':label,'seconds':seconds,'packets':len(ps),
             'sinad_mean':statistics.mean(vals) if vals else None,'sinad_min':min(vals) if vals else None,'sinad_max':max(vals) if vals else None,
             'below_10_fraction':sum(v<10 for v in vals)/len(vals) if vals else None,
             'noise_mean':statistics.mean(noise) if noise else None,'noise_max':max(noise) if noise else None,
             'rf_measured_hz':(ps[-1]['rx']['reader']['samples']-ps[0]['rx']['reader']['samples'])/(ps[-1]['monotonic_s']-ps[0]['monotonic_s']),
             'pcm_measured_hz':(ps[-1]['rx']['playback']['frames_written']-ps[0]['rx']['playback']['frames_written'])/(ps[-1]['monotonic_s']-ps[0]['monotonic_s']),
             'config':ps[-1]['config'],'tx_config':ts[-1]['config'],
             'tx_measured_hz':(ts[-1]['tx']['writer']['samples']-ts[0]['tx']['writer']['samples'])/(ts[-1]['monotonic_s']-ts[0]['monotonic_s']),'last_rx':ps[-1]['rx'],'last_tx':ts[-1]['tx']}
    results.append(summary);(out/'results.json').write_text(json.dumps(results,indent=2))
    print('RESULT '+json.dumps({k:v for k,v in summary.items() if k not in ('last_rx','last_tx')}),flush=True)

try:
    for divider,rate in [(1,4000000),(10,400000)]:
        case='tx_div'+format(divider,'X')
        launch('rx',case,True);rpc('rx',op='rx_rate',hz=4000000)
        launch('tx',case);rpc('tx',op='tx_rate',hz=rate,dbm=0)
        record(case,'settled',3);rpc('rx',op='capture');record(case,'verified',60)
        shutdown('tx');shutdown('rx');print('CASE_COMPLETE '+case,flush=True)
    print('COMPLETE '+str(out),flush=True)
finally:
    for role in ('tx','rx'):
        try:shutdown(role)
        except Exception as exc:print('CLEANUP '+role+' '+str(exc),flush=True)
    for f in stderr_files:f.close()
    print('RELEASED '+run,flush=True)
