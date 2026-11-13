import os,sys,json,pathlib,subprocess,time,shlex,select,statistics,signal
helper=pathlib.Path('/tmp/cariboulite-diag-radio-helper.py').read_text()
run='cariboulite-rx-buffered-rates-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
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
             'config':ps[-1]['config'],'last_rx':ps[-1]['rx'],'last_tx':ts[-1]['tx']}
    results.append(summary);(out/'results.json').write_text(json.dumps(results,indent=2))
    print('RESULT '+json.dumps({k:v for k,v in summary.items() if k not in ('last_rx','last_tx')}),flush=True)

try:
    launch('tx','source');rpc('tx',op='power',dbm=0)
    rates=[(1,4000000),(2,2000000),(3,1333333),(4,1000000),(5,800000),(6,666667),(8,500000),(10,400000)]
    (out/'metadata.json').write_text(json.dumps({'tx_host':'pi4g-trixie','rx_host':'pi4f-trixie','frequency_hz':430100000,'tx_rate_hz':4000000,'nbfm_tx_power_dbm':0,'nbfm_tx_tone_amplitude':.4,'wbfm_tx_power_dbm':6,'wbfm_tx_tone_amplitude':1.0,'tx_deviation_hz':2500,'rates':rates,'capture_samples':1000000},indent=2))
    for mode in ['nbfm','wbfm']:
        if mode=='wbfm':rpc('tx',op='tone',amplitude=1.0);rpc('tx',op='power',dbm=6)
        for divider,rate in rates:
            case=mode+'_div'+format(divider,'X');launch('rx',case,mode=='wbfm')
            telem={role:rpc(role,op='telemetry')['telemetry'] for role in ('rx','tx')}
            (out/case/'host-before.json').write_text(json.dumps(telem,indent=2))
            rpc('rx',op='mode',mode=mode);rpc('rx',op='rx_rate',hz=rate)
            if mode=='nbfm':
                record(case,'verified',20)
                r=results[-1];assert r['sinad_min']>15 and r['sinad_max']-r['sinad_min']<3,r
            else:
                record(case,'settled',6);(out/case/'arm.json').write_text(json.dumps(rpc('rx',op='capture'),indent=2));record(case,'verified',4)
                assert results[-1]['sinad_mean'] is None
            r=results[-1];a=r['last_rx']['alignment']
            assert a['acquisitions']>=1 and a['failures']==a['invalid_frames']==0,a
            assert abs(r['rf_measured_hz']/(4000000/divider)-1)<.03,r
            assert abs(r['pcm_measured_hz']/48000-1)<.04,r
            rx=r['last_rx'];assert rx['queue']['drops']==rx['audio_queue']['drops']==rx['playback']['underruns']==rx['reader']['errors']==0,rx
            assert r['last_tx']['driver']['underrun_periods']==0,r['last_tx']
            telem={role:rpc(role,op='telemetry')['telemetry'] for role in ('rx','tx')}
            (out/case/'host-after.json').write_text(json.dumps(telem,indent=2))
            shutdown('rx');print('CASE_COMPLETE '+case,flush=True)
    print('COMPLETE '+str(out),flush=True)
finally:
    for role in ('tx','rx'):
        try:shutdown(role)
        except Exception as exc:print('CLEANUP '+role+' '+str(exc),flush=True)
    for f in stderr_files:f.close()
    print('RELEASED '+run,flush=True)
