import os,sys,json,pathlib,subprocess,time,shlex,select,statistics,csv,signal
helper=pathlib.Path('/tmp/cariboulite-diag-radio-helper.py').read_text()
run='cariboulite-fixed-tx-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
out=pathlib.Path('/tmp')/run
out.mkdir(mode=0o755)
print('ARTIFACTS '+str(out),flush=True)
children={};errors={};request_count=0

def receive(p,timeout=25):
    if not select.select([p.stdout],[],[],timeout)[0]: raise RuntimeError('Helper reply timeout')
    raw=p.stdout.readline()
    if not raw: raise RuntimeError('Helper exited '+str(p.poll()))
    obj=json.loads(raw)
    if not obj.get('ok'): raise RuntimeError(str(obj))
    return obj

def rpc(role,**request):
    p=children[role];p.stdin.write(json.dumps(request)+'\n');p.stdin.flush()
    return receive(p)

def stats(values):
    return {'mean':statistics.mean(values),'min':min(values),'max':max(values),'stdev':statistics.stdev(values) if len(values)>1 else 0,'n':len(values)} if values else None

rows=[]
try:
    for role in ('rx','tx'):
        errors[role]=(out/(role+'-helper-stderr.log')).open('w')
        argv=['python3','-u','-c',helper,role,run]
        if role=='tx':
            remote=shlex.join(['sudo','-n',*argv])
            argv=['sudo','-u','pi','ssh','-F','/dev/null','-o','BatchMode=yes','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3',
                  '-o','UserKnownHostsFile=/tmp/cariboulite-power-sweep-hosts','pi@pi4g-trixie',remote]
        p=subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=errors[role],text=True,bufsize=1)
        children[role]=p
        ready=receive(p)
        (out/(role+'-ready.json')).write_text(json.dumps(ready,indent=2))
        print('READY '+role+' '+ready['metadata']['host'],flush=True)
    metadata={'frequency_hz':430100000,'channel':'RF24','tx_sample_rate_hz':4000000,
              'tx_power_dbm':0,'tone_hz':1000,'tone_amplitude':.4,'tx_deviation_hz':2500,
              'sequence':[[1000000,60],[2000000,35],[4000000,35],[1000000,60],[4000000,35],[2000000,35],[1000000,60]]}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    rpc('tx',op='power',dbm=0)
    rawfile=(out/'points.jsonl').open('w')
    for number,(rate,duration) in enumerate(metadata['sequence']):
        rpc('rx',op='rx_rate',hz=rate)
        print('PHASE '+json.dumps({'number':number,'rx_rate_hz':rate,'seconds':duration}),flush=True)
        time.sleep(3)
        packets=[];txpackets=[];lastseq=None
        start=time.monotonic();nextpoll=start
        while time.monotonic()-start<duration:
            nextpoll+=.2
            packet=rpc('rx',op='status',wait=.02)['snapshot']
            tx=rpc('tx',op='status',wait=.01)['snapshot']
            if packet and packet['monotonic_s']>=start and packet['sequence']!=lastseq:
                lastseq=packet['sequence'];packets.append(packet);txpackets.append(tx)
                rawfile.write(json.dumps({'phase':number,'rx_rate_hz':rate,'rx':packet,'tx':tx})+'\n');rawfile.flush()
            if not tx or tx['state']!='TX' or tx['control']['transition'] or tx['tx'].get('fault'):raise RuntimeError('TX not steady')
            if packet['state']!='RX':raise RuntimeError('RX not running')
            delay=nextpoll-time.monotonic()
            if delay>0:time.sleep(delay)
        vals=[p['rx']['sinad']['db'] for p in packets if p['rx']['sinad']['db'] is not None]
        noise=[p['rx']['noise']['rms'] for p in packets]
        summary={'phase':number,'rx_rate_hz':rate,'seconds':duration,'packets':len(packets),
                 'sinad_db':stats(vals),'sinad_below_10_fraction':sum(v<10 for v in vals)/len(vals) if vals else None,
                 'noise_rms':stats(noise),'rx_last':packets[-1]['rx'],'tx_last':txpackets[-1]['tx']}
        rows.append(summary)
        (out/'results.json').write_text(json.dumps(rows,indent=2))
        print('RESULT '+json.dumps({k:v for k,v in summary.items() if k not in ('rx_last','tx_last')}),flush=True)
    rawfile.close()
    print('COMPLETE '+str(out),flush=True)
finally:
    # Stop TX first; remote helper also cleans up on input EOF / SSH closure.
    for role in ('tx','rx'):
        p=children.get(role)
        if p and p.poll() is None:
            try: rpc(role,op='shutdown')
            except Exception as exc: print('CLEANUP '+role+' '+str(exc),flush=True)
            try:p.stdin.close();p.wait(timeout=20)
            except Exception:
                p.terminate()
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
    for f in errors.values():f.close()
    print('RELEASED '+run,flush=True)
