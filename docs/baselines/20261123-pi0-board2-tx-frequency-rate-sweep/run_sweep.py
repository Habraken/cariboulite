"""Conducted NBFM sweep: repaired device #2 TX only, original reference RX only."""
import csv,gzip,hashlib,json,math,os,pathlib,select,signal,statistics,subprocess,sys,time,traceback
HERE=pathlib.Path(__file__).resolve().parent
RUN='pi0-board2-tx-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
OUT=HERE/RUN; OUT.mkdir()
(HERE/'latest-run.txt').write_text(str(OUT)+'\n')
HOSTS={'tx':'192.168.6.69','rx':'192.168.6.37'}
RATES=[4000000,2000000,1333333,1000000,800000,666667,500000,400000]
KEY='/Users/janhabraken/.ssh/id_ed25519_pi0a_trixie'
plan=json.loads((HERE/'draft-plan.json').read_text());plan.update(status='authorized',tx_power_dbm=0,attenuation_db_nominal=90,power_levels_dbm=[0],run=RUN,settle_seconds=2,measure_seconds=5)
plan['points']=[dict(point=i+1,frequency_hz=f['frequency_hz'],sample_rate_hz=rate,frequency_reason=f['reason'],bands=f['bands']) for i,(rate,f) in enumerate((r,f) for r in RATES for f in plan['frequencies'])]
(OUT/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
for name in ['radio_helper.py','run_sweep.py']:
    (OUT/name).write_bytes((HERE/name).read_bytes())
children={};logs=[];results=[];failures=[];interrupted=False
raw=gzip.open(OUT/'paired-snapshots.jsonl.gz','wt',compresslevel=1)
fields=['phase','point','frequency_hz','sample_rate_hz','tx_power_dbm','snapshots','rssi_mean_dbm','rssi_min_dbm','rssi_max_dbm','sinad_mean_db','sinad_min_db','sinad_max_db','sinad_valid_fraction','transport_clean','rx_new_errors','tx_new_errors']
csvfile=(OUT/'results.csv').open('w',newline='');writer=csv.DictWriter(csvfile,fieldnames=fields);writer.writeheader();csvfile.flush()
jsonrows=(OUT/'results.jsonl').open('w')
def log(s):print(time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())+' '+s,flush=True)
def receive(p,timeout=35):
    if not select.select([p.stdout],[],[],timeout)[0]:raise RuntimeError('Helper response timeout')
    line=p.stdout.readline()
    if not line:raise RuntimeError('Helper exited '+str(p.poll()))
    obj=json.loads(line)
    if not obj.get('ok'):raise RuntimeError(str(obj))
    return obj

def rpc(role,**req):
    p=children[role];p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();return receive(p)

def launch(role,tag):
    remote_run=RUN+'/'+tag
    args=['ssh','-i',KEY,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','pi@'+HOSTS[role],'python3','-u','/tmp/cl-pi0-board2-sweep-helper.py',role,remote_run]
    f=(OUT/(role+'-'+tag+'-helper.log')).open('w');logs.append(f)
    p=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=f,text=True,bufsize=1);children[role]=p
    ready=receive(p);(OUT/(role+'-'+tag+'-ready.json')).write_text(json.dumps(ready,indent=2)+'\n')
    log('READY '+role+' '+tag)

def shutdown(role):
    p=children.pop(role,None)
    if p is None:return
    if p.poll() is None:
        try:
            p.stdin.write('{"op":"shutdown"}\n');p.stdin.flush();receive(p,20)
        except Exception as e:log('SHUTDOWN_WARNING '+role+' '+str(e))
        finally:
            try:p.stdin.close()
            except Exception:pass
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:p.kill();p.wait()

def stop_all():
    for role in ('tx','rx'):
        try:shutdown(role)
        except Exception as e:log('CLEANUP_ERROR '+role+' '+str(e))

def stats(v):
    return dict(mean=statistics.mean(v),median=statistics.median(v),min=min(v),max=max(v),stdev=statistics.stdev(v) if len(v)>1 else 0,n=len(v)) if v else None

def check(r,t,point):
    hz=point['frequency_hz'];rate=point['sample_rate_hz']
    assert r['state']=='RX' and t['state']=='TX',(r['state'],t['state'])
    assert not t['tx'].get('fault'),t['tx'].get('fault')
    assert not r['tx']['running'],'Receiver TX unexpectedly running'
    assert r['config']['requested_frequency_hz']==hz and t['config']['tx_frequency_hz']==hz,'Frequency mismatch'
    assert r['config']['sample_rate_hz']==rate and t['config']['tx_sample_rate_hz']==rate,'Rate mismatch'
    assert t['config']['tx_power_dbm']==0 and t['config']['tx_actual_power_dbm']==0,'TX power mismatch'
    for s in (r,t):
        assert s['config']['ctcss']['rx_frequency_hz']==0 and s['config']['ctcss']['tx_frequency_hz']==0
        assert s['config']['cdcss']['rx_code']=='off' and s['config']['cdcss']['tx_code']=='off'
    assert t['config']['tx_tone_hz']==1000
    assert t['config']['tx_tone_amp']==.4 or abs(t['config']['tx_tone_amp']-.4)<1e-6

ERRORS={'rx':[('queue','drops'),('audio_queue','drops'),('playback','underruns'),('playback','write_errors'),('reader','errors'),('alignment','failures'),('alignment','invalid_frames')], 'tx':[('driver','underrun_periods'),('writer','errors'),('queue','drops')]}
def counters(snaps,role):
    values={}
    for a,b in ERRORS[role]:
        vals=[p[role][a][b] for p in snaps]
        values[a+'.'+b]=dict(first=vals[0],last=vals[-1],min=min(vals),max=max(vals),delta=vals[-1]-vals[0],reset=any(y<x for x,y in zip(vals,vals[1:])))
    return values

def measure(point,phase):
    rpc('tx',op='off')
    rpc('rx',op='configure',hz=point['frequency_hz'],rate=point['sample_rate_hz'])
    rpc('tx',op='configure',hz=point['frequency_hz'],rate=point['sample_rate_hz'])
    until=time.monotonic()+2
    while True:
        r=rpc('rx',op='status',wait=.01)['snapshot'];t=rpc('tx',op='status',wait=.01)['snapshot'];check(r,t,point)
        if time.monotonic()>=until:break
        time.sleep(.08)
    rseq=r['sequence'];tseq=t['sequence'];rgate=r['monotonic_s'];tgate=t['monotonic_s']
    rs=[];ts=[];pairs=[];start=time.monotonic();nextpoll=start
    while time.monotonic()-start<5:
        nextpoll+=.2
        r=rpc('rx',op='status',wait=.005)['snapshot'];t=rpc('tx',op='status',wait=.005)['snapshot'];check(r,t,point)
        if r['sequence']>rseq and t['sequence']>tseq and r['monotonic_s']>rgate and t['monotonic_s']>tgate:
            rseq=r['sequence'];tseq=t['sequence'];rs.append(r);ts.append(t)
            pairs.append(dict(phase=phase,**point,rx=r,tx=t))
        time.sleep(max(0,nextpoll-time.monotonic()))
    assert len(rs)>=15,'Too few fresh paired snapshots: '+str(len(rs))
    rss=[];sinad=[]
    for r in rs:
        v=r['hardware']['rf24']['registers'].get('RSSI')
        if isinstance(v,int) and v!=127:rss.append(v-256 if v>127 else v)
        v=r['rx']['sinad'].get('db')
        if isinstance(v,(float,int)) and math.isfinite(v):sinad.append(v)
    assert len(rss)>=15,'RSSI unavailable'
    re=counters(rs,'rx');te=counters(ts,'tx')
    clean=not any(v['delta'] or v['reset'] for v in [*re.values(),*te.values()])
    row=dict(phase=phase,**point,tx_power_dbm=0,snapshots=len(rs),rssi_dbm=stats(rss),sinad_db=stats(sinad),sinad_valid_fraction=len(sinad)/len(rs),transport_clean=clean,rx_errors=re,tx_errors=te,rx_config=rs[-1]['config'],tx_config=ts[-1]['config'],rx_first_monotonic_s=rs[0]['monotonic_s'],rx_last_monotonic_s=rs[-1]['monotonic_s'])
    flat=dict(phase=phase,point=point['point'],frequency_hz=point['frequency_hz'],sample_rate_hz=point['sample_rate_hz'],tx_power_dbm=0,snapshots=len(rs),rssi_mean_dbm=row['rssi_dbm']['mean'],rssi_min_dbm=min(rss),rssi_max_dbm=max(rss),sinad_mean_db=statistics.mean(sinad) if sinad else None,sinad_min_db=min(sinad) if sinad else None,sinad_max_db=max(sinad) if sinad else None,sinad_valid_fraction=row['sinad_valid_fraction'],transport_clean=clean,rx_new_errors=sum(max(0,v['delta']) for v in re.values()),tx_new_errors=sum(max(0,v['delta']) for v in te.values()))
    for pair in pairs:raw.write(json.dumps(pair,separators=(',',':'))+'\n')
    raw.flush()
    writer.writerow(flat);csvfile.flush();jsonrows.write(json.dumps(row)+'\n');jsonrows.flush();results.append(row)
    log('RESULT '+json.dumps(flat,separators=(',',':')))
    return row

def progress(state):
    matrix=[r for r in results if r['phase']=='matrix']
    (OUT/'progress.json').write_text(json.dumps(dict(run=RUN,state=state,completed=len(matrix),planned=len(plan['points']),preflight_completed=sum(r['phase']=='preflight' for r in results),failures=failures,updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),transport_flagged=sum(not r['transport_clean'] for r in matrix)),indent=2)+'\n')

def interrupted_handler(sig,frame):raise KeyboardInterrupt('Signal '+str(sig))
for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,interrupted_handler)
log('ARTIFACTS '+str(OUT));state='starting'
try:
    launch('tx','session1');launch('rx','session1')
    for i,rate in enumerate(RATES):
        measure(dict(point=-(i+1),frequency_hz=430100000,sample_rate_hz=rate,frequency_reason='preflight',bands=['amateur-70cm']),'preflight');progress('preflight')
    # Stop before full acquisition if every rate fails to yield a measurable tone.
    assert all(r['sinad_db'] and r['sinad_valid_fraction']>=.9 and r['transport_clean'] for r in results),'Preflight not clean and usable at every sample rate; stopping before full sweep'
    log('PREFLIGHT_COMPLETE');state='matrix';consecutive=0
    for point in plan['points']:
        for attempt in (1,2):
            try:
                if not children:
                    tag='resume'+str(point['point'])+'-'+str(attempt);launch('tx',tag);launch('rx',tag)
                measure(point,'matrix');break
            except Exception as e:
                failures.append(dict(**point,attempt=attempt,error=str(e)));log('POINT_FAILURE '+json.dumps(failures[-1]));stop_all()
                if attempt==2:raise RuntimeError('Point failed twice; stopped without skipping it')
        progress('matrix')
        if point['point']%25==0:
            for role in ('rx','tx'):
                if role in children:
                    with (OUT/'telemetry.jsonl').open('a') as f:f.write(json.dumps(dict(point=point['point'],role=role,**rpc(role,op='telemetry')))+'\n')
    state='complete' if not failures else 'complete_with_failures'
except BaseException as e:
    state='interrupted' if isinstance(e,KeyboardInterrupt) else 'failed'
    (OUT/'exception.txt').write_text(traceback.format_exc());log('ABORT '+repr(e))
finally:
    stop_all();raw.close();csvfile.close();jsonrows.close()
    for f in logs:f.close()
    progress(state);log('RELEASED '+state)
    (OUT/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='SHA256SUMS'))
if state not in ('complete','complete_with_failures'):sys.exit(1)
