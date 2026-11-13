import os,sys,json,pathlib,subprocess,time,shlex,select,statistics,csv,signal
helper=pathlib.Path('/tmp/cariboulite-power-radio-helper.py').read_text()
run='cariboulite-power-sweep-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
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
    metadata={'frequency_hz':430100000,'channel':'RF24','attenuation_db_nominal':90,'tx_sample_rate_hz':4000000,
              'tone_hz':1000,'tone_amplitude':.4,'tx_deviation_hz':2500,'settle_seconds':2,'measure_seconds':6,
              'rx_rates_hz':[1000000,2000000,4000000],'tx_powers_dbm':list(range(-12,15,2))}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    rawfile=(out/'points.jsonl').open('w')
    for rate in metadata['rx_rates_hz']:
        rpc('tx',op='off');rpc('rx',op='rx_rate',hz=rate)
        for power in [None,*metadata['tx_powers_dbm']]:
            print('POINT '+json.dumps({'rx_rate':rate,'tx_power':power}),flush=True)
            if power is not None: rpc('tx',op='power',dbm=power)
            time.sleep(2)
            # Drain stale receiver snapshots after the settling interval.
            rpc('rx',op='status',wait=.05)
            packets=[];txpackets=[];lastseq=None
            start=time.monotonic(); nextpoll=start
            while time.monotonic()-start<6:
                nextpoll+=.2
                packet=rpc('rx',op='status',wait=.02)['snapshot']
                tx=rpc('tx',op='status',wait=.01)['snapshot']
                if packet and packet['sequence']!=lastseq:
                    lastseq=packet['sequence'];packets.append(packet);txpackets.append(tx)
                    rawfile.write(json.dumps({'rx_rate_hz':rate,'tx_power_dbm':power,'rx':packet,'tx':tx})+'\n');rawfile.flush()
                if power is not None and (not tx or tx['state']!='TX' or tx['control']['transition'] or tx['tx'].get('fault')):
                    raise RuntimeError('TX not steady during measurement')
                if packet['state']!='RX':raise RuntimeError('Receiver left RX')
                delay=nextpoll-time.monotonic()
                if delay>0:time.sleep(delay)
            rss=[];sinad=[];noise=[]
            for p in packets:
                v=p['hardware']['rf24']['registers'].get('RSSI')
                if isinstance(v,int) and v!=127:rss.append(v-256 if v>127 else v)
                v=p['rx']['sinad'].get('db')
                if isinstance(v,(int,float)):sinad.append(v)
                v=p['rx']['noise'].get('rms')
                if isinstance(v,(int,float)):noise.append(v)
            txlast=txpackets[-1]
            row={'rx_rate_hz':rate,'tx_power_dbm':power,'nominal_rx_input_dbm':None if power is None else power-90,
                 'packets':len(packets),'rssi_dbm':stats(rss),'sinad_db':stats(sinad),'noise_rms':stats(noise),
                 'sinad_valid_fraction':len(sinad)/len(packets),'gate_open_fraction':sum(p['rx']['gate_open'] is True for p in packets)/len(packets),
                 'tx_actual_power_dbm':txlast['config']['tx_actual_power_dbm'],'tx_actual_frequency_hz':txlast['config']['tx_actual_frequency_hz'],
                 'tx_driver_underruns_max':max((p['tx'].get('driver',{}).get('underrun_periods') or 0) for p in txpackets),
                 'rx_errors':{},'tx_snapshot':txlast}
            for name,path,key in [('iq_drops','queue','drops'),('audio_drops','audio_queue','drops'),('alsa_underruns','playback','underruns'),('write_errors','playback','write_errors'),('read_errors','reader','errors')]:
                values=[p['rx'][path][key] for p in packets]
                row['rx_errors'][name]={'start':values[0],'end':values[-1],'delta':values[-1]-values[0]}
            rows.append(row)
            (out/'results.json').write_text(json.dumps(rows,indent=2))
            brief={k:v for k,v in row.items() if k not in ('tx_snapshot','rx_errors')}
            print('RESULT '+json.dumps(brief),flush=True)
        rpc('tx',op='off')
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
