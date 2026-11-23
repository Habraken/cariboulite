"""Independently verify snapshots and retain both hosts' logs when sweep ends."""
import collections,csv,gzip,hashlib,json,math,pathlib,statistics,subprocess,sys,tarfile,time,traceback
HERE=pathlib.Path(__file__).resolve().parent
OUT=pathlib.Path((HERE/'latest-run.txt').read_text().strip());RUN=OUT.name
while True:
    try:p=json.loads((OUT/'progress.json').read_text())
    except (FileNotFoundError,json.JSONDecodeError):time.sleep(10);continue
    if p['state'] in ('complete','complete_with_failures','failed','interrupted'):break
    time.sleep(20)
# Allow collector's final hash write and log closure to finish.
time.sleep(2)
verification={'state':p['state'],'planned':p['planned'],'completed':p['completed'],'errors':[],'raw_paired_snapshots':0,'receiver_tx_commands':0,'role_logs_collected':{}}
key='/Users/janhabraken/.ssh/id_ed25519_pi0a_trixie'
for role,ip in [('tx','192.168.6.69'),('rx','192.168.6.37')]:
    path=OUT/(role+'-host-artifacts.tar.gz')
    with path.open('wb') as f:
        result=subprocess.run(['ssh','-i',key,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=10','pi@'+ip,'tar','-czf','-','-C','/home/pi/src/cariboulite/build/pi0-sweeps',RUN],stdout=f,stderr=subprocess.PIPE)
    verification['role_logs_collected'][role]=result.returncode==0
    if result.returncode:verification['errors'].append(role+' archive: '+result.stderr.decode())
    else:
        with tarfile.open(path) as archive:
            for member in archive.getmembers():
                if not member.isfile():continue
                if member.name.endswith('/commands.jsonl'):
                    for line in archive.extractfile(member):
                        cmd=json.loads(line)['request'].split()[2:]
                        if role=='rx' and cmd and cmd[0].startswith('tx_'):
                            verification['receiver_tx_commands']+=1
                            verification['errors'].append('Receiver TX command: '+' '.join(cmd))
                if member.name.endswith('/metadata.json'):
                    meta=json.load(archive.extractfile(member))
                    expected={'rx':('pi0a-trixie','c132d0e7-d7dd-457a-b105-4a5cd97e7e80'),'tx':('pi0b-trixie','36b67ac4-b313-4369-99bd-4b3e09caabfa')}[role]
                    if (meta['host'],meta['board_uuid'])!=expected:verification['errors'].append('Board role mismatch '+role)
for role,ip in [('tx','192.168.6.69'),('rx','192.168.6.37')]:
    result=subprocess.run(['ssh','-i',key,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=10','pi@'+ip,'ps -eo pid,args'],capture_output=True,text=True)
    (OUT/(role+'-final-processes.txt')).write_text(result.stdout+result.stderr)
    owners=[line for line in result.stdout.splitlines() if any(name in line for name in ('/cariboulite_radio_backend','/cariboulite_test_app'))]
    if result.returncode or owners:verification['errors'].append(role+' final release check: '+str(owners)+' '+result.stderr)
rows=[json.loads(line) for line in (OUT/'results.jsonl').read_text().splitlines()]
bykey={(r['phase'],r['point']):r for r in rows};values=collections.defaultdict(lambda:{'rssi':[],'sinad':[],'count':0})
with gzip.open(OUT/'paired-snapshots.jsonl.gz','rt') as f:
    for line in f:
        d=json.loads(line);r=d['rx'];t=d['tx'];v=values[(d['phase'],d['point'])];v['count']+=1;verification['raw_paired_snapshots']+=1
        for ok,label in [(r['state']=='RX','RX state'),(t['state']=='TX','TX state'),(not r['tx']['running'],'RX TX running'),(t['config']['tx_power_dbm']==0,'Power'),(r['config']['requested_frequency_hz']==d['frequency_hz']==t['config']['tx_frequency_hz'],'Frequency'),(r['config']['sample_rate_hz']==d['sample_rate_hz']==t['config']['tx_sample_rate_hz'],'Sample rate')]:
            if not ok:verification['errors'].append(label+' mismatch at '+str(d['point']))
        x=r['hardware']['rf24']['registers'].get('RSSI')
        if isinstance(x,int) and x!=127:v['rssi'].append(x-256 if x>127 else x)
        x=r['rx']['sinad'].get('db')
        if isinstance(x,(int,float)) and math.isfinite(x):v['sinad'].append(x)
for key,row in bykey.items():
    v=values[key]
    if v['count']!=row['snapshots']:verification['errors'].append('Snapshot count '+str(key))
    for field,name in [('rssi_dbm','rssi'),('sinad_db','sinad')]:
        if bool(row[field])!=bool(v[name]):verification['errors'].append('Availability '+str(key)+field)
        elif v[name] and abs(statistics.mean(v[name])-row[field]['mean'])>1e-9:verification['errors'].append('Mean '+str(key)+field)
verification['results_verified']=len(rows)
verification['passed']=not verification['errors']
(OUT/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
matrix=[r for r in rows if r['phase']=='matrix']; rates=sorted({r['sample_rate_hz'] for r in matrix},reverse=True)
lines=['Pi Zero / repaired device #2 TX conducted NBFM frequency and sample-rate sweep',
       'Run: '+RUN,'State: '+p['state'],
       'TX: pi0b-trixie, repaired device #2; RX only: pi0a-trixie, original reference.',
       'Nominal link attenuation: 90 dB; requested TX power: 0 dBm (uncalibrated).',
       'Signal: 1 kHz tone, amplitude 0.4; NBFM deviation setting 2500 Hz.',
       'Matched TX/RX sample rates; 2 s settling, 5 s measurement per point.',
       'Frequency grid: 172 points, 30 MHz to 5999.999 MHz; 6 GHz is exclusive.',
       'SINAD is the backend estimate over 300-3000 Hz; unavailable values remain blank.',
       'Transport-clean flags mean no counter increases/resets within the measurement window.',
       'Measurements describe the complete TX / attenuator / RX link, not calibrated receiver sensitivity.',
       '',f"Completed matrix points: {len(matrix)} / {p['planned']}",
       'Independent raw-snapshot verification: '+('PASS' if verification['passed'] else 'FAIL'),
       'Receiver TX commands found: '+str(verification['receiver_tx_commands']),
       '', 'Rate Hz | points | points with numeric SINAD | transport flagged | RSSI mean range dBm | SINAD mean range dB']
for rate in rates:
    rr=[r for r in matrix if r['sample_rate_hz']==rate];rss=[r['rssi_dbm']['mean'] for r in rr if r['rssi_dbm']];sn=[r['sinad_db']['mean'] for r in rr if r['sinad_db']]
    fmt=lambda a:f'{min(a):.2f} to {max(a):.2f}' if a else 'unavailable'
    lines.append(f"{rate} | {len(rr)} | {len(sn)} | {sum(not r['transport_clean'] for r in rr)} | {fmt(rss)} | {fmt(sn)}")
if p['failures']:lines.extend(['','Failed points:',json.dumps(p['failures'],indent=2)])
if verification['errors']:lines.extend(['','Verification errors:',json.dumps(verification['errors'],indent=2)])
(OUT/'REPORT.txt').write_text('\n'.join(lines)+'\n')
(OUT/'SHA256SUMS').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+f.name+'\n' for f in sorted(OUT.iterdir()) if f.is_file() and f.name!='SHA256SUMS'))
print('\n'.join(lines),flush=True)
