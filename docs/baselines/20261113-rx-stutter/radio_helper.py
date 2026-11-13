import os, sys, json, socket, subprocess, time, signal, pathlib, hashlib, platform

def send(obj):
    print(json.dumps(obj),flush=True)

root=pathlib.Path('/home/pi/src/cariboulite')
binary=root/'build/radio-backend/cariboulite_radio_backend'
role=sys.argv[1]; run=sys.argv[2]
out=pathlib.Path('/tmp')/run/role
out.mkdir(parents=True,mode=0o755)
ipc=out/'ipc'; ipc.mkdir(mode=0o700)
args=[str(binary),'--channel','RF24','--rx-frequency','430100000','--tx-frequency','430100000',
      '--tx-sample-rate','4000000','--tx-power','-12','--tx-source','tone','--tx-deviation','2500',
      '--rx-mode','nbfm','--rx-sample-rate','1000000','--rx-sinad','--rx-playback','plughw:USB,0,0',
      '--seconds','1200','--socket',str(ipc/'status.sock')]
meta={'host':platform.node(),'kernel':platform.release(),'role':role,'args':args,'uid':os.geteuid(),
      'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
(out/'metadata.json').write_text(json.dumps(meta,indent=2))
proc=None; control=None; status=None; sequence=0; last=None
log=(out/'backend.log').open('w'); statuses=(out/'status.jsonl').open('w'); commands=(out/'commands.jsonl').open('w')

def collect(wait=0):
    global last
    deadline=time.monotonic()+wait
    while True:
        try:
            raw=status.recv(16384)
            if not raw: raise RuntimeError('Status disconnected')
            last=json.loads(raw); statuses.write(raw.decode()+'\n'); statuses.flush()
        except BlockingIOError:
            if time.monotonic()>=deadline: break
            time.sleep(.01)
    if proc.poll() is not None: raise RuntimeError('Backend exited '+str(proc.returncode))
    return last

def command(text):
    global sequence
    sequence+=1
    wire=f'v2 {sequence} {text}\n'
    control.send(wire.encode()); reply=control.recv(1024).decode().strip()
    commands.write(json.dumps({'monotonic_s':time.monotonic(),'request':wire.strip(),'reply':reply})+'\n'); commands.flush()
    fields=reply.split()
    if len(fields)<4 or fields[1]!=str(sequence) or fields[2]!='OK': raise RuntimeError(reply)
    return reply

def waitstate(expected,timeout=12):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        p=collect(.06)
        if p and p['state']==expected and not p['control']['transition']:
            if expected!='TX' or p['tx']['running'] is True: return p
        if p and (p['state'] in ('FAULT','FPGA_NEEDS_PROGRAM') or p['tx'].get('fault')):
            raise RuntimeError('Radio fault '+json.dumps(p['control'])+' '+str(p['tx'].get('fault')))
    raise RuntimeError('State timeout '+expected+' last='+str(last and last['state']))

try:
    for p in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
        try:
            exe=p.read_bytes().split(b'\0')[0].split(b'/')[-1]
            if exe in (b'cariboulite_radio_backend',b'cariboulite_test_app',b'test_app'):
                raise RuntimeError('Radio owner already running '+str(p))
        except (FileNotFoundError,ProcessLookupError,PermissionError): pass
    env=dict(os.environ)
    if role=='rx' and env.get('CARIBOULITE_CAPTURE_SO'):
        env['LD_PRELOAD']=env['CARIBOULITE_CAPTURE_SO']
        env['CARIBOULITE_CAPTURE_PREFIX']=str(out/'capture')
    proc=subprocess.Popen(args,cwd=root,stdout=log,stderr=log,env=env)
    deadline=time.monotonic()+15
    while time.monotonic()<deadline and proc.poll() is None:
        if (ipc/'control.sock').exists() and (ipc/'status.sock').exists(): break
        time.sleep(.05)
    control=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);control.settimeout(15);control.connect(str(ipc/'control.sock'))
    status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.connect(str(ipc/'status.sock'));status.setblocking(False)
    command('get');waitstate('IDLE')
    if role=='tx': command('tx_tone 1000 0.4')
    send({'ok':True,'ready':True,'out':str(out),'metadata':meta})
    for line in sys.stdin:
        req=json.loads(line); op=req['op']
        try:
            if op=='shutdown':
                if role=='tx': command('tx_stop'); waitstate('IDLE')
                else: command('stop'); waitstate('IDLE')
                send({'ok':True,'stopped':True});break
            elif op=='power':
                assert role=='tx'
                command('tx_stop'); waitstate('IDLE'); command('tx_power '+str(int(req['dbm'])))
                command('tx_start'); p=waitstate('TX')
                send({'ok':True,'snapshot':p})
            elif op=='off':
                assert role=='tx';command('tx_stop');p=waitstate('IDLE');send({'ok':True,'snapshot':p})
            elif op=='rx_rate':
                assert role=='rx';command('stop');waitstate('IDLE');command('sample_rate '+str(int(req['hz'])))
                command('start');p=waitstate('RX');send({'ok':True,'snapshot':p})
            elif op=='capture':
                assert role=='rx'
                os.kill(proc.pid, signal.SIGUSR1)
                send({'ok':True,'armed_monotonic_s':time.monotonic(),'backend_pid':proc.pid})
            elif op=='rx_reset':
                assert role=='rx'
                kind=req['kind']
                if kind!='fpga_program':command('stop');waitstate('IDLE')
                assert kind in ('fpga_soft_reset','fpga_hard_reset','fpga_program')
                command(kind)
                p=waitstate('FPGA_NEEDS_PROGRAM' if kind=='fpga_hard_reset' else 'IDLE')
                send({'ok':True,'snapshot':p})
            elif op=='telemetry':
                info={'monotonic_s':time.monotonic(),'realtime_s':time.time(),'loadavg':os.getloadavg()}
                for key,path in [('cpu_frequency_khz','/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq'),('temperature_millic','/sys/class/thermal/thermal_zone0/temp'),('cpu_stat','/proc/stat'),('memory','/proc/meminfo')]:
                    try: info[key]=pathlib.Path(path).read_text()
                    except OSError as exc:info[key]=str(exc)
                info['backend_pid']=proc.pid
                info['backend_stat']=pathlib.Path('/proc/'+str(proc.pid)+'/stat').read_text()
                send({'ok':True,'telemetry':info})
            elif op=='status': send({'ok':True,'snapshot':collect(req.get('wait',0))})
            else: raise RuntimeError('Unknown operation')
        except Exception as exc:
            send({'ok':False,'error':str(exc)});raise
except Exception as exc:
    send({'ok':False,'error':str(exc),'out':str(out)})
    sys.exit(1)
finally:
    if proc is not None and proc.poll() is None:
        if control is not None:
            try: command('tx_stop' if role=='tx' else 'stop')
            except Exception: pass
        proc.send_signal(signal.SIGTERM)
        try: proc.wait(timeout=12)
        except subprocess.TimeoutExpired: proc.kill();proc.wait()
    if control: control.close()
    if status: status.close()
    log.close();statuses.close();commands.close()
