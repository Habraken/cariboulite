import hashlib, json, os, pathlib, select, signal, socket, subprocess, sys, time

root=pathlib.Path('/home/pi/src/cariboulite')
program='--program' in sys.argv
out=root/'build/tx-startup-20261124'/(('program-' if program else 'stress-')+time.strftime('%H%M%S'))
out.mkdir(parents=True)
ipc=out/'ipc';ipc.mkdir(mode=0o700)
assert pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode()=='36b67ac4-b313-4369-99bd-4b3e09caabfa'
old=pathlib.Path('/proc/1906')
if old.exists():
    assert (old/'exe').resolve().name=='cariboulite_radio_backend'
    os.kill(1906,signal.SIGTERM)
    until=time.monotonic()+12
    while old.exists() and time.monotonic()<until:time.sleep(.1)
    assert not old.exists(),'old backend did not exit'
binary=root/'build/radio-backend/cariboulite_radio_backend'
args=[str(binary),'--rx-playback','plughw:CARD=H650e,DEV=0','--tx-microphone','plughw:CARD=H650e,DEV=0',
      '--tx-source','tone','--rx-sinad','--rx-gain-profile','matched','--rx-mode-gain','am=-7',
      '--rx-mode-gain','usb=0','--rx-mode-gain','lsb=0','--tx-frequency','430100000',
      '--tx-power','-12','--seconds','240','--socket',str(ipc/'status.sock')]
(out/'metadata.json').write_text(json.dumps({'args':args,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
    'library_sha256':hashlib.sha256((root/'build/libcariboulite.so').read_bytes()).hexdigest(),
    'fpga_sha256':hashlib.sha256((root/'firmware/top.bin').read_bytes()).hexdigest(),'reprogram':program,'started':time.time()},indent=2))
log=(out/'backend.log').open('w');events=(out/'events.jsonl').open('w')
proc=subprocess.Popen(args,cwd=root,stdout=log,stderr=log)
control=status=None;seq=0;last=None;results=[]
def event(kind,value):
    events.write(json.dumps({'time':time.time(),'kind':kind,'value':value})+'\n');events.flush()
def collect(seconds):
    global last
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if select.select([status],[],[],min(.06,max(0,until-time.monotonic())))[0]:
            raw=status.recv(1048576)
            if not raw:raise RuntimeError('status disconnected')
            last=json.loads(raw);event('status',last)
        if proc.poll() is not None:raise RuntimeError('backend exited')
    return last
def command(text):
    global seq
    seq+=1;wire=f'v2 {seq} {text}\n'
    control.send(wire.encode());reply=control.recv(16384).decode().strip()
    event('command',{'request':wire.strip(),'reply':reply})
    if reply.split()[2]!='OK':raise RuntimeError(reply)
def wait(states,timeout=8):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        p=collect(.07)
        if p and p['state'] in states and not p['control']['transition']:return p
    raise RuntimeError('state timeout '+str(last and last['state']))
def stop():
    command('tx_stop');wait({'IDLE','FAULT'})
    command('stop');wait({'IDLE'})
try:
    until=time.monotonic()+15
    while not (ipc/'control.sock').exists():
        if proc.poll() is not None or time.monotonic()>until:raise RuntimeError('startup failed')
        time.sleep(.05)
    control=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);control.settimeout(6);control.connect(str(ipc/'control.sock'))
    status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.connect(str(ipc/'status.sock'));status.setblocking(False)
    command('get');wait({'IDLE'})
    command('fpga_regs')
    if program:
        command('fpga_program');wait({'IDLE','FAULT'},30)
        assert last['state']=='IDLE','FPGA programming failed'
        command('fpga_regs')
    plan=[(1000000,'tone')]*(32 if program else 12)+[(rate,'tone') for rate in (4000000,2000000,1333333,800000,666667,500000,400000)]*(4 if program else 2)
    print(json.dumps({'started':True,'out':str(out),'pid':proc.pid,'cycles':len(plan)}),flush=True)
    for n,(rate,source) in enumerate(plan,1):
        command('tx_sample_rate '+str(rate))
        command('tx_source '+source)
        if source=='tone':command('tx_tone 1000 0.4')
        command('tx_start');p=wait({'TX','FAULT'})
        start_state=p['state'];collect(.7 if start_state=='TX' else .25)
        p=last
        result={'cycle':n,'rate':rate,'source':source,'start_state':start_state,'snapshot':p}
        results.append(result);(out/'results.json').write_text(json.dumps(results,indent=2))
        print(json.dumps({'cycle':n,'rate':rate,'source':source,'state':start_state,'fault':p['tx']['fault'],
            'periods':p['tx']['driver'].get('completed_periods'),'underruns':p['tx']['driver'].get('underrun_periods'),
            'dequeued':p['tx']['driver'].get('fifo_dequeued_bytes'),'writer_error':p['tx']['workers']['writer']['last_error']}),flush=True)
        stop()
finally:
    if control is not None:
        try:stop()
        except Exception as e:event('cleanup_error',str(e))
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
        try:proc.wait(timeout=12)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
    if control:control.close()
    if status:status.close()
    log.close();events.close()
    k=subprocess.run(['journalctl','-k','--since','2026-11-24 00:03:00','--no-pager'],capture_output=True,text=True)
    (out/'kernel.log').write_text('\n'.join(x for x in k.stdout.splitlines() if 'Reading SMI settings to user' not in x)+'\n')
print(json.dumps({'done':True,'out':str(out),'cycles':len(results),'failed':sum(r['start_state']!='TX' for r in results),'backend_exit':proc.returncode}),flush=True)
