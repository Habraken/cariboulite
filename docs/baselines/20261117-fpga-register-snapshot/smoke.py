"""Idle-only explicit snapshot check; no LED writes, reset, RX or TX start."""
import pathlib,subprocess,socket,tempfile,json,time,platform,hashlib
root=pathlib.Path('/home/pi/src/cariboulite');out=root/'build/fpga-register-snapshot-20261117';out.mkdir(exist_ok=True)
for f in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
 try:assert f.read_bytes().split(b'\0')[0].split(b'/')[-1] not in (b'cariboulite_radio_backend',b'cariboulite_test_app',b'test_app'),'Radio owned'
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
ipc=pathlib.Path(tempfile.mkdtemp(prefix='fpga-reg-smoke-'));binary=root/'build/radio-backend/cariboulite_radio_backend';controller=binary.with_name('cariboulite_radio_controller');records=[];log=(out/'backend.log').open('w');p=None;peer=None
try:
 p=subprocess.Popen([str(binary),'--channel','RF24','--seconds','20','--socket',str(ipc/'status.sock')],stdout=log,stderr=log)
 deadline=time.monotonic()+15
 while not (ipc/'control.sock').exists():
  assert p.poll() is None and time.monotonic()<deadline;time.sleep(.05)
 peer=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);peer.settimeout(4);peer.connect(str(ipc/'control.sock'))
 for i,command in enumerate(['get','fpga_regs','get','fpga_regs','get'],1):
  peer.send(f'v2 {i} {command}\n'.encode());reply=peer.recv(4096).decode().strip();fields=reply.split()
  assert fields[2:4]==['OK','OK'] and fields[7]=='IDLE',reply
  assert len(fields)==(45 if command=='fpga_regs' else 33)
  records.append({'command':command,'reply':reply})
 assert records[0]['reply'].split()[2:]==records[2]['reply'].split()[2:]==records[4]['reply'].split()[2:]
 a=list(map(int,records[1]['reply'].split()[34:]));b=list(map(int,records[3]['reply'].split()[34:]));assert a[0]==b[0]==1023
 masks=[255,255,255,255,255,31,3,255,255,252]
 assert all((x&m)==(y&m) for x,y,m in zip(a[1:],b[1:],masks))
 peer.close();peer=None
 r=subprocess.run([str(controller),'--socket',str(ipc/'control.sock'),'--command','fpga registers'],capture_output=True,text=True,timeout=5)
 assert r.returncode==0 and 'SYS.gap_sync_source' in r.stdout and 'UNAVAILABLE' not in r.stdout,r.stderr
 (out/'controller.txt').write_text(r.stdout+r.stderr)
 (out/'smoke.json').write_text(json.dumps({'host':platform.node(),'board_uuid':pathlib.Path('/proc/device-tree/hat/uuid').read_bytes().rstrip(b'\0').decode(),'backend_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'library_sha256':hashlib.sha256((root/'build/libcariboulite.so').read_bytes()).hexdigest(),'records':records,'normal_get_unchanged':True,'stable_defined_control_bits':True,'controller_success':True,'no_rx_tx_led_reset_commands':True},indent=2)+'\n')
 print(r.stdout,flush=True)
finally:
 if peer:peer.close()
 if p is not None and p.poll() is None:p.terminate();p.wait(timeout=15)
 log.close()
