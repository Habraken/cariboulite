#!/usr/bin/env python3
"""CW backend integration through real control/status sockets; no RF or ALSA."""
from pathlib import Path
import json,os,socket,subprocess,tempfile,time
root=Path(__file__).resolve().parents[3]
binary=Path(os.environ.get('CARIBOULITE_BACKEND',root/'build/radio-backend/cariboulite_radio_backend'))
with tempfile.TemporaryDirectory(prefix='cw-backend-') as tmp:
    path=Path(tmp)/'status.sock'
    with (Path(tmp)/'backend.log').open('w+') as log:
        proc=subprocess.Popen([str(binary),'--dry-run','--socket',str(path),'--seconds','20','--status-hz','20'],stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+5
            while not (Path(tmp)/'control.sock').exists():
                assert proc.poll() is None
                assert time.monotonic()<deadline;time.sleep(.01)
            controller=binary.with_name('cariboulite_radio_controller')
            for text in ['cw setup iambic 12 0 1 250 5','cw rx 500 600']:
                result=subprocess.run([str(controller),'--socket',str(Path(tmp)/'control.sock'),'--command',text],capture_output=True,text=True,timeout=3)
                assert result.returncode==0,(text,result.stdout,result.stderr)
                time.sleep(.02)
            control=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);control.settimeout(3);control.connect(str(Path(tmp)/'control.sock'))
            status=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);status.settimeout(3);status.connect(str(path))
            sequence=0
            def command(text,ok=True):
                global sequence
                sequence+=1;control.send(f'v2 {sequence} {text}\n'.encode());reply=control.recv(4096).decode()
                assert (' OK ' in reply)==ok,(text,reply)
                return reply
            def until(predicate,timeout=2):
                end=time.monotonic()+timeout;latest=None
                while time.monotonic()<end:
                    latest=json.loads(status.recv(65536))
                    if predicate(latest):return latest
                raise AssertionError(latest)
            command('cw_setup iambic 12 0 1 250 5')
            command('cw_enable 1');command('cw_test 3')
            until(lambda s:s['cw']['busy'] and s['cw']['practice'])
            command('cw_setup cootie 20 0 1 250 5',False)
            time.sleep(.3)
            s=until(lambda s:s['cw']['busy']);assert not s['control']['tx_requested']
            command('cw_test 0');until(lambda s:not s['cw']['busy'])
            command('cw_setup straight 12 0 0 250 5');command('cw_test 1')
            s=until(lambda s:s['control']['tx_requested'])
            time.sleep(.6) # held contact is not timed out
            until(lambda s:s['control']['tx_requested'])
            generation=s['control']['tx_pipeline_generation']
            command('cw_test 0');time.sleep(.1);command('cw_test 1')
            held=until(lambda s:s['control']['tx_requested'])
            assert held['control']['tx_pipeline_generation']==generation
            command('cw_test 0')
            s=until(lambda s:s['control']['rx_requested'] and not s['control']['transition'])
            assert not s['control']['tx_requested']
            command('stop');command('mode cw');command('cw_rx 250 700');command('cw_enable 1');command('cw_test 2')
            s=until(lambda s:s['control']['tx_requested']);assert s['config']['tx_mode']=='cw'
            command('cw_enable 0')
            until(lambda s:not s['control']['tx_requested'] and not s['control']['transition'])
            command('cw_test 1',False)
            command('cw_setup straight 31 0 0 250 5',False)
            command('cw_rx 250 nan',False)
            command('cw_enable 1');command('cw_test 1');command('cw_enable 0')
            until(lambda s:not s['cw']['enabled'] and not s['control']['tx_requested'] and not s['control']['transition'])
            control.close();status.close()
        except BaseException:
            log.flush();log.seek(0);print(log.read());raise
        finally:
            proc.terminate();assert proc.wait(timeout=5)==0
print('CW backend: practice, key ownership, settings interlock, held key, automatic TX/RX, native CW, disarm and validation PASS')
