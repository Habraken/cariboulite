"""Mocked dynamic IF override; no hardware access."""
from pathlib import Path
import json,os,subprocess
p=Path('/home/pi/src/cariboulite/build/demod-audio-20261125')
subprocess.run(['cc','-shared','-fPIC','-O2','-Wall','-Wextra','-Werror',str(p/'if_probe.c'),'-ldl','-o',str(p/'if_probe.so')],check=True)
(p/'if_mock.c').write_text('#include <stdint.h>\nint64_t at86rf215_setup_channel(void*d,int ch,uint64_t f){(void)d;(void)ch;return (int64_t)f+7;}\n')
(p/'if_main.c').write_text('''#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
int64_t at86rf215_setup_channel(void*,int,uint64_t);
int main(void){uint64_t expected=getenv("DEMOD_TX_IF_HZ")?2385000000ULL:2495000000ULL;
assert(at86rf215_setup_channel(0,1,2495000000ULL)==(int64_t)expected+7);
assert(at86rf215_setup_channel(0,0,2495000000ULL)==2495000007LL);
assert(at86rf215_setup_channel(0,1,2450000000ULL)==2450000007LL);return 0;}
''')
subprocess.run(['cc','-shared','-fPIC',str(p/'if_mock.c'),'-o',str(p/'libifmock.so')],check=True)
subprocess.run(['cc',str(p/'if_main.c'),'-L'+str(p),'-Wl,-rpath,'+str(p),'-lifmock','-o',str(p/'test-if')],check=True)
env=dict(os.environ,LD_PRELOAD=str(p/'if_probe.so'));env.pop('DEMOD_TX_IF_HZ',None)
subprocess.run([str(p/'test-if')],env=env,check=True)
subprocess.run([str(p/'test-if')],env=dict(env,DEMOD_TX_IF_HZ='2385000000'),check=True)
bad=subprocess.run([str(p/'test-if')],env=dict(env,DEMOD_TX_IF_HZ='123'))
assert bad.returncode==98
(p/'if-test.json').write_text(json.dumps({'passed':True,'checks':['passthrough without override','only RF24 2495 MHz becomes 2385 MHz','real frequency return propagated','other frequencies/channels unchanged','invalid override rejected']},indent=2))
print('PASS IF override and dynamic interposition tests; no hardware used.')
