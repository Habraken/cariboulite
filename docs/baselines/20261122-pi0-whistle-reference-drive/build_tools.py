from pathlib import Path
import os,subprocess
root=Path('/home/pi/src/cariboulite');out=root/'build/whistle-drive';src=Path('/tmp/whistle-drive-tools')
for name in ('drive_probe','route_probe','capture_tap'):
 subprocess.run(['cc','-shared','-fPIC','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter',str(src/(name+'.c')),'-ldl',*(['-lasound'] if name=='capture_tap' else []),'-o',str(out/(name+'.so'))],check=True)
subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',str(src/'test_drive_probe.c'),'-ldl','-o',str(out/'test-drive')],check=True)
for ma in (None,2,4,6,8):
 env=dict(os.environ);env.pop('WHISTLE_REF_DRIVE_MA',None)
 if ma is not None:env['WHISTLE_REF_DRIVE_MA']=str(ma)
 subprocess.run([str(out/'test-drive'),str(3 if ma is None else ma//2-1)],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=True)
for value in ('','3','0','9','4junk'):
 r=subprocess.run([str(out/'test-drive'),'3'],env=dict(os.environ,WHISTLE_REF_DRIVE_MA=value),capture_output=True)
 assert r.returncode==98,(value,r.returncode)
# Linux dynamic interposition, using mock hardware only.
(out/'mock.c').write_text('''#include <stdint.h>
static int value;
void at86rf215_set_clock_output(void *p,int drive,int clock){(void)p;value=((drive&3)<<3)|(clock&7);}
int at86rf215_read_byte(void *p,uint16_t reg){(void)p;return reg==7?value:-1;}
''')
(out/'preload-check.c').write_text('''#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
void at86rf215_set_clock_output(void *,int,int);
int at86rf215_read_byte(void *,uint16_t);
int main(int argc,char **argv){assert(argc==2);for(int f=0;f<8;f++){at86rf215_set_clock_output(0,3,f);assert(at86rf215_read_byte(0,7)==((atoi(argv[1])<<3)|f));}return 0;}
''')
subprocess.run(['cc','-shared','-fPIC','-Wall','-Wextra','-Werror',str(out/'mock.c'),'-o',str(out/'libclockmock.so')],check=True)
subprocess.run(['cc',str(out/'preload-check.c'),'-L'+str(out),'-Wl,-rpath,'+str(out),'-lclockmock','-o',str(out/'preload-check')],check=True)
for ma in (2,4,6,8):
 subprocess.run([str(out/'preload-check'),str(ma//2-1)],env=dict(os.environ,LD_PRELOAD=str(out/'drive_probe.so'),WHISTLE_REF_DRIVE_MA=str(ma)),capture_output=True,check=True)
print('Native mock and dynamic interposition checks passed; no hardware accessed.')
