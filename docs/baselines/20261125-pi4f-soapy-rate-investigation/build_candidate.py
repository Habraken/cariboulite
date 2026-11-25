from pathlib import Path
import subprocess,shlex,hashlib,json,shutil
repo=Path('/home/pi/src/cariboulite'); src=repo/'software/libcariboulite/src'; build=repo/'build'; out=build/'soapy-rate-fix-20261125';out.mkdir(exist_ok=True)
original=src/'soapy_api/Cariboulite.cpp'; data=original.read_text()
assert hashlib.sha256(data.encode()).hexdigest()=='4d944ae8b96808644bb7171f578a56f691a79951571b8d624e6c3e216a2f36e3'
old='if (std::fabs(rate - (1333000.0)) < 1) fs = cariboulite_radio_rx_sample_rate_1333khz;'
assert data.count(old)==1
fixed=data.replace(old,'if (std::fabs(rate - (1333000.0)) < 1)')
(out/'Cariboulite.before.cpp').write_text(data);(out/'Cariboulite.cpp').write_text(fixed)
flags={}
for line in (build/'CMakeFiles/SoapyCariboulite.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
  name,value=line.split('=',1);flags[name.strip()]=shlex.split(value)
compile_args=['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-I'+str(src/'soapy_api'),'-c',str(out/'Cariboulite.cpp'),'-o',str(out/'Cariboulite.cpp.o')]
subprocess.run(compile_args,cwd=build,check=True)
args=shlex.split((build/'CMakeFiles/SoapyCariboulite.dir/link.txt').read_text())
args=[a for a in args if not a.startswith('-Wl,--dependency-file=')]
args[args.index('-o')+1]=str(out/'libSoapyCariboulite.so')
obj='CMakeFiles/SoapyCariboulite.dir/src/soapy_api/Cariboulite.cpp.o';assert obj in args;args[args.index(obj)]=str(out/'Cariboulite.cpp.o')
subprocess.run(args,cwd=build,check=True)
paths=[build/a for a in args if a.endswith(('.o','.a','.so','.so.1.9.1','.so.0.8.1')) and (build/a).is_file()]
info={'source_sha256':hashlib.sha256(fixed.encode()).hexdigest(),'compile_command':compile_args,'link_command':args,'link_inputs_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},'candidate_sha256':hashlib.sha256((out/'libSoapyCariboulite.so').read_bytes()).hexdigest(),'original_installed_sha256':hashlib.sha256(Path('/usr/local/lib/SoapySDR/modules0.8-3/libSoapyCariboulite.so').read_bytes()).hexdigest()}
(out/'build.json').write_text(json.dumps(info,indent=2)+'\n')
print(json.dumps(info,indent=2))
