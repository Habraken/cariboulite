from pathlib import Path
import subprocess,shlex,hashlib,json,shutil
repo=Path('/home/pi/src/SDRPlusPlus');src=repo/'source_modules/soapy_source/src/main.cpp';build=repo/'build/source_modules/soapy_source';out=repo/'build/auto-bandwidth-fix-20261125';out.mkdir(exist_ok=True)
old='        float cur = bandwidthList[1];';original=src.read_text();assert original.count(old)==1
fixed=original.replace(old,'        // If no bandwidth reaches the sample rate, use the widest available.\n        float cur = bandwidthList.back();')
(out/'main.before.cpp').write_text(original);(out/'main.cpp').write_text(fixed)
values={}
for line in (build/'CMakeFiles/soapy_source.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
  k,v=line.split('=',1);values[k.strip()]=shlex.split(v)
args=['c++',*values['CXX_DEFINES'],*values['CXX_INCLUDES'],*values['CXX_FLAGS'],'-c',str(out/'main.cpp'),'-o',str(out/'main.cpp.o')]
subprocess.run(args,cwd=build,check=True)
link=shlex.split((build/'CMakeFiles/soapy_source.dir/link.txt').read_text());link=[x for x in link if not x.startswith('-Wl,--dependency-file=')]
link[link.index('-o')+1]=str(out/'soapy_source.so');link[link.index('CMakeFiles/soapy_source.dir/src/main.cpp.o')]=str(out/'main.cpp.o')
link=[('/lib/libsdrpp_core.so' if x=='../../core/libsdrpp_core.so' else '-Wl,-rpath,/usr/local/lib' if x.startswith('-Wl,-rpath,') else x) for x in link]
subprocess.run(link,cwd=build,check=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
info={'repo_revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'original_source_sha256':sha(out/'main.before.cpp'),'fixed_source_sha256':sha(out/'main.cpp'),'candidate_sha256':sha(out/'soapy_source.so'),'original_installed_sha256':sha(Path('/usr/lib/sdrpp/plugins/soapy_source.so')),'compile':args,'link':link,'installed_core_sha256':sha(Path('/lib/libsdrpp_core.so'))}
(out/'build.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
