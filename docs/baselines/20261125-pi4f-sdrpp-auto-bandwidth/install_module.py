from pathlib import Path
import hashlib,json,shutil,os,time
repo=Path('/home/pi/src/SDRPlusPlus');out=repo/'build/auto-bandwidth-fix-20261125';src=repo/'source_modules/soapy_source/src/main.cpp';installed=Path('/usr/lib/sdrpp/plugins/soapy_source.so');sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=json.loads((out/'server-test.json').read_text());assert checks['passed'] and len(checks['rows'])==3 and all(r['passed'] for r in checks['rows'])
for p in Path('/proc').glob('[0-9]*/exe'):
 try:assert p.resolve().name!='sdrpp','server still running'
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
info=json.loads((out/'build.json').read_text());assert sha(installed)==info['original_installed_sha256'];assert sha(src)==info['original_source_sha256'];assert sha(out/'soapy_source.so')==info['candidate_sha256']
backup=out/'backup';backup.mkdir(exist_ok=True)
assert not (backup/'soapy_source.so').exists();shutil.copy2(installed,backup/'soapy_source.so');shutil.copy2(src,backup/'main.cpp')
src.write_bytes((out/'main.cpp').read_bytes());tmp=installed.with_name(installed.name+'.auto-bw-tmp');shutil.copyfile(out/'soapy_source.so',tmp);os.chmod(tmp,0o644);os.chown(tmp,0,0);os.replace(tmp,installed)
assert sha(installed)==info['candidate_sha256'] and sha(src)==info['fixed_source_sha256']
result={'installed':str(installed),'installed_sha256':sha(installed),'fixed_source_sha256':sha(src),'backup_directory':str(backup),'time_unix':time.time(),'server_test_passed':True}
(out/'installation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
