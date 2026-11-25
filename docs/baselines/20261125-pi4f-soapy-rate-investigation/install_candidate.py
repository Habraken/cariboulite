from pathlib import Path
import json,hashlib,os,shutil,time
repo=Path('/home/pi/src/cariboulite');out=repo/'build/soapy-rate-fix-20261125'
result=json.loads((out/'candidate-rates.json').read_text());assert result['exit_code']==0 and len(result['rates'])==8 and all(r['passed'] for r in result['rates'])
assert json.loads((out/'mock-tests.json').read_text())['passed']
for p in Path('/proc').glob('[0-9]*/exe'):
 try:
  assert p.resolve().name not in ('sdrpp','cariboulite_radio_backend','backend','cariboulite_test_app','check-rates'),str(p)
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
installed=Path('/usr/local/lib/SoapySDR/modules0.8-3/libSoapyCariboulite.so');source=repo/'software/libcariboulite/src/soapy_api/Cariboulite.cpp'
info=json.loads((out/'build.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(installed)==info['original_installed_sha256'];assert sha(out/'libSoapyCariboulite.so')==info['candidate_sha256']
assert source.read_bytes()==(out/'Cariboulite.before.cpp').read_bytes()
backup=out/'backup';shutil.copy2(source,backup/'Cariboulite.cpp')
# Update the known source file in place, preserving its owner/mode and other checkout changes.
source.write_bytes((out/'Cariboulite.cpp').read_bytes())
tmp=installed.with_name(installed.name+'.rate-fix-tmp');shutil.copyfile(out/'libSoapyCariboulite.so',tmp);os.chmod(tmp,0o644);os.chown(tmp,0,0);os.replace(tmp,installed)
assert sha(installed)==info['candidate_sha256'];assert sha(source)==info['source_sha256']
result={'installed':str(installed),'installed_sha256':sha(installed),'source_sha256':sha(source),'backup_plugin':str(backup/installed.name),'backup_source':str(backup/'Cariboulite.cpp'),'time_unix':time.time(),'hardware_rates_passed':8,'mock_tests_passed':True}
(out/'installation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
