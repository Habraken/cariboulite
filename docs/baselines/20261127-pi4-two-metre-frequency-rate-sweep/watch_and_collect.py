"""Collect this specific remotely running sweep after completion."""
import json,pathlib,shlex,subprocess,tarfile,time,traceback
here=pathlib.Path(__file__).resolve().parent
base='/home/pi/src/cariboulite/build/pi4-two-metre-sweep-20261127'
ssh=['ssh','-i','/Users/janhabraken/.ssh/id_ed25519_pi0a_trixie','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','pi@192.168.6.163']
probe="""import pathlib,json
p=pathlib.Path(%r)
def read(n):
 try:return json.loads((p/n).read_text())
 except (OSError,ValueError):return None
d={'update':read('update-completion.json'),'completion':read('completion.json')}
try:
 out=pathlib.Path((p/'latest-run.txt').read_text().strip())
 d.update(run=out.name,plan=json.loads((out/'plan.json').read_text()),progress=json.loads((out/'progress.json').read_text()))
except (OSError,ValueError):pass
print(json.dumps(d))
""" % base
for attempt in range(600):
 try:
  r=subprocess.run(ssh+['python3 -c '+shlex.quote(probe)],capture_output=True,text=True,timeout=40,check=True)
  d=json.loads(r.stdout);(here/'remote-status.json').write_text(json.dumps(d,indent=2)+'\n')
  if d.get('update') and any(d['update'].values()):raise RuntimeError('Host update failed: '+str(d['update']))
  if d.get('plan',{}).get('software_revision')=='a997ec4b92e28fc79fe38b320654d334dae61619' and (d.get('completion') or {}).get('out','').endswith('/'+d.get('run','?')):
   run=d['run'];assert run.startswith('pi4-two-metre-') and '/' not in run
   archive=here/(run+'.tar.gz')
   with archive.open('wb') as f:subprocess.run(ssh+['tar -czf - -C '+base+' '+run],stdout=f,check=True,timeout=600)
   with tarfile.open(archive) as t:t.extractall(here,filter='data')
   for name in ['rx-update.log','tx-update.log','update-completion.json','completion.json','controller.log','finalize.log']:
    r=subprocess.run(ssh+['cat '+base+'/'+name],capture_output=True,timeout=40,check=True)
    (here/name).write_bytes(r.stdout)
   (here/'collection-complete.json').write_text(json.dumps(d['completion'],indent=2)+'\n')
   if d['completion'].get('matrix_verification_exit')==0:
    plotpython=here.parents[2]/'build/sweep-analysis-venv/bin/python'
    subprocess.run([str(plotpython),str(here/'plot_sweep.py'),str(here/run)],check=True,timeout=180)
    report=here/run/'REPORT.md'
    figures='\n## Frequency response\n\n![RSSI and estimated SINAD](analysis/frequency-overview.png)\n\n![Frequency/rate maps](analysis/frequency-rate-heatmaps.png)\n\n![Rate profiles](analysis/rate-profiles.png)\n\n![Path boundaries](analysis/path-boundary-zooms.png)\n'
    report.write_text(report.read_text()+figures)
    import hashlib
    out=here/run
    (out/'SHA256SUMS').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(out))+'\n' for f in sorted(out.rglob('*')) if f.is_file() and f.name!='SHA256SUMS'))
   print('COLLECTED '+run,flush=True);break
 except Exception:
  traceback.print_exc()
 time.sleep(60)
else:raise RuntimeError('Collection timed out after ten hours')
