import pathlib,subprocess,json,time
root=pathlib.Path('/home/pi/src/cariboulite');out=pathlib.Path('/tmp')/('am-bittrace-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir()
loader=['python3',str(root/'firmware/diagnostics/program_image.py')]
print('OUTPUT='+str(out),flush=True)
result=None
try:
 load=subprocess.run(loader+['/tmp/am-bittrace-fpga/bittrace.bin','--sha256','79f2c1e698463e7c933359c5baaebaffb8772aa2412e87b2819a9aad3572484c','--receipt',str(out/'load.json')],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=40)
 (out/'load-output.log').write_text(load.stdout);print('LOAD_EXIT='+str(load.returncode),flush=True)
 if load.returncode:raise RuntimeError(load.stdout)
 with (out/'patterns.log').open('w') as log:
  completed=subprocess.run(['/tmp/am-bittrace'],cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=120)
 result=completed.returncode
 for line in (out/'patterns.log').read_text().splitlines():
  if line.startswith(('SUMMARY','FIXED_PATTERN_CONTROL','DRIVER native')):print(line,flush=True)
 print('PATTERN_EXIT='+str(result),flush=True)
finally:
 restore=subprocess.run(loader+['--restore','--receipt',str(out/'restore.json')],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=40)
 (out/'restore-output.log').write_text(restore.stdout);print('RESTORE_EXIT='+str(restore.returncode),flush=True)
 if restore.returncode:raise RuntimeError(restore.stdout)
print('COMPLETE='+str(out),flush=True)
raise SystemExit(result or 0)
