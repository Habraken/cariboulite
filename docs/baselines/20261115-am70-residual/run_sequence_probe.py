import pathlib,subprocess,json,time
root=pathlib.Path('/home/pi/src/cariboulite');out=pathlib.Path('/tmp')/('am70-sequence-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir()
loader=['python3',str(root/'firmware/diagnostics/program_image.py')]
print('OUTPUT='+str(out),flush=True)
result=None
try:
 load=subprocess.run(loader+['/tmp/am70-sequence-fpga/sequence.bin','--sha256','41d9effb16feffaf05d9638340d35f5a389224410c90f32230989fd7dada4cf7','--receipt',str(out/'load.json')],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=40)
 (out/'load-output.log').write_text(load.stdout);print('LOAD_EXIT='+str(load.returncode),flush=True)
 if load.returncode:raise RuntimeError(load.stdout)
 result=0
 for name,args in [('clean',[]),('negative',['bad'])]:
  with (out/(name+'.log')).open('w') as log:
   completed=subprocess.run(['/tmp/am70-sequence',*args],cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=45)
  print(name+' EXIT='+str(completed.returncode),flush=True)
  print((out/(name+'.log')).read_text()[-1200:],flush=True)
  result=max(result,completed.returncode)
finally:
 restore=subprocess.run(loader+['--restore','--receipt',str(out/'restore.json')],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=40)
 (out/'restore-output.log').write_text(restore.stdout);print('RESTORE_EXIT='+str(restore.returncode),flush=True)
 if restore.returncode:raise RuntimeError(restore.stdout)
print('COMPLETE='+str(out),flush=True)
raise SystemExit(result or 0)
