import pathlib,os,json,platform,subprocess
names={b'cariboulite_radio_backend',b'cariboulite_test_app',b'cariboulite_radio_controller',b'cariboulite_radio_monitor',b'cariboulite_radio_monitor.py'}
owners=[];smi_fds=[]
for d in pathlib.Path('/proc').glob('[0-9]*'):
 try:
  args=(d/'cmdline').read_bytes().split(b'\0')
  if args[0].split(b'/')[-1] in names or (len(args)>1 and args[1].split(b'/')[-1] in names):owners.append(int(d.name))
  for fd in (d/'fd').iterdir():
   try:
    if os.readlink(fd)=='/dev/smi':smi_fds.append({'pid':int(d.name),'fd':fd.name})
   except (FileNotFoundError,PermissionError):pass
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
pcm={str(p):p.read_text().strip() for p in pathlib.Path('/proc/asound').glob('card*/pcm*p/sub*/status')}
result={'host':platform.node(),'kernel':platform.release(),'radio_processes':owners,'smi_open_fds':smi_fds,'playback':pcm,'loaded_driver_srcversion':pathlib.Path('/sys/module/smi_stream_dev/srcversion').read_text().strip(),'installed_driver_srcversion':subprocess.check_output(['/usr/sbin/modinfo','-F','srcversion','smi_stream_dev'],text=True).strip()}
assert not owners and not smi_fds,result
assert result['loaded_driver_srcversion']==result['installed_driver_srcversion']=='5AD373CAE4DBC118FE6E425',result
print(json.dumps(result))
