import pathlib, subprocess, sys, json, time
here=pathlib.Path(__file__).resolve().parent
processes=[]
for role,args in [('rx',['bash',str(here/'update_pi.sh')]),('tx',['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','pi@192.168.6.77','bash',str(here/'update_pi.sh')])]:
    log=(here/(role+'-update.log')).open('w')
    processes.append((role,subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT),log))
result={}
for role,proc,log in processes:
    result[role]=proc.wait(); log.close()
(here/'update-completion.json').write_text(json.dumps(result,indent=2)+'\n')
if any(result.values()):sys.exit('Update failed; sweep was not restarted')
subprocess.run([sys.executable,str(here/'run_and_finalize.py')],check=True)
