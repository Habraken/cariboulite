import subprocess,time,json,re,pathlib
args=['gdb','-q','-batch','-p','2667','-ex','set pagination off']
for cmd in ["p *(double*)&'server::sampleRate'","p *(unsigned char*)&'server::compression'","x/24bx *(char**)&'server::bbuf'"]:args+=['-ex',cmd]
r=subprocess.run(args,capture_output=True,text=True);print(r.stdout,flush=True)
rows=[]
for i in range(6):
 text=subprocess.check_output(['ss','-tinp','sport = :5259'],text=True);row={'time':time.monotonic(),'ss':text}
 for k in ('bytes_acked','bytes_sent','bytes_retrans','bytes_received','notsent'):
  m=re.search(k+r':(\d+)',text);row[k]=int(m[1]) if m else 0
 rows.append(row)
 if i:print('DELIVERED_MBIT_S',(row['bytes_acked']-rows[i-1]['bytes_acked'])*8/(row['time']-rows[i-1]['time'])/1e6,'NOTSENT',row['notsent'],flush=True)
 if i<5:time.sleep(1)
root=pathlib.Path('/home/pi/src/cariboulite/build/soapy-rate-fix-20261125');(root/'4m-transport.json').write_text(json.dumps(rows,indent=2));(root/'4m-wire-header.txt').write_text(r.stdout+r.stderr)
