import pathlib,subprocess,json,os,struct,ctypes,re,time
pid=2667;core='/lib/libsdrpp_core.so';root=pathlib.Path('/home/pi/src/cariboulite/build/soapy-rate-fix-20261125')
syms={}
for line in subprocess.check_output(['nm','-D',core],text=True).splitlines():
 parts=line.split()
 if len(parts)==3:
  try:syms[parts[2]]=int(parts[0],16)
  except ValueError:pass
base=next(int(x.split('-')[0],16) for x in pathlib.Path('/proc',str(pid),'maps').read_text().splitlines() if 'libsdrpp_core.so' in x and x.split()[2]=='00000000')
fd=os.open('/proc/'+str(pid)+'/mem',os.O_RDONLY)
lib=ctypes.CDLL('libzstd.so.1');lib.ZSTD_getFrameContentSize.argtypes=[ctypes.c_void_p,ctypes.c_size_t];lib.ZSTD_getFrameContentSize.restype=ctypes.c_ulonglong;lib.ZSTD_decompress.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t];lib.ZSTD_decompress.restype=ctypes.c_size_t;lib.ZSTD_isError.argtypes=[ctypes.c_size_t];lib.ZSTD_isError.restype=ctypes.c_uint
rows=[]
for i in range(6):
 rate=struct.unpack('<d',os.pread(fd,8,base+syms['_ZN6server10sampleRateE']))[0]
 ptr=struct.unpack('<Q',os.pread(fd,8,base+syms['_ZN6server4bbufE']))[0]
 typ,size=struct.unpack('<II',os.pread(fd,8,ptr));assert typ in (2,3) and 16<=size<4000000
 packet=os.pread(fd,size,ptr);body=packet[8:];payload=body
 if typ==3:
  n=lib.ZSTD_getFrameContentSize(body,len(body));assert 8<=n<4000000
  target=ctypes.create_string_buffer(n);result=lib.ZSTD_decompress(target,n,body,len(body));assert not lib.ZSTD_isError(result);payload=target.raw[:result]
 version,sampletype,scaler=struct.unpack('<HHf',payload[:8]);bits={0:8,1:16,2:32}[sampletype]
 sock=subprocess.check_output(['ss','-tin','sport = :5259'],text=True)
 dm=subprocess.check_output(['dmesg'],text=True);lines=[x for x in dm.splitlines() if 'init programmed read. missed:' in x]
 row={'time':time.monotonic(),'sample_rate_hz':rate,'wire_bits':bits,'compressed':typ==3,'wire_packet_bytes':size,'samples_per_packet':(len(payload)-8)//(bits//8*2),'ss':sock,'latest_rx_counter':lines[-1] if lines else None}
 for k in ('bytes_acked','notsent'):
  m=re.search(k+r':(\d+)',sock);row[k]=int(m[1]) if m else 0
 rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='ss'}),flush=True)
 if i<5:time.sleep(1)
os.close(fd);(root/'4m-passive-observation.json').write_text(json.dumps(rows,indent=2)+'\n')
