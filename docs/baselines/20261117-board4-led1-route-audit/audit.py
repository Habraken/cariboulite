"""Read-only audit of existing ASC: no synthesis, placement or programming.
Yosys proc lowers the reconstructed Verilog only for conservative fanout tracing.
"""
import collections,ctypes,hashlib,json,pathlib,re,subprocess,tempfile
ROOT=pathlib.Path(__file__).resolve().parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
with tempfile.TemporaryDirectory(prefix='led1-route-audit-') as temporary:
 d=pathlib.Path(temporary);pcf=[];pins={}
 for line in (ROOT/'firmware/io.pcf').read_text().splitlines():
  fields=line.split('#')[0].split()
  if fields and fields[0]=='set_io':
   name,pin=fields[-2:];pcf.append(f'set_io {name} {pin}');pins[name]=pin
 assert len(set(pins.values()))==len(pins),'duplicate package pin assignments'
 (d/'io.pcf').write_text('\n'.join(pcf)+'\n')
 subprocess.run(['icepack',str(ROOT/'firmware/top.asc'),str(d/'packed.bin')],check=True)
 h=(ROOT/'software/libcariboulite/src/cariboulite_fpga_firmware.h').read_text().split('uint8_t cariboulite_firmware[] = {')[1].split('}')[0]
 embedded=bytes(int(v,16) for v in re.findall(r'0x([0-9a-fA-F]+)',h))
 images={'asc_repacked':(d/'packed.bin').read_bytes(),'top.bin':(ROOT/'firmware/top.bin').read_bytes(),'source_embedded':embedded}
 lib=ROOT/'build/libcariboulite.so'
 if lib.exists():images['local_build_embedded']=bytes((ctypes.c_ubyte*len(embedded)).in_dll(ctypes.CDLL(str(lib)),'cariboulite_firmware'))
 assert len({sha(b) for b in images.values()})==1,'images differ'
 with (d/'routed.v').open('w') as f:
  subprocess.run(['icebox_vlog','-p',str(d/'io.pcf'),'-d','qn84','-L',str(ROOT/'firmware/top.asc')],stdout=f,check=True)
 with (d/'yosys.log').open('w') as f:
  subprocess.run(['yosys','-Q','-T','-p',f'read_verilog {d}/routed.v; hierarchy -top chip; proc; opt_clean; write_json {d}/routed.json'],stdout=f,stderr=subprocess.STDOUT,check=True)
 m=json.loads((d/'routed.json').read_text())['modules']['chip']
 led=m['netnames']['_o_led1$SB_IO_OUT']['bits'];seen=set(led);queue=collections.deque(led);cons=collections.defaultdict(list)
 for n,c in m['cells'].items():
  if 'port_directions' not in c:
   assert c['type']=='SB_RAM40_4K'
   c['port_directions']={p:('output' if p=='RDATA' else 'input') for p in c['connections']}
  for port,bits in c['connections'].items():
   if c['port_directions'][port]=='input':
    for bit in bits:
     if isinstance(bit,int):cons[bit].append((n,c))
 visited=set()
 while queue:
  bit=queue.popleft()
  for n,c in cons[bit]:
   if n in visited:continue
   visited.add(n)
   for port,bits in c['connections'].items():
    if c['port_directions'][port]=='output':
     for b in bits:
      if isinstance(b,int) and b not in seen:seen.add(b);queue.append(b)
 outputs=[n for n,p in m['ports'].items() if p['direction']!='input' and seen.intersection(p['bits'])]
 sensitive=[n for n,v in m['netnames'].items() if seen.intersection(v['bits']) and any(k in n for k in ['fifo','clock','smi','reset','rx_sync'])]
 assert set(outputs)=={'o_led1','o_miso'} and not sensitive
 result=dict(image_sha256={n:sha(b) for n,b in images.items()},image_bytes=len(embedded),asc_sha256=sha((ROOT/'firmware/top.asc').read_bytes()),pins=pins,conservative_led1_fanout_outputs=outputs,reachable_cells=len(visited),reachable_bits=len(seen),sensitive_named_nets=sensitive)
 print(json.dumps(result,indent=2))
