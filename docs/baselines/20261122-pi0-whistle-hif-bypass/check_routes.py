"""Validate frontend mode selection/readback recorded by route_probe.c."""
from pathlib import Path
import re,json
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
DATA=ROOT/'build/whistle-bypass-analysis/data'
rows=[]
for log in sorted(DATA.glob('*/**/backend.log')):
 events=[]
 for line in log.read_text().splitlines():
  if not line.startswith('BYPASS_ROUTE '):continue
  e={k:int(v) for k,v in re.findall(r'(requested|set_rc|actual|debug|read_rc)=(-?\d+)',line)}
  assert e['set_rc']==0 and e['read_rc']==0 and e['actual']==e['requested'],(str(log),e)
  assert e['requested'] in (0,1),(str(log),e)
  events.append(e)
 assert events and any(e['actual']==1 for e in events),str(log)
 rows.append(dict(log=str(log.relative_to(DATA)),bypass_selections=sum(e['actual']==1 for e in events),events=events))
assert len(rows)==10,len(rows)
(HERE/'route-verification.json').write_text(json.dumps(rows,indent=2)+'\n')
print('Verified bypass mode 1 readback in TX and all nine RX sessions; no conversion modes selected.')
