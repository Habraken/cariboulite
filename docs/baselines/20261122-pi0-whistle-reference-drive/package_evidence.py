"""Package immutable captures per drive block; keep individual archives modest."""
from pathlib import Path
import json,tarfile,hashlib
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];DATA=ROOT/'build/whistle-drive-analysis/data'
rows=json.loads((HERE/'measurements.json').read_text());assert len(rows)==36
for block in ('a8','b4','c2','d8'):
 names=[m['case']['name'] for m in rows if m['case']['block']==block];assert len(names)==9
 with tarfile.open(HERE/('captures-'+block+'.tar.gz'),'w:gz',compresslevel=5) as archive:
  for name in names:
   for p in sorted((DATA/name).rglob('*')):
    if p.is_file() and '.decoded.' not in p.name:archive.add(p,arcname=str(p.relative_to(DATA)),recursive=False)
 with tarfile.open(HERE/('captures-'+block+'.tar.gz')) as archive:
  members=archive.getmembers();assert sum(m.name.endswith('/capture.iq') for m in members)==9
  assert sum(m.name.endswith('/capture.pcm') for m in members)==9
 print('Packaged',block,flush=True)
confirmation=json.loads((HERE/'confirmation-measurements.json').read_text());assert len(confirmation)==4
with tarfile.open(HERE/'captures-confirmation.tar.gz','w:gz',compresslevel=5) as archive:
 for m in confirmation:
  for p in sorted((DATA/m['case']['name']).rglob('*')):
   if p.is_file() and '.decoded.' not in p.name:archive.add(p,arcname=str(p.relative_to(DATA)),recursive=False)
with tarfile.open(HERE/'captures-confirmation.tar.gz') as archive:
 members=archive.getmembers();assert sum(m.name.endswith('/capture.iq') for m in members)==4
 assert sum(m.name.endswith('/capture.pcm') for m in members)==4
with tarfile.open(HERE/'tx-session.tar.gz','w:gz',compresslevel=5) as archive:
 for name in ('tx-session','confirmation-tx-session'):
  for p in sorted((DATA/name).rglob('*')):
   if p.is_file():archive.add(p,arcname=str(p.relative_to(DATA)),recursive=False)
for p in HERE.glob('*-helper.log'):
 assert not p.stat().st_size,p.read_text()
 p.unlink()
with (HERE/'SHA256SUMS').open('w') as out:
 for p in sorted(HERE.rglob('*')):
  if p.is_file() and p.name!='SHA256SUMS' and '__pycache__' not in p.parts:
   out.write(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(HERE))+'\n')
print('Capture archives validated and checksums recorded.')
