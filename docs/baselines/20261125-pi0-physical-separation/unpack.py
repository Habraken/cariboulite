"""Extract retained host evidence; never accesses radios."""
from pathlib import Path
import tarfile
root=Path(__file__).resolve().parent
for archive in sorted(root.glob('*-host.tar.gz')):
 with tarfile.open(archive) as t:
  assert all(not Path(m.name).is_absolute() and '..' not in Path(m.name).parts and not m.issym() and not m.islnk() for m in t.getmembers())
  t.extractall(root/'data',filter='data')
 print('Extracted',archive.name)
