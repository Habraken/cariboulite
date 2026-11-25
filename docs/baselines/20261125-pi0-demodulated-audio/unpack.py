"""Extract retained receiver captures for offline analysis; no radio access."""
from pathlib import Path
import tarfile
root=Path(__file__).resolve().parent;dest=root/'data';dest.mkdir(exist_ok=True)
for archive in sorted(root.glob('*-rx-host.tar.gz')):
 with tarfile.open(archive) as t:
  assert all(not Path(m.name).is_absolute() and '..' not in Path(m.name).parts and not m.issym() and not m.islnk() for m in t.getmembers())
  t.extractall(dest,filter='data')
 print('Extracted',archive.name)
