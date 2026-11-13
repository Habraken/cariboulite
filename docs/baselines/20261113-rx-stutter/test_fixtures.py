#!/usr/bin/env python3
"""Hardware-free checks of real SMI false-lock fixtures. No radio access."""
import hashlib,json,pathlib,struct
base=pathlib.Path(__file__).resolve().parent
for f in json.loads((base/'fixtures.json').read_text()):
    data=(base/f['file']).read_bytes()
    assert hashlib.sha256(data).hexdigest()==f['sha256']
    def framing(offset):
        return [struct.unpack_from('<I',data,k)[0]&0xc001c000==0x80004000
                for k in range(offset,len(data)-3,4)]
    # Existing detector accepts the first phase whose first four words match.
    legacy=next(phase for phase in range(4) if all(framing(phase)[:4]))
    assert legacy==f['legacy_offset'] and legacy!=f['expected_offset']
    # Independent ground truth: every complete word at the expected phase
    # has the hardware framing symbols; the selected legacy phase does not.
    assert all(framing(f['expected_offset']))
    assert not all(framing(legacy))
    score=[sum(framing(p))/len(framing(p)) for p in range(4)]
    assert score.index(max(score))==f['expected_offset']
    print(f"{f['file']}: legacy falsely selects {legacy}; full fixture confirms phase {f['expected_offset']}")
