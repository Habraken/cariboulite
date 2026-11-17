#!/usr/bin/env python3
"""Offline analysis of bounded, little-endian raw SMI RX captures."""
import argparse
import hashlib
import json
from pathlib import Path

MASK = 0xC001C000
EXPECTED = 0x80004000
BITS = [14, 15, 16, 30, 31]


def phase_stats(data, phase, detailed=True):
    words = [int.from_bytes(data[k:k+4], 'little') for k in range(phase, len(data)-3, 4)]
    errors = {str(b): 0 for b in BITS}
    ones = [0]*32
    valid = longest = run = suffix = repeated = zero = full = 0
    previous = None
    for w in words:
        if detailed:
            for b in range(32):
                ones[b] += (w >> b) & 1
            for b in BITS:
                errors[str(b)] += ((w ^ EXPECTED) >> b) & 1
        good = (w & MASK) == EXPECTED
        valid += good
        run = run+1 if good else 0
        longest = max(longest, run)
        repeated += previous == w
        zero += w == 0
        full += w == 0xffffffff
        previous = w
    suffix = run
    return dict(phase=phase, words=len(words), valid_words=valid,
                longest_valid_run=longest, valid_suffix=suffix,
                framing_bit_errors=errors, bit_ones=ones,
                stuck_zero_bits=[b for b,n in enumerate(ones) if words and n == 0],
                stuck_one_bits=[b for b,n in enumerate(ones) if words and n == len(words)],
                adjacent_repeated_words=repeated, zero_words=zero, all_one_words=full)


def analyze(data):
    phases = [phase_stats(data, p) for p in range(4)]
    # Byte permutations are hypotheses, not automatic corrections. Try every
    # phase; retain a partial final word unchanged (ignored by phase_stats).
    swaps = {}
    for name, order in [('swap_bytes_in_halfwords', (1,0,3,2)),
                        ('swap_halfwords', (2,3,0,1)), ('reverse_word_bytes', (3,2,1,0))]:
        results = []
        for phase in range(4):
            aligned = data[phase:]
            changed = b''.join(bytes(aligned[k+i] for i in order)
                               for k in range(0, len(aligned)-3, 4))
            stats = phase_stats(changed, 0, detailed=False)
            results.append(dict(input_phase=phase, valid_words=stats['valid_words'],
                                words=stats['words'], longest_valid_run=stats['longest_valid_run']))
        swaps[name] = results
    return dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
                framing_mask=hex(MASK), framing_expected=hex(EXPECTED),
                phases=phases, byte_order_hypotheses=swaps,
                note='Aggregate run lengths do not reproduce decoder chunk boundaries. Stuck payload bits may be legitimate signal data. No upstream/SMI fault localization is implied.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    if args.capture.stat().st_size > 1024*1024:
        parser.error('capture exceeds diagnostic limit of 1 MiB')
    print(json.dumps(analyze(args.capture.read_bytes()), indent=2))


if __name__ == '__main__':
    main()
