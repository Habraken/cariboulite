# Alignment fix validation, 2026-11-13

Normal RX now maintains per-device stream framing, complete-word buffering and
partial-byte carry across legacy/timed read calls. It never pads or interpolates
missing IQ. Initial acquisition requires one unambiguous phase with at least
512 valid consecutive words in the retained suffix, allowing a counted stale
startup prefix; ambiguous input waits for more evidence up to a bounded window,
then fails explicitly. Every locked frame is checked. A framing failure reports
an error and updates counters instead of decoding header bits as IQ.

Stream/channel changes clear buffered data; the RX reader is joined before
those transitions. The shared global stream-state cache was replaced with an
actual per-device state update. New protocol-v1 `rx.alignment` counters and a
monitor row expose locks, acquisition failures, ambiguous windows, malformed
frames, discarded bytes, genuine output samples and intentional stream resets.
Other applications need rebuilding against the matched updated library/headers.

## Software validation

- UBSan C regressions: all four byte phases, both I/Q channel mappings, arbitrary
  one-byte/short input fragments, small output capacities, retained complete words,
  the actual false-lock tone fixtures, stale startup prefixes, ambiguous input,
  damaged frames and recovery. No interpolated or duplicate samples are accepted.
- Soapy/SMI deadline and partial-I/O suite, including byte-fragmented syscall reads
  with one-sample callers and buffered timed-read native-batch limits.
- Complete backend suite: five C suites, the new framing regression and 96 Python
  tests. The framing JSON values and monitor text also have a live serializer fixture.
- Production decode of both full 32 MB raw bad captures exactly matched every
  complete word of the independent continuously aligned reference: 7,999,999
  genuine IQ samples each, one acquisition, no framing failures, and only the
  initial one/two bytes discarded. The final three/two incomplete bytes remain
  buffered, not synthesized. See [exact output hashes](baselines/20261113-alignment-fix/full-capture-proof.json).

AddressSanitizer could not reserve its shadow address range in the execution
sandbox; the same framing suite passed with UndefinedBehaviorSanitizer.

## Live validation

pi4g was TX at 430.100 MHz, RF24, 4 MS/s, requested 0 dBm, 1000 Hz NBFM tone,
amplitude 0.4 and configured 2500 Hz deviation through the existing nominal
90 dB wired attenuation. pi4f was RX with direct Jabra playback and realtime
workers. The production library/backend were used with **no diagnostic preloads**.

| Fresh RX case | Verified duration | Mean SINAD | SINAD range | Acquisitions | Discarded startup bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| fresh_2m | 15 s | 21.93 dB | 21.35–22.28 dB | 1 | 3 |
| fresh_4m | 15 s | 22.06 dB | 21.89–22.36 dB | 1 | 1 |
| fresh_1m_a | 15 s | 21.55 dB | 21.36–21.74 dB | 1 | 2 |
| fresh_1m_b | 15 s | 21.53 dB | 21.30–21.80 dB | 1 | 1680 |
| fresh_1m_c | 15 s | 21.47 dB | 21.30–21.65 dB | 1 | 1632 |
| sustained_1m | 120 s | 21.56 dB | 20.60–21.97 dB | 1 | 1664 |

Every case passed the predeclared min-SINAD (>15 dB), within-run spread (<2 dB),
zero alignment-failure/malformed-frame, zero application drop/read-error/ALSA
underrun and zero TX driver-underrun checks. The three fresh 1 MS/s cases and
additional sustained run were not conditioned with a reset or higher RX rate.
One early implementation rejected a 2 MS/s startup before testing the full
matrix; acquisition was refined and the entire final matrix rerun successfully.
The final timed-read output-bound refinement was covered by the SMI API tests;
the live backend uses the legacy RX reader.

[Machine-readable results](baselines/20261113-alignment-fix/hardware-results.json)
and [setup](baselines/20261113-alignment-fix/metadata.json) are retained. Raw
logs are in `build/radio-backend/implementation-20261113/` (ignored), including
the successful run `cariboulite-alignment-proof-20261113T215214Z` and failed
first attempt. Both radios were automatically stopped by the test supervisor.

This fixes the diagnosed [false-lock failure](rx-stutter-diagnosis-2026-11-13.md).
SINAD remains an uncalibrated link estimate; short validation does not establish
indefinite continuity under every host workload or all possible RF waveforms.
