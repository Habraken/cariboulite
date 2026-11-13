# RX stutter diagnosis: false SMI byte alignment, 2026-11-13

**The reproduced stutter is caused by a software framing detector accepting a
false byte alignment in received SMI data.** Initial receiver state changes
which byte phase is presented at stream startup, explaining the apparently
intermittent behavior. The received RF waveform in the bad captures was clean:
correctly unpacking those same raw bytes restores about 22 dB SINAD.

The original CaribouLite on `pi4g-trixie` remained the transmitter and
`pi4f-trixie` remained the receiver throughout. They used the existing wired
connection with 90 dB nominal attenuation, RF24 at 430.100 MHz. TX was held at
4 MS/s, requested 0 dBm, NBFM, a 1000 Hz tone of amplitude 0.4, and configured
2500 Hz deviation. TX was not restarted or changed within each experiment
block. RX used the Jabra directly and realtime worker scheduling.

Both radios are now stopped and released. Installed backend, shared-library
and firmware-file hashes are unchanged. Receiver FPGA runtime/configuration
resets were exercised, including loading the existing `firmware/top.bin`; no
firmware file was modified. Neither Pi was rebooted or power-cycled.

## Evidence of the cause

The SMI frame format has fixed header bits in each four-byte IQ word:

```c
(word & 0xC001C000) == 0x80004000
```

The existing [caribou_smi_find_buffer_offset](../software/libcariboulite/src/caribou_smi/caribou_smi.c)
accepts the first byte offset at which **four consecutive words** match those
bits. In a narrowband tone, slowly changing payload bits can also satisfy that
short test at the wrong phase. The decoder then interprets parts of headers
and payload as IQ values. Normal magnitudes near 80 ADC counts become false
excursions of several thousand counts, producing FM noise bursts and clicks.

Two faulty fresh 1 MS/s starts yielded these captures:

| Capture | Correct byte offset | Legacy false-lock offset | False locks / device reads | Samples in false-lock reads |
| --- | ---: | ---: | ---: | ---: |
| fresh_1m_a | 1 | 0 | 62 / 1044 | 465184 |
| fresh_1m_b | 2 | 1 | 79 / 1044 | 530400 |

At the correct phase, every complete frame in each raw capture has valid
header bits. The raw byte stream is intact; reporting an “invalid frame” rate
at byte offset zero would mislabel a byte-phase problem as hardware corruption.
The saved library-decoded IQ is the corrupted representation.

Simulating the original detector and unpacker matched **every checked
non-interpolated returned IQ sample**: 7,999,018 samples in capture A and
7,998,956 in capture B. This connects the actual returned data to the false
alignment decisions, rather than merely correlating stutter with a reset.
The existing decoder also discards partial words and synthesizes samples when
realigning, which is separate unfinished work for a production repair.

Offline replay used the same current NBFM DSP with fixed audio-rate correction
zero, without radio hardware, worker scheduling, ALSA, or squelch muting:

| Capture | Replay of library-decoded IQ | Replay after correct continuous byte alignment |
| --- | ---: | ---: |
| fresh_1m_a | 3.17 dB | 22.47 dB |
| fresh_1m_b | 3.04 dB | 22.20 dB |

The corrected decode only chooses the proper phase and retains the continuous
complete words; it does not repair the waveform, interpolate away clicks, or
alter the transmitter. Offline statistics exclude the first second for filter
settling. SINAD remains the existing approximate unweighted estimator.

[Plot: the same capture before and after correct alignment](baselines/20261113-rx-stutter/offline-replay.svg)

## Receiver history experiment

The first test held TX continuous while RX followed
**1 → 2 → 4 → 1 → 4 → 2 → 1 MS/s**. Each return to 1 MS/s was made within the
same backend session. This separated sample-rate history from TX power and
elapsed power-sweep order.

| Phase | RX rate | Measurement duration | Mean SINAD | Observed SINAD range |
| ---: | --- | ---: | ---: | ---: |
| 0 | 1 MS/s | 60 s | 2.74 dB | 1.00–21.81 dB |
| 1 | 2 MS/s | 35 s | 21.95 dB | 21.63–22.26 dB |
| 2 | 4 MS/s | 35 s | 22.10 dB | 21.82–22.30 dB |
| 3 | 1 MS/s | 60 s | 21.82 dB | 21.40–22.10 dB |
| 4 | 4 MS/s | 35 s | 22.00 dB | 21.85–22.20 dB |
| 5 | 2 MS/s | 35 s | 21.86 dB | 21.62–22.11 dB |
| 6 | 1 MS/s | 60 s | 21.56 dB | 20.99–21.94 dB |

The first 1 MS/s start was faulty. Subsequent 1 MS/s returns after higher-rate
operation were clean. A new backend was then used for each capture case while
the transmitter stayed continuous within that block:

| Receiver startup case | Mean live SINAD during capture | Range | Correct raw byte phase |
| --- | ---: | ---: | ---: |
| Fresh 1 MS/s, A | 2.96 dB | 1.49–9.54 dB | 1 |
| Run at 4 MS/s, then 1 MS/s | 21.54 dB | 21.18–22.03 dB | 0 |
| Fresh 1 MS/s, B | 3.11 dB | 2.40–5.18 dB | 2 |
| FPGA soft reset, then 1 MS/s | 21.51 dB | 21.31–21.64 dB | 0 |
| FPGA hard reset + existing image, then 1 MS/s | 21.74 dB | 21.63–21.90 dB | 0 |
| Fresh 2 MS/s | 21.52 dB | 21.38–21.60 dB | 0 |
| Fresh 4 MS/s | 21.49 dB | 21.40–21.65 dB | 1 |

Reset/warmup changed the initial byte phase and avoided the bug in these tests.
This does not establish that a reset will always align every future stream.
The important result is that 1 MS/s itself can operate cleanly, and software
must handle the byte phases a stream can present at startup.

The fault was present before capture was armed and remained present while
capture ran. Clean warmup/reset cases stayed clean with the same capture code.
The temporary hook copied bounded IQ, metadata and raw `/dev/smi` reads into
prefaulted RAM, writing files only at normal backend shutdown. It did not
change returned data. Mock tests checked exact returned samples, metadata,
device selection, buffer bounds, read records and shutdown flushing.

All final recorded application RX queue drops, audio queue drops, ALSA
underruns, RX read errors, playback write errors, TX queue drops, TX writer
errors and TX driver underruns were zero. False-positive alignment is accepted
as a normal decode, so those counters cannot detect it. Both Pis reported
`throttled=0x0` at final inspection. Pi load or playback scheduling is not needed
to explain the captured failure; the same failure can be reproduced offline.

## Temporary decoder candidate: live confirmation

A diagnostic candidate replaced the four-word first-match decision with
whole-read framing agreement, using the previous phase to break equal-score
ties on short reads. Offline C regression tests selected the correct phase on
all 2,088 device reads in the two faulty captures, avoiding all 141 false locks.

The candidate was loaded with `LD_PRELOAD` only into test receiver processes.
Its other SMI routines came from the same unmodified source; TX used the
installed library throughout. It did not reset or change the raw stream's
byte phase, so the live nonzero-phase results test the software correction
directly:

| Candidate fresh start | Mean SINAD during capture | Range | Selected phase | Legacy false locks avoided during run |
| --- | ---: | ---: | ---: | ---: |
| candidate_fresh_1m_a | 21.56 dB | 21.16–21.89 dB | 2 | 180 |
| candidate_fresh_1m_b | 21.57 dB | 21.36–21.73 dB | 1 | 167 |
| candidate_fresh_1m_c | 21.55 dB | 21.36–21.73 dB | 0 | 0 |
| candidate_fresh_2m | 21.72 dB | 21.55–21.82 dB | 0 | 0 |
| candidate_fresh_4m | 21.51 dB | 21.45–21.62 dB | 0 | 0 |

This candidate validates the cause. It is **not installed and not presented as
a finished production fix**: it retains the old padding/fragment behavior,
uses diagnostic single-reader phase state, and has only short live-run
validation. A complete repair should maintain framing and partial-word carry
per stream, avoid false locks across arbitrary read sizes, return genuine
sample counts, and expose alignment/resynchronization diagnostics. Debug
modes, both radio channels and timed/legacy read APIs need regression coverage.

## Practical workaround and next implementation

Until the decoder is repaired, on the **receiving pi4f** controller with RX/TX
stopped:

```text
fpga soft reset
rx start
```

The selected RX settings remain in effect. Another observed workaround is to
receive briefly at 4 MS/s, stop RX, select 1 MS/s and restart. These are observed
workarounds, not substitutes for correct decoding. No Pi reboot was needed to
recover clean operation in this investigation.

The permanent fix should be driven by the captured false-lock fixtures and
continuous-byte-stream tests, rather than simply adding an unconditional FPGA
reset or requiring 4 MS/s. This diagnosis does not change the calibration
limitations of [the earlier power sweep](tx-power-sweep-2026-11-13.md).

## Evidence and reproduction

- [Structured results and alignment proof](baselines/20261113-rx-stutter/summary.json)
- [Experimental setup](baselines/20261113-rx-stutter/metadata.json)
- [Release checks, throttling flags and unchanged installed hashes](baselines/20261113-rx-stutter/release-and-hashes.txt)
- [Two real 512-byte regression fixtures and expected phases](baselines/20261113-rx-stutter/fixtures.json)
- [Hardware-free fixture checks](baselines/20261113-rx-stutter/test_fixtures.py)
- [Diagnostic candidate patch](baselines/20261113-rx-stutter/consensus-diagnostic.patch)

Run the small fixture checks without hardware:

```sh
python3 docs/baselines/20261113-rx-stutter/test_fixtures.py
```

The baseline directory preserves the collector, capture, replay, proof and
candidate-generation source artifacts. The candidate patch is diagnostic;
do not apply it as an installed production replacement without completing the
streaming decoder work and broader tests described above.

Complete raw captures, original library-decoded IQ, continuously aligned IQ,
replayed PCM/raw audio, detailed logs and compressed receiver/transmitter
archives are retained in the ignored directory:

`build/radio-backend/stutter-diagnosis-20261113/`

Run identifiers are `cariboulite-fixed-tx-20261113T204722Z`,
`cariboulite-capture-cases-20261113T205437Z`, and
`cariboulite-consensus-trial-20261113T210846Z`. The corresponding `/tmp` copies
on the radios are temporary; the local archives preserve them. Checksum and
byte-phase assertions, exact legacy-unpacker simulation, same-data offline
replay and the live nonzero-phase candidate runs support the diagnosis.
