# RX sample-rate / SINAD diagnostic sweep, 2026-11-13

The user reported audible stuttering below 4 MS/s while receiving a transmitted
1 kHz NBFM tone. An RX-only sweep on `pi4f-trixie` compared 1, 2 and 4 MS/s,
with the SINAD estimator disabled and enabled. Each initial run lasted 40 s.
The transmitter was left under the user's control and was not reconfigured.

## Setup

- RF24, requested 430.100 MHz, NBFM, RX deviation normalization 2500 Hz.
- Direct ALSA playback `plughw:USB,0,0`, Jabra SPEAK 510 USB.
- Default noise squelch enabled, thresholds 0.20 / 0.30; carrier squelch off.
- Default PCM gain 8000 and 50 us audio de-emphasis.
- Backend built from commit `68a6513`; exact executable SHA-256 is in metadata.
- Backend launched via `sudo -n`: all RX workers obtained SCHED_FIFO and their
  requested CPU affinities. This may differ from an unprivileged launch.
- Native Jabra playback was confirmed at 48000 Hz, S16_LE stereo, period 480,
  buffer 2400 frames. Capture was stopped. No audio routing was changed.
- Read-only status collection at the default 5 Hz; full logs saved per run.

## Initial results

CPU percentages refer to one CPU core and are approximate whole-process
averages over each run, including normal status publication.

| RF rate | SINAD enabled | CPU % | Mean SINAD dB | Mean noise RMS | Mean raw RSSI dBm |
| --- | --- | ---: | ---: | ---: | ---: |
| 1 MS/s | No | 11.54 | unavailable | 0.0511 | -104.74 |
| 1 MS/s | Yes | 12.66 | 15.42 | 0.0524 | -104.86 |
| 2 MS/s | No | 15.56 | unavailable | 0.0539 | -103.74 |
| 2 MS/s | Yes | 15.74 | 15.44 | 0.0549 | -104.04 |
| 4 MS/s | No | 20.25 | unavailable | 0.0557 | -97.96 |
| 4 MS/s | Yes | 22.75 | 15.61 | 0.0564 | -97.69 |

All six runs exited normally. Every running snapshot reported zero application
IQ/audio queue drops, ALSA underruns, playback write errors and RX read errors.
There were no observed squelch closures after the initial two seconds.
Positive short reads and zero-length reads occurred; the reader accumulated
partial reads into full blocks without reporting a read error.

The audio queue was sampled at depth 0 or 1 of 24, and the backend's logged
resampling correction was +500 ppm throughout the sampled diagnostics, at all
three RF rates. That exposes little application-level buffering and a clock
correction at its emergency limit; it is a lead for further investigation,
not proof of an audio discontinuity or of a sample-rate-specific fault.

The raw signed RSSI register averaged about 7 dB higher at 4 MS/s than at
1 MS/s, while measured SINAD changed by less than 0.2 dB. The sweep changed RF
rate and therefore the configured digital cutoff (125/250/500 kHz), not just
software processing load. The mechanism behind the RSSI change was not
established. These RSSI values should not be treated as a calibrated common
input-power measurement across rates.

## Two-minute 1 MS/s repeat

An additional 120 s run with SINAD enabled also exited normally. Across all
599 running status snapshots, application IQ/audio queue drops, ALSA underruns,
playback write errors and RX read errors stayed at zero. The squelch did not
close after the initial settling interval. Mean SINAD was 15.83 dB (range
15.08–16.28 dB), mean noise RMS was 0.0576, and process CPU averaged 12.80%
of one core. Audio queue high-water depth reached 5/24, with final sampled
depth 4/24, so the nearly empty queue in the short tests did gradually fill.
The final counter difference of 480 requested versus written playback frames
is consistent with one in-flight write in a non-atomic status snapshot; no
write error or partial write was reported.

The long-run receiver artifacts are under
`/tmp/cariboulite-rx-sinad-sweep-20261113T200401Z`, also copied into the local
artifact directory below. Both tests left the receiver backend stopped.

## Scope and artifacts

Counters do not establish waveform continuity. No audio or IQ recording was
made, so these measurements cannot rule out audible modulation artifacts,
losses below the application queues, or speaker behavior. A steady tone makes
small artifacts easier to hear than speech. The result does not establish a
cause for the user's stutter report and does not justify a DSP change yet.

Raw logs, full status JSONL, per-run summaries, executable hash, sweep scripts,
USB hardware parameters and analysis are retained locally under:

`build/radio-backend/rx-sinad-sweep-20261113/`

This is an ignored build artifact directory. The original receiver-side copies
are under `/tmp/cariboulite-rx-sinad-sweep-20261113T195940Z` on `pi4f-trixie`.
No source files on the receiver were edited, and no transmitter commands were
issued.

## Follow-up: listening confirmation and original launch

The user confirmed that the audio during the sweep was smooth, without
interruptions. Their normal launch command was:

```sh
./build/radio-backend/cariboulite_radio_backend \
  --rx-playback plughw:CARD=USB,DEV=0 \
  --tx-microphone plughw:CARD=USB,DEV=0 --rx-sinad
```

`plughw:CARD=USB,DEV=0` and the sweep's `plughw:USB,0,0` select the same ALSA
card/device/subdevice. TX microphone configuration is not used by RX.
The important confirmed launch difference is privilege: the sweep used sudo;
the user's backend runs as UID 1000 (`pi`). A read-only SSH check found a
realtime-priority limit of 0 for the pi account, no executable extended
attributes/file capabilities, and zero effective/permitted/ambient process
capabilities. The observed backend had only its main thread at that moment
(RX was not running), so its RX worker scheduling was not observed directly.
This launch cannot grant the requested SCHED_FIFO worker priorities, whereas
the sweep recorded successful SCHED_FIFO scheduling for all three RX workers.
That is a concrete candidate explanation, not yet a controlled reproduction
of the stutter. A listening comparison with the same command, settings and
rate, adding sudo, is the next discriminating check. No running user backend
was stopped or reconfigured during these read-only checks.

## Subsequent power sweep

A later [wired TX power sweep](tx-power-sweep-2026-11-13.md) reproduced rapid
clicking/distortion and large within-window SINAD variation at RX 1 MS/s while
recorded application/ALSA/TX driver error counters remained zero. Timestamp-
filtered 2 and 4 MS/s measurements were stable during that later sweep. This
qualifies the earlier clean fixed-power result; zero counters alone do not
establish clean demodulated audio.
