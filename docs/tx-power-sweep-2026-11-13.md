# Wired TX power / RX sample-rate sweep, 2026-11-13

The original CaribouLite on **pi4g-trixie** transmitted a 1 kHz NBFM tone over
the user's wired connection with **90 dB nominal attenuation** to **pi4f-trixie**.
TX stayed at **4 MS/s** while RX was tested at **1, 2 and 4 MS/s**. Requested TX
power was swept from **−12 to +14 dBm in 2 dB steps**, with a TX-off baseline
at each RX rate: 42 transmitting points and three baselines.

**The intermittent 1 MS/s problem was reproduced.** Within-window SINAD varied
by more than 2 dB at 9 of 14 power points at 1 MS/s, with rapid noise-RMS spikes.
The user independently heard fast breaks with distortion/clicking, distinct
from the intentional pauses between power steps, and reported no stuttering
in the later runs. At 2 and 4 MS/s every timestamp-filtered power point had
less than 2 dB of SINAD spread. This describes this sweep, not a guarantee of
long-term continuity at those rates.

All recorded RX application IQ/audio queue drop, ALSA underrun, playback write
error and RX read error counters stayed zero, as did the recorded TX driver
underrun, TX queue drop and TX writer error counters. Therefore those counters
do not explain the disturbance. Its appearance in pre-playback SINAD and noise
telemetry means the disturbance is visible before playback and is not
explained solely by a speaker/ALSA playback interruption. The fault has not been localized to the
TX source, RF path, RX hardware/driver or demodulator.

Both backends and controllers were verified absent after the sweep; the Jabra
playback device was closed. No radio application source, library or firmware
was changed for this measurement.

## Plots and recorded data

- [SINAD versus requested TX power](baselines/20261113-tx-power-sweep/sinad.svg)
- [RSSI versus requested TX power](baselines/20261113-tx-power-sweep/rssi.svg)
- [CSV: means, medians, ranges, noise RMS and error counters](baselines/20261113-tx-power-sweep/results.csv)
- [Structured summary](baselines/20261113-tx-power-sweep/summary.json)
- [Setup and binary hashes](baselines/20261113-tx-power-sweep/metadata.json)

The plot bars show observed minima/maxima, not uncertainty or confidence
intervals. SINAD means below are arithmetic averages of the reported dB values,
not a power-domain aggregate. A dagger marks a SINAD spread greater than 2 dB:
these averages include genuine unstable intervals and should not be interpreted
as clean receiver sensitivity measurements.

Each RX cell contains **mean RSSI dBm / mean estimated SINAD dB**.

| Requested TX dBm | PA code | Nominal input dBm* | RX 1 MS/s | RX 2 MS/s | RX 4 MS/s |
| ---: | ---: | ---: | ---: | ---: | ---: |
| -12 | 0 | -102 | -103.4 / 16.27 | -103.0 / 15.87 | -97.9 / 15.71 |
| -10 | 1 | -100 | -102.4 / 17.03 | -102.5 / 16.78 | -97.6 / 16.51 |
| -8 | 3 | -98 | -100.8 / 18.45 | -101.0 / 18.24 | -97.8 / 18.06 |
| -6 | 6 | -96 | -98.4 / 19.97 | -98.0 / 19.88 | -96.4 / 19.84 |
| -4 | 8 | -94 | -96.4 / 7.75 † | -96.5 / 21.02 | -95.4 / 20.74 |
| -2 | 10 | -92 | -94.4 / 2.32 † | -94.6 / 21.60 | -93.8 / 21.48 |
| +0 | 12 | -90 | -92.2 / 2.60 † | -92.8 / 22.07 | -92.0 / 22.07 |
| +2 | 14 | -88 | -90.6 / 4.26 † | -90.8 / 22.45 | -90.6 / 22.45 |
| +4 | 17 | -86 | -87.9 / 23.17 | -88.1 / 22.85 | -88.2 / 23.28 |
| +6 | 21 | -84 | -84.5 / 5.09 † | -84.7 / 22.90 | -84.5 / 23.53 |
| +8 | 29 | -82 | -79.6 / 10.26 † | -79.9 / 23.13 | -79.8 / 23.19 |
| +10 | 31 | -80 | -79.0 / 9.93 † | -79.1 / 22.66 | -79.1 / 22.57 |
| +12 | 31 | -78 | -78.9 / 9.63 † | -79.1 / 22.56 | -79.0 / 22.56 |
| +14 | 31 | -76 | -79.0 / 12.31 † | -79.0 / 22.57 | -79.0 / 22.50 |

\* Nominal input is requested TX power minus 90 dB. It is not calibrated input
power. In particular, the RF24 polynomial/clamping maps +10, +12 and +14 dBm
to the **same PA code 31**, with backend power estimate +8 dBm at all three
settings. Those requests do not represent three distinct physical output
levels. The flat RSSI at the top of the sweep is consistent with that mapping;
it is not evidence of receiver RSSI saturation. The backend's “actual” power
field is a model estimate, not an RF power-meter measurement.

At the lowest requested power, SINAD was about 16 dB at all RX rates. The 12 dB
point was not reached. At stronger levels the clean 2/4 MS/s estimates leveled
near 23 dB. The fixed 1 kHz notch's frequency sensitivity and transmitter
noise/distortion can limit that result; it is not an absolute receiver SINAD
ceiling. See [estimator definition and limitations](sinad.md).

## TX-off baselines

| RX rate | Mean raw RSSI dBm | Mean noise RMS | SINAD | Squelch |
| --- | ---: | ---: | --- | --- |
| 1 MS/s | -118.96 | 1.2151 | unavailable | closed |
| 2 MS/s | -107.52 | 1.2774 | unavailable | closed |
| 4 MS/s | -98.08 | 1.2754 | unavailable | closed |

Raw signed RSSI changes substantially with RX sample rate, including with TX
off. The observed baseline shift from 1 to 4 MS/s is about 21 dB, while the
post-demodulation noise RMS is similar. RX digital cutoff changes with rate
(125/250/500 kHz); the mechanism and calibration of this RSSI shift were not
established. Do not use these raw values as directly comparable calibrated
input levels across rates.

## Method and timestamp handling

Both backends used RF24 at requested 430.100 MHz and were launched with sudo,
obtaining realtime worker scheduling. TX source was a continuous 1000 Hz sine
with amplitude 0.4 and configured 2500 Hz deviation: nominal peak modulation
deviation 1000 Hz. RX used default 2500 Hz normalization, gain 8000, 50 us audio
de-emphasis, noise squelch thresholds 0.20/0.30 and direct Jabra playback
`plughw:USB,0,0`. Carrier squelch was off; SINAD observes audio before muting,
volume and de-emphasis. The original-board TX and modified-board RX roles
were not reversed. This is an RX-rate comparison at one fixed TX rate, not a
matrix of TX sample rates.

For each rate the receiver ran continuously while TX was stopped, configured
and restarted for each requested power. After TX reported steady state the
script waited two seconds, then polled both status streams for six seconds.
The TX restart produces deliberate brief gaps and Quindar cues between power
steps; these are separate from the reported rapid clicking within the tone.

Review found that a status socket can deliver an old queued packet as the
first measurement packet, after being unread during settling. Its timestamp
can predate the current measurement window. This explained isolated initial
low SINAD values in some 2/4 MS/s records. To apply a uniform rule independent
of measured values, analysis retains only RX snapshots in the **trailing five
seconds of each recorded point**, using the receiver's monotonic timestamps.
Every final point contains 25 status samples. This removes the queued entry
packet; it does not remove the repeated 1 MS/s disruptions within the retained
interval. The unfiltered originals are preserved alongside the filtered
summary. Future collection should reject stale packets at acquisition time.

Status is published at 5 Hz, whereas SINAD updates every 500 ms. Repeated
SINAD values in snapshots are not independent measurements. The SINAD number
is a 300–3000 Hz filtered, unweighted link estimate; tone-frequency offset,
uncalibrated RF power and fixed attenuation limit absolute sensitivity claims.
No IQ or audio waveform recording was made. A useful follow-up is a fixed-power
1/2/4 MS/s comparison with raw IQ/discriminator capture, preserving the rapid
fault rather than treating zero application counters as proof of continuity.

## Reproduction and provenance

The complete raw RX/TX snapshots, command/reply logs and backend logs are in
`build/radio-backend/tx-power-sweep-20261113/cariboulite-power-sweep-20261113T202050Z/`.
Original copies are under `/tmp/cariboulite-power-sweep-20261113T202050Z` on each
radio; these temporary copies are not permanent storage. The local build
artifact directory also contains separate RX/TX tar archives and the run log.
The initial startup-check attempt stopped RX before any TX was launched because
the collector expected STOPPED instead of IDLE; its diagnostic log is retained.

The [collector](baselines/20261113-tx-power-sweep/run_sweep.py),
[radio helper](baselines/20261113-tx-power-sweep/radio_helper.py) and
[analysis/plot script](baselines/20261113-tx-power-sweep/analyze.py) are preserved
as exact artifacts. The collector uses `/tmp/cariboulite-power-radio-helper.py`
on pi4f and launches the TX helper over pi4f's existing SSH access to pi4g.
It must only be used with both radios released and the wired attenuated setup
explicitly authorized. Analysis can be repeated without hardware:

```sh
python3 docs/baselines/20261113-tx-power-sweep/analyze.py   build/radio-backend/tx-power-sweep-20261113/cariboulite-power-sweep-20261113T202050Z   docs/baselines/20261113-tx-power-sweep
```

Both radios had matching backend and shared-library hashes. On-disk FPGA image
hashes also matched; these are file hashes, not an independent verification of
the loaded FPGA contents. See [release checks and supporting hashes](baselines/20261113-tx-power-sweep/release-and-hashes.txt).

## Subsequent diagnosis

The [fixed-TX and raw-data investigation](rx-stutter-diagnosis-2026-11-13.md)
identified the reproduced 1 MS/s clicks as false byte-alignment locks in the
SMI software decoder. Correct continuous alignment of the same raw bytes
restored clean SINAD, and a temporary decoder candidate passed fresh live
starts at nonzero byte phases. The earlier sweep values remain unchanged;
its unstable entries reflect this decoder fault rather than a calibrated
receiver sensitivity curve.
