# Native sample-rate and TX power sweeps after continuity fixes

The original CaribouLite on **pi4g-trixie** transmitted a 1 kHz NBFM tone
through the user's wired connection with **90 dB nominal attenuation** to
**pi4f-trixie**, at **430.100 MHz**. The Jabra USB speaker was used for playback.

Two sweeps isolate the directions: all eight RX rates with TX fixed at 4 MS/s,
then all eight TX rates with RX fixed at 4 MS/s. Each block includes TX off
and requested TX power −12 through +14 dBm in 2 dB steps. This gives
**224 transmitting points and 16 TX-off baselines**. The 4 MS/s–4 MS/s curve
is measured independently in both sweeps; this is not an 8-by-8 rate matrix.

**Every recorded transport error counter stayed zero.** The largest within-point SINAD spread was **0.863 dB**. At the lowest requested power, mean SINAD across the tested rate pairs was **15.49–16.10 dB**.

The alignment fix, burst-aware playback buffer and FIFO-preserving TX restart
were in place for this run. The user reported clean tones after the driver
change. These results cover the recorded runs, not indefinite uptime or every
possible startup condition.

## Figures and complete data

- [SINAD curves](baselines/20261114-native-power-sweep/sinad.svg)
- [RSSI curves](baselines/20261114-native-power-sweep/rssi.svg)
- [All point statistics and error counters, CSV](baselines/20261114-native-power-sweep/results.csv)
- [Structured summary](baselines/20261114-native-power-sweep/summary.json)
- [Setup and native filter profiles](baselines/20261114-native-power-sweep/metadata.json)

Curve bands show observed minima/maxima, not confidence intervals. Means of
reported dB values are arithmetic averages, not power-domain aggregates.
Status is sampled at 5 Hz while SINAD updates at 2 Hz; repeated snapshots are
not independent measurements. All recorded values are retained, with no
filter based on the measured SINAD value.

## Selected points

Each cell is **mean raw RSSI dBm / mean estimated SINAD dB**. The CSV contains
every power step; these tables highlight the weakest input, 0 dBm request and
+8 dBm request.

### RX rates, TX fixed at 4 MS/s

| Native rate | −12 dBm TX | 0 dBm TX | +8 dBm TX |
| --- | ---: | ---: | ---: |
| 4 MS/s | -98.0 / 16.10 | -92.0 / 21.87 | -79.5 / 23.00 |
| 2 MS/s | -103.2 / 15.86 | -92.8 / 21.52 | -79.5 / 22.25 |
| 4/3 MS/s | -104.1 / 15.78 | -93.1 / 21.25 | -79.9 / 22.08 |
| 1 MS/s | -104.2 / 15.50 | -93.1 / 20.93 | -79.9 / 21.93 |
| 800 kS/s | -104.3 / 15.58 | -93.1 / 21.03 | -80.0 / 22.23 |
| 2/3 MS/s | -104.3 / 15.49 | -93.2 / 20.99 | -79.8 / 22.07 |
| 500 kS/s | -104.6 / 15.54 | -93.0 / 21.00 | -80.0 / 22.11 |
| 400 kS/s | -104.4 / 15.63 | -93.1 / 21.00 | -80.0 / 22.01 |

### TX rates, RX fixed at 4 MS/s

| Native rate | −12 dBm TX | 0 dBm TX | +8 dBm TX |
| --- | ---: | ---: | ---: |
| 4 MS/s | -98.0 / 15.55 | -92.2 / 21.25 | -79.8 / 22.40 |
| 2 MS/s | -98.2 / 15.58 | -92.6 / 21.38 | -80.0 / 22.30 |
| 4/3 MS/s | -97.2 / 15.66 | -92.5 / 21.91 | -79.8 / 22.65 |
| 1 MS/s | -97.7 / 15.63 | -92.2 / 21.79 | -79.7 / 22.66 |
| 800 kS/s | -98.2 / 15.62 | -92.2 / 21.98 | -79.8 / 22.53 |
| 2/3 MS/s | -97.6 / 15.62 | -92.3 / 21.87 | -80.0 / 22.88 |
| 500 kS/s | -97.8 / 15.65 | -92.4 / 21.84 | -80.0 / 22.55 |
| 400 kS/s | -97.5 / 15.66 | -92.8 / 21.62 | -80.0 / 22.53 |

## Power and sensitivity limits

Requested power minus 90 dB is nominal input, not a calibrated RF level.
The RF24 mapping sends +10, +12 and +14 dBm requests to the same PA code 31;
those are repeated physical settings. The backend's actual-power field is
an estimate from its model, not a power-meter measurement. The CSV records
requested power, observed PA code and that model estimate separately.

The 12 dB SINAD point was not reached with the available attenuation and TX power range. No absolute receiver sensitivity is established.

The estimator measures the 300–3000 Hz, unweighted link with a fixed 1 kHz
notch. Transmitter noise/distortion, tone-frequency error, filtering and the
uncalibrated RF path limit interpretation. The stronger-signal plateau is not
an absolute receiver SINAD ceiling. See [estimator definition](sinad.md).

## TX-off baselines and RSSI

| Sweep | Variable native rate | Mean RSSI (dBm) | Mean noise RMS | Gate open fraction |
| --- | --- | ---: | ---: | ---: |
| RX | 4 MS/s | -98.04 | 1.2822 | 0.000 |
| RX | 2 MS/s | -107.36 | 1.2875 | 0.000 |
| RX | 4/3 MS/s | -117.64 | 1.2810 | 0.000 |
| RX | 1 MS/s | -117.79 | 1.1955 | 0.000 |
| RX | 800 kS/s | -116.80 | 1.2687 | 0.000 |
| RX | 2/3 MS/s | -119.08 | 1.2902 | 0.000 |
| RX | 500 kS/s | -119.00 | 1.2600 | 0.000 |
| RX | 400 kS/s | -120.04 | 1.2757 | 0.000 |
| TX | 4 MS/s | -97.96 | 1.2902 | 0.000 |
| TX | 2 MS/s | -98.08 | 1.2822 | 0.000 |
| TX | 4/3 MS/s | -98.12 | 1.2826 | 0.000 |
| TX | 1 MS/s | -98.08 | 1.2838 | 0.000 |
| TX | 800 kS/s | -98.04 | 1.2690 | 0.000 |
| TX | 2/3 MS/s | -98.21 | 1.2739 | 0.000 |
| TX | 500 kS/s | -98.00 | 1.2748 | 0.000 |
| TX | 400 kS/s | -98.00 | 1.2782 | 0.000 |

Raw RSSI noise floors depend strongly on RX rate/filter settings even with TX
off. They should not be compared across rates as calibrated input levels.
Post-demodulation noise RMS provides a different measurement. This test does
not establish the mechanism or calibration of the RSSI offset.

## Repeated 4 MS/s reference

The independently repeated 4 MS/s–4 MS/s curves differed by at most 0.698 dB in mean SINAD and 0.792 dB in mean RSSI at matching requests. These are temporal comparisons, not calibrated uncertainty estimates.

## Collection and validation

The TX tone amplitude was 0.4 with a 2500 Hz deviation setting: nominal peak
modulation deviation 1000 Hz. RX used NBFM, 2500 Hz audio normalization,
50 µs de-emphasis, gain 8000 and noise squelch thresholds 0.20/0.30; carrier
squelch was off. SINAD observes the audio before squelch muting and volume.
All native clocks use exact 4 MHz/divider arithmetic; fractional-Hz command
labels are rounded. Profile details are in the metadata.

For each power step, TX was stopped, configured and restarted. Status was
drained throughout two seconds of settling, then recorded for five seconds.
Only RX snapshots with timestamps newer than the measurement start were
accepted. Intentional TX power-transition gaps and Quindar cues are outside
the measurement windows. Every point has at least 20 fresh RSSI snapshots;
every transmitting point has valid SINAD throughout its retained window.

The collector checked cumulative RX alignment failures/invalid frames, RX
read errors, IQ/audio queue drops, ALSA underruns/write errors, TX queue drops,
TX writer errors and driver underruns. All were zero. It would stop on a
transport fault or an unexpected TX/RX state rather than discard that point.
SINAD spread itself was retained and summarized independently of those checks.

The source manifest matched the committed implementation on pi4f-trixie.
Both radios used the same installed FIFO-preserving driver image, retained
rollback modules and their existing driver parameters. Backend, library,
controller, test application and Soapy builds were refreshed. No backend
startup service was added.

## Provenance and reproduction

Implementation commits:

- `8618ddd`: streaming RX framing and fragmented-sample preservation.
- `be0a0b0`: all native RX rates and DMA-burst-aware playback buffering.
- `24f1411`: preserve TX IQ at finite SMI-window restarts.
- `1e4948c`: all native TX rates and exact fractional frame pacing.

See the [alignment proof](alignment-fix-validation-2026-11-13.md),
[RX-rate validation](rx-native-rates-validation-2026-11-13.md),
[TX restart comparison](tx-window-restart-validation-2026-11-14.md) and
[TX-rate validation](tx-native-rates-validation-2026-11-13.md).

The collector, radio helper, source manifest and analysis/report scripts are
committed alongside the data. Full per-snapshot records and backend logs are
retained under `build/radio-backend/implementation-20261113/`; temporary
originals remain on the radios under the run directory recorded in metadata.
Both backends/controllers were stopped and playback was closed after the run.

Independent analysis reconciled all **5760 saved snapshots** with the point
summaries, including statistics, state/profile/tone settings and monotonic
timestamp order. [Verification record](baselines/20261114-native-power-sweep/raw-verification.json)
and [final hardware-release check](baselines/20261114-native-power-sweep/release-check.jsonl)
are committed with the results. No radio application or SMI owner remained,
and every recorded playback substream was closed on both hosts.
