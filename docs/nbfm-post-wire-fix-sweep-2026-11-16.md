# NBFM sample-rate and TX-power retest after TX wire fixes

The restored **90 dB nominal attenuated link** connects pi4g-trixie as transmitter to pi4f-trixie as receiver at **430.100 MHz, RF24**. All eight native RX rates and all eight native TX rates passed the recorded power sweep, with zero recorded transport errors. This is 224 transmitting points and 16 TX-off baselines, rather than an 8-by-8 matrix of rate pairs.

## What changed in measured performance

This repeat does **not establish a NBFM SINAD improvement** over the [historical sweep](native-sample-rate-power-sweep-2026-11-14.md). The shared TX wire corrections remain necessary for sample accuracy, but their benefit cannot be inferred from a small change in this fixed-notch link-quality estimate.

For the RX-rate axis, current minus historical mean SINAD spans **-1.02 to +0.08 dB**, averaging **-0.30 dB** across all 112 transmitting points. These averages include the repeated PA-code-31 requests and are descriptive, not independent observations or causal estimates.

For the TX-rate axis, current minus historical mean SINAD spans **-0.89 to -0.06 dB**, averaging **-0.40 dB** across all 112 transmitting points. These averages include the repeated PA-code-31 requests and are descriptive, not independent observations or causal estimates.

Within this run, the two separately measured 4 MS/s reference curves differ by up to **0.90 dB SINAD** and **1.00 dB RSSI** at matching requests. The largest observed within-point SINAD range is **2.077 dB**. Neither quantity is a calibrated uncertainty bound.

Intervening changes include the TX byte-order correction, preservation of the low I-data bit, RX playback calibration, AM support and the corrected driver quiet-frame encoding. The SINAD implementation is unchanged, reads the raw discriminator before de-emphasis, volume and squelch, and AM DAC shaping is disabled throughout this NBFM run. Separate dates, calibration/startup state, temperature and the reconnected attenuation path prevent attributing differences solely to an individual fix.

The fixed 1 kHz notch can leave residual fundamental energy from small tone/clock offsets. The measured strong-signal plateau therefore does not establish the RF distortion limit or receiver sensitivity. See the [estimator definition](sinad.md).

## Figures and complete data

- [SINAD curves](baselines/20261116-nbfm-post-wire-fix/sinad.svg)
- [RSSI curves](baselines/20261116-nbfm-post-wire-fix/rssi.svg)
- [Change in SINAD from historical sweep](baselines/20261116-nbfm-post-wire-fix/sinad_db-delta.svg)
- [Change in RSSI from historical sweep](baselines/20261116-nbfm-post-wire-fix/rssi_dbm-delta.svg)
- [All current point statistics, CSV](baselines/20261116-nbfm-post-wire-fix/results.csv)
- [Matched historical/current comparison, CSV](baselines/20261116-nbfm-post-wire-fix/comparison.csv)
- [Setup and exact profiles](baselines/20261116-nbfm-post-wire-fix/metadata.json)

Bands show observed minima and maxima, not confidence intervals. Statistics average reported dB values arithmetically. Status snapshots at 5 Hz can repeat the estimator’s 500 ms windows and are not independent samples.

## Selected measured points

Cells show **mean raw RSSI dBm / mean estimated SINAD dB**. Requested powers are not power-meter readings.

### RX rates; TX fixed at 4 MS/s

| Native rate | −12 dBm request | 0 dBm request | +8 dBm request |
| --- | ---: | ---: | ---: |
| 4 MS/s | -97.8 / 16.00 | -92.1 / 21.28 | -79.7 / 22.67 |
| 2 MS/s | -102.8 / 15.57 | -93.1 / 21.29 | -79.8 / 22.08 |
| 4/3 MS/s | -104.2 / 15.47 | -93.2 / 21.11 | -79.9 / 21.94 |
| 1 MS/s | -104.7 / 15.29 | -93.4 / 20.75 | -80.2 / 21.61 |
| 800 kS/s | -104.4 / 15.36 | -93.4 / 20.77 | -80.1 / 21.49 |
| 2/3 MS/s | -104.4 / 15.33 | -93.5 / 20.73 | -80.1 / 21.35 |
| 500 kS/s | -104.4 / 15.37 | -93.5 / 20.65 | -80.2 / 21.39 |
| 400 kS/s | -104.9 / 15.41 | -93.4 / 20.65 | -80.1 / 21.40 |

### TX rates; RX fixed at 4 MS/s

| Native rate | −12 dBm request | 0 dBm request | +8 dBm request |
| --- | ---: | ---: | ---: |
| 4 MS/s | -97.6 / 15.33 | -93.0 / 20.77 | -80.2 / 21.77 |
| 2 MS/s | -97.7 / 15.36 | -92.0 / 20.92 | -80.2 / 21.52 |
| 4/3 MS/s | -97.5 / 15.38 | -92.5 / 21.41 | -80.0 / 22.24 |
| 1 MS/s | -97.5 / 15.19 | -92.3 / 21.32 | -80.2 / 22.10 |
| 800 kS/s | -98.0 / 15.41 | -92.5 / 21.48 | -80.1 / 22.15 |
| 2/3 MS/s | -97.6 / 15.45 | -93.0 / 21.37 | -80.2 / 22.12 |
| 500 kS/s | -97.6 / 15.38 | -93.3 / 21.36 | -80.0 / 22.21 |
| 400 kS/s | -98.0 / 15.43 | -92.4 / 21.44 | -80.1 / 22.17 |

## Isolated low-power dip and follow-up

Point 167 (TX 1 MS/s, RX 4 MS/s, -12 dBm request) briefly reported 13.388 dB SINAD within a 15.192 dB mean window. No transport error accompanied it. This point remains in all original statistics and plots. Three subsequent repetitions using the same settling/measurement protocol returned means of 15.706, 15.643 and 15.611 dB, with respective minima of 15.532, 15.552 and 15.517 dB. The dip was not reproduced; its cause is unresolved. These are supplementary observations, not replacements for the original point. See [recheck results](baselines/20261116-nbfm-post-wire-fix/recheck-results.json).

## Method and validation

TX tone: 1 kHz, amplitude 0.4, 2500 Hz deviation setting (nominal 1000 Hz peak deviation). RX: NBFM, 50 microsecond de-emphasis, legacy trim 8000, SINAD enabled, Jabra USB playback. TX power requests run from -12 through +14 dBm in 2 dB steps. Each block contains an off baseline. The other direction stays at 4 MS/s. Every power transition stops, configures and restarts TX; two seconds of settling precede five seconds of measurement. Startup/tail cues are excluded.

Each retained RX snapshot must be newer than the measurement start and have a fresh sequence. RSSI is the signed raw RF24 register. The profile verifier checks active tone/mode/rates and native hardware filter settings on every transmitting snapshot. The collector aborts on a radio-state/transport fault rather than discarding the failed point.

All **5767 retained snapshots** reconcile independently to the point statistics. Every transmitting point has valid SINAD throughout its retained window. RX alignment failures/invalid frames, reader errors, IQ/audio drops and playback underruns/write errors stayed zero, as did TX queue drops, writer errors and driver underruns. This verifies the measured windows, not indefinite uptime or every hardware initial condition.

+10, +12 and +14 dBm requests map to PA code 31 and repeat the same physical setting. Requested power minus 90 dB is nominal input only. Raw RSSI noise floors depend on RX rate, so cross-rate RSSI is not a calibrated sensitivity comparison. The 12 dB SINAD point was not reached. No calibrated receiver sensitivity, complete THD, spectral compliance or causal single-fix effect is established.

Both backend binaries match SHA256 `25a9b8cf2c764fc4945e2fc5facb4b1143bf3bad9b7d606abc997cc4d3e390b9` at implementation commit `3edde8e`; both loaded/installed drivers have srcversion `1AC65CD6AE81A1E2ED63F14`. FPGA images and production settings were not modified. Both radios were stopped and released after collection.

Collection began at 2026-11-15 23:11:39 UTC, which is 16 November in Europe/Brussels. The UTC run ID is retained in the artifacts. See the [baseline README](baselines/20261116-nbfm-post-wire-fix/README.md) for raw data locations, reproduction and verification.
