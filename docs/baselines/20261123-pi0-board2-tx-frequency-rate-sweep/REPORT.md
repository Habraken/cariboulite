# Repaired CaribouLite device #2 TX frequency and sample-rate sweep

**23 November 2026 · 13:57–17:28 Europe/Brussels · pi0b TX → pi0a RX**

## Results

All **1,376 planned measurements** completed: **172 frequencies × eight matched TX/RX sample rates**, at requested and hardware-reported **0 dBm TX** through the user-confirmed nominal **90 dB attenuated RF24/HiF link**. Independent verification passed against **32,142 paired snapshots**, including eight preflight measurements. The main matrix contains **31,955 pairs**.

Numeric SINAD is available at **1,310/1,376 points**; **66 points have no numeric estimate**, and **7 additional windows are partially valid**. Missing SINAD remains blank in the CSV. **2 matrix windows** showed increases or resets in the selected transport counters.

The best transport-clean point mean is **25.02 dB SINAD at 50 MHz and 0.4 MS/s**. This describes the complete link and the backend estimator; it does not establish calibrated receiver sensitivity.

## Setup

| Item | Setting |
|---|---|
| TX | Repaired device #2 on `pi0b-trixie`, `192.168.6.69` |
| TX board UUID | `36b67ac4-b313-4369-99bd-4b3e09caabfa` |
| RX | Original reference board on `pi0a-trixie`, `192.168.6.37`; RX only |
| RX board UUID | `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` |
| TX oscillator | Correct 125 MHz oscillator fitted, as reported by the operator; no oscillator timing workaround |
| RF path | HiF/RF24 ports, nominal 90 dB attenuated conducted link |
| Frequency grid | 172 frequencies, 30 MHz–5,999.999 MHz; 50 MHz grid plus band anchors and path boundaries |
| Rates | 4, 2, 1.333333, 1, 0.8, 0.666667, 0.5, 0.4 MS/s; TX and RX changed together |
| TX signal | NBFM; 1 kHz sine, amplitude 0.4; deviation setting 2,500 Hz |
| Power | Requested and reported 0 dBm at every retained sample; no independent RF power calibration |
| Signalling | TX/RX CTCSS and CDCSS off |
| RX gain profile | `matched`, with `am=-7`, `usb=0`, `lsb=0` |
| RX controls | Carrier/noise squelch disabled; LED1 held low, original LED state restored on release |
| Reference clock drive | Standard 8 mA setting, no drive override or preload |
| Acquisition | 2 s settling, 5 s measurement per point; fresh sequences and monotonic timestamps on each host |
| SINAD | Backend NBFM 1 kHz estimate over 300–3,000 Hz |

Exactly 6 GHz is excluded by the radio API, so the highest setting is 5,999,999,000 Hz. The fractional native rates are 4 MHz divided by 3 and 6, with rounded integer command labels. RX analog bandwidth remains 160 kHz and TX analog bandwidth 80 kHz; digital cutoff changes with rate from 500 kHz to 50 kHz. Thus each sample-rate setting changes a complete profile on both boards.

The oscillator replacement was reported by the operator. This sweep checks functional operation with the standard driver, including 4 MS/s TX; it does not independently measure oscillator frequency or phase noise. The microphone compressor is enabled in configuration but is microphone-only, so it does not process this synthetic tone source.

## Frequency response

![RSSI and SINAD versus frequency](analysis/frequency-overview.png)

[Vector figure](analysis/frequency-overview.svg). Points are discrete measured settings. Connecting lines guide the eye; missing SINAD interrupts the curves. Dotted lines mark the 2,385 and 2,495 MHz RF path boundaries. Black crosses identify transport-flagged windows.

Representative means at 1 MS/s:

| Frequency MHz | RSSI dBm | Estimated SINAD dB |
|---:|---:|---:|
| 30.000 | -90.86 | 23.54 |
| 430.100 | -93.26 | 22.03 |
| 1000.000 | -99.96 | 14.82 |
| 1350.000 | -100.22 | 16.95 |
| 1750.000 | -105.61 | 11.39 |
| 2000.000 | -110.96 | 7.58 |
| 2384.900 | -88.48 | Unavailable |
| 2385.000 | -107.52 | 15.84 |
| 2399.900 | -106.35 | 15.41 |
| 2400.000 | -105.42 | Unavailable |
| 2494.900 | -108.25 | 14.37 |
| 2495.000 | -106.79 | 9.85 |
| 2650.000 | -58.00 | Unavailable |
| 4650.000 | -83.86 | 20.99 |
| 5999.999 | -94.54 | 13.37 |

## Frequency and rate maps

![Frequency and rate heatmaps](analysis/frequency-rate-heatmaps.png)

[Vector figure](analysis/frequency-rate-heatmaps.svg). Frequency cells follow the actual nonuniform grid, with edges at neighboring midpoints. Rates are discrete rows. Gray SINAD cells indicate missing estimates; colors are not interpolated.

## Sample-rate summary

| Rate MS/s | Numeric SINAD / 172 | Transport flags | Median RSSI dBm | Median numeric SINAD dB | Points ≥12 dB | Median paired SINAD Δ vs 1 MS/s dB |
|---:|---:|---:|---:|---:|---:|---:|
| 4 | 163 | 0 | -91.33 | 16.15 | 121 | -0.12 |
| 2 | 164 | 1 | -94.06 | 17.04 | 121 | -0.10 |
| 1.333333 | 164 | 0 | -94.31 | 17.14 | 124 | +0.20 |
| 1 | 164 | 0 | -94.17 | 16.25 | 122 | +0.00 |
| 0.8 | 164 | 0 | -93.95 | 17.15 | 126 | +0.28 |
| 0.666667 | 164 | 1 | -93.79 | 17.10 | 124 | +0.30 |
| 0.5 | 164 | 0 | -94.15 | 16.91 | 124 | +0.22 |
| 0.4 | 163 | 0 | -93.99 | 17.24 | 124 | +0.42 |

Among **160 frequencies** with fully valid SINAD and clean transport at every rate, the median range between rate-specific means is **0.81 dB**, with a maximum of **12.91 dB**. The sweep measured one full frequency pass per rate, sequentially; time and temperature are therefore not separated from rate effects.

Medians weight every measured frequency equally, including clustered anchors. Numeric-SINAD medians exclude missing estimates. Paired differences retain only fully valid, transport-clean windows at both rates; counts are in [rate-summary.csv](analysis/rate-summary.csv). The ≥12 dB count is descriptive, not a sensitivity specification.

![Rate profiles at selected frequencies](analysis/rate-profiles.png)

[Vector figure](analysis/rate-profiles.svg). TX and RX rates change together; these profiles cannot assign a change to one board.

The largest rate-dependent RSSI range is **21.43 dB at 2200 MHz**, spanning **-113.26 to -91.83 dBm**. Numeric SINAD means at this frequency span **6.94–7.76 dB**. A change in reported RSSI alone does not establish an equal change in wanted-signal input power.

## RF path boundaries

![Path boundary detail](analysis/path-boundary-zooms.png)

[Vector figure](analysis/path-boundary-zooms.svg). These zooms show the closely spaced measurements around the native-modem/mixer boundaries. Any coincident step remains a combined TX/RX link observation.

## Missing and partially valid SINAD

An unavailable estimate is not zero SINAD and does not establish that no RF signal was received. RSSI can include interference and conversion artifacts. Only finite numeric estimates contribute to the reported SINAD statistics.

| Frequency MHz | Rates with no numeric SINAD, MS/s |
|---:|---|
| 2300 | All eight |
| 2300.1 | All eight |
| 2320.1 | All eight |
| 2350 | All eight |
| 2360 | All eight |
| 2384.9 | All eight |
| 2400 | All eight |
| 2450 | 0.4 |
| 2650 | All eight |
| 3150 | 4 |

| Frequency MHz | Rate MS/s | Numeric snapshots / total |
|---:|---:|---:|
| 3150 | 2 | 3 / 24 |
| 3150 | 1.333333 | 21 / 24 |
| 3150 | 1 | 15 / 21 |
| 3150 | 0.8 | 19 / 23 |
| 3150 | 0.666667 | 19 / 20 |
| 3150 | 0.5 | 18 / 24 |
| 3150 | 0.4 | 9 / 24 |

## Acquisition integrity

The collector recorded **10 failed setup attempts**. A failed setup triggers a clean helper restart and one retry; failed partial windows are not retained as completed points. The matrix verifier checks every planned point is present exactly once.

**2 matrix windows** have transport flags. Flags are recomputed from every retained raw counter value, including checks for resets. They cover RX queue/audio drops, playback underruns/write errors, reader errors, alignment failures/invalid frames, and TX driver underruns, writer errors and queue drops.

Whole-session histories include startup, tuning and settling, so their cumulative counters can differ from the clean-window result:

- RX `alignment.invalid_frames`: peak **7**, total positive increments **14**.
- TX `driver.underrun_periods`: peak **3**, total positive increments **3**.

Independent checks verify host/board identities, RX-only operation, requested frequency/rate, requested and reported power, signalling off, tone settings, fresh sample sequences, RSSI/SINAD summary statistics, CSV consistency, and the complete 172 × 8 matrix. The receiver command audit found **zero TX commands**, and both backend release checks passed. Details are in [verification.json](verification.json) and [matrix-verification.json](matrix-verification.json).

All eight 430.1 MHz preflight measurements passed:

| Rate MS/s | Mean RSSI dBm | Mean SINAD dB |
|---:|---:|---:|
| 4 | -89.46 | 22.32 |
| 2 | -91.54 | 22.73 |
| 1.333333 | -92.00 | 23.44 |
| 1 | -92.14 | 22.67 |
| 0.8 | -92.26 | 23.54 |
| 0.666667 | -92.29 | 23.39 |
| 0.5 | -92.25 | 23.16 |
| 0.4 | -92.50 | 23.58 |

Transport-flagged measurement windows:

| Point | Frequency MHz | Rate MS/s | Counter changes |
|---:|---:|---:|---|
| 273 | 2800 | 2 | rx.playback.underruns: Δ=1, reset=False |
| 975 | 3405 | 0.666667 | rx.playback.underruns: Δ=1, reset=False |

Recovered setup attempts:

| Matrix point | Frequency MHz | Rate MS/s | Failed attempt |
|---:|---:|---:|---:|
| 103 | 2900 | 4 | 1 |
| 260 | 2425 | 2 | 1 |
| 302 | 4100 | 2 | 1 |
| 506 | 5650.1 | 1.333333 | 1 |
| 559 | 900 | 1 | 1 |
| 565 | 1050 | 1 | 1 |
| 868 | 51.9 | 0.666667 | 1 |
| 1005 | 4850 | 0.666667 | 1 |
| 1037 | 50 | 0.5 | 1 |
| 1079 | 950 | 0.5 | 1 |

Exact fault messages are retained in [progress.json](progress.json) and [controller.log](controller.log), with each backend log in the host archives. Retried points contribute only their successful complete measurement window. A recovered startup failure remains an acquisition issue even when that window is clean; it is not classified as a measured SINAD or transport failure.

## Interpretation limits

These values characterize the complete transmitter–attenuator–receiver link. Requested power and nominal attenuation do not establish actual receiver input power over frequency. This run therefore does not measure receiver sensitivity, noise figure, or calibrated standalone TX/RX frequency response.

The 21 November baseline used the original pi0a board as TX and reconstructed board #1 as RX on pi0b. This run reverses the roles and uses repaired device #2 on pi0b, with the current backend. Differences between those runs cannot be assigned solely to the oscillator repair.

## Data and regeneration

- [Point measurements CSV](results.csv), [full point summaries](results.jsonl), and [test plan](plan.json).
- [Acquisition report](REPORT.txt), [host/snapshot verification](verification.json), and [matrix verification](matrix-verification.json).
- [Paired raw snapshots](paired-snapshots.jsonl.gz), [TX host archive](tx-host-artifacts.tar.gz), and [RX host archive](rx-host-artifacts.tar.gz).
- [Analysis summary](analysis/analysis-summary.json), [rate summary](analysis/rate-summary.csv), and [analysis source](analysis/analyze_sweep.py).

With Python, NumPy and Matplotlib installed, run from the baseline directory:

```sh
python3 verify_matrix.py .
MPLCONFIGDIR=/tmp/cariboulite-matplotlib python3 analysis/analyze_sweep.py .
```

The acquisition sources preserve original deployment addresses and key paths as historical records. Review them before reuse; running acquisition code transmits RF. Offline verification and analysis do not access hardware.

The source manifest preserves the original completion checksums. The package manifest covers the archived evidence, and the analysis manifest covers the generated report and figures.
