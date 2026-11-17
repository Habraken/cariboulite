# Board #4 — RX qualification with LED1=0 — 16–17 November 2026

The original **430.1 MHz sweep on 16 November** is retained below. Additional **1240.1 MHz and 144.1 MHz sweeps on 17 November** are recorded in later sections, with separate raw data and plots.

Scoped RX sweep outcome: **passed**. 120 points recorded; 0 points with nonzero cumulative transport counters; 0 rate startup failures.

Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f-trixie. Original pi4g transmitter UUID: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`. No EEPROM rewrite or TX test on #4.

## Setup and observations

This run measures #4 across all eight native RX rates with explicit LED1=0 and LED0 preserved at1. It does not establish the cause of the LED1 dependency or qualify the oscillator.

User-requested pi4f reboot immediately before the sweep; no manual test app or LED menu initialization afterward. Each RX helper explicitly preserves LED0 and sets LED1 low through backend commands.

The original TX reference remained at 4 MS/s, RF24 430.100 MHz, 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. Each RX rate has TX off plus fourteen power requests from -12 to +14 dBm, two seconds settling and five seconds measurement. The user confirmed the nominal 90 dB attenuated link; it was not independently calibrated. Production DSP, driver and FPGA software were unchanged.

| RX rate | Points | Largest invalid-frame counter | Largest other RX transport counter |
| --- | ---: | ---: | ---: |
| 4 MS/s | 15 | 0 | 0 |
| 2 MS/s | 15 | 0 | 0 |
| 4/3 MS/s | 15 | 0 | 0 |
| 1 MS/s | 15 | 0 | 0 |
| 800 kS/s | 15 | 0 | 0 |
| 2/3 MS/s | 15 | 0 | 0 |
| 500 kS/s | 15 | 0 | 0 |
| 400 kS/s | 15 | 0 | 0 |

Cells below are mean raw RSSI dBm / estimated SINAD dB.

| RX rate | -12 dBm TX | 0 dBm TX | +8 dBm TX |
| --- | ---: | ---: | ---: |
| 4 MS/s | -93.1 / 18.18 | -87.4 / 23.92 | -75.6 / 26.09 |
| 2 MS/s | -99.5 / 17.73 | -89.2 / 24.24 | -76.1 / 25.60 |
| 4/3 MS/s | -100.5 / 17.60 | -89.4 / 24.04 | -76.3 / 25.13 |
| 1 MS/s | -100.8 / 17.48 | -89.6 / 23.33 | -76.2 / 25.05 |
| 800 kS/s | -100.8 / 17.49 | -89.6 / 23.30 | -76.3 / 24.98 |
| 2/3 MS/s | -100.9 / 17.47 | -89.6 / 23.18 | -76.4 / 24.76 |
| 500 kS/s | -101.1 / 17.39 | -89.8 / 23.25 | -76.5 / 24.78 |
| 400 kS/s | -101.0 / 17.50 | -89.8 / 23.33 | -76.7 / 24.60 |

Before every RX rate, the helper reads the digital register, preserves LED0 and issues an explicit backend LED1=0 write with checked readback. These commands are retained in the raw logs. This is an intentional test condition, not an automatic production initialization change. See led-verification.json for all eight cases.

## Evidence and limits

All 2879 retained measurement snapshots independently reconcile with the summaries. Settings, role UUIDs, rates, tone, filters and transport counters were inspected. Commands and backend states were checked for absence of TX operation on #4. Both radios were stopped and released afterward.

The initial failures and previous LED-state experiments remain separate evidence. This run explicitly sets LED1 low and does not establish the cause of the LED1 dependency. No test app was used. No oscillator-frequency measurement was made. These short, uncalibrated RX link tests do not establish oscillator or TX acceptance, calibrated sensitivity, or indefinite continuity. +10/+12/+14 dBm requests repeat PA code 31.

[Raw evidence and reproduction](baselines/20261116-board4-led1-low-rx/README.md) · [RSSI](baselines/20261116-board4-led1-low-rx/rssi.svg) · [SINAD](baselines/20261116-board4-led1-low-rx/sinad.svg) · [CSV](baselines/20261116-board4-led1-low-rx/results.csv)

## Additional qualification at 1240.1 MHz — 17 November 2026

The same RX-rate/TX-power sweep at **1240.1 MHz** **passed**: **120 points**, **112 transmitting points with fully valid SINAD**, and **0 points with nonzero transport counters**.

Receiver #4 remained on pi4f-trixie; the original CaribouLite on pi4g-trixie transmitted through the user-confirmed nominal 90 dB coax link. LED0 was preserved and LED1 explicitly set to 0 with checked readback before each RX rate. No reboot, EEPROM rewrite or test-app initialization was requested or performed for this sweep. No TX trial was run on #4.

TX stayed at 4 MS/s, RF24 1240.100 MHz, with the same 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. Each of the eight native RX rates has a TX-off baseline and fourteen power requests from -12 to +14 dBm in 2 dB steps. Settling and measurement times remained 2 seconds and 5 seconds.

Cells show **mean raw RSSI dBm / mean estimated SINAD dB**.

| RX rate | -12 dBm TX request | 0 dBm TX request | +8 dBm TX request |
| --- | ---: | ---: | ---: |
| 4 MS/s | -93.8 / 11.26 | -93.0 / 17.68 | -83.5 / 19.34 |
| 2 MS/s | -102.7 / 10.91 | -96.1 / 17.48 | -83.8 / 19.01 |
| 4/3 MS/s | -108.5 / 10.76 | -96.9 / 17.07 | -83.6 / 18.61 |
| 1 MS/s | -107.7 / 10.59 | -97.3 / 16.79 | -83.7 / 18.52 |
| 800 kS/s | -108.5 / 10.51 | -97.0 / 16.74 | -83.8 / 18.42 |
| 2/3 MS/s | -108.0 / 10.53 | -97.5 / 16.70 | -83.7 / 18.36 |
| 500 kS/s | -108.3 / 10.54 | -97.4 / 16.61 | -83.8 / 18.47 |
| 400 kS/s | -108.3 / 10.60 | -97.5 / 16.61 | -83.8 / 18.27 |

### Verification and comparison

All **2877 retained measurement snapshots** independently reconcile with the point summaries. Mode/tone, requested/native rate readbacks, modem filters, board roles and transport counters were verified. LED setup checks cover all **8 RX rate starts**. Both radios were stopped and released afterward.

Raw RSSI, 1240.1 MHz minus 430.1 MHz on the same board, averaged **-7.11 dB** across 112 matched transmitting settings (range -8.51 to -0.25 dB).
Estimated SINAD, 1240.1 MHz minus 430.1 MHz on the same board, averaged **-6.52 dB** across 112 matched transmitting settings (range -7.24 to -5.38 dB).

These are end-to-end link differences, not isolated frequency-dependent receiver sensitivity. TX/mixer output, frontend gain, cables and attenuator response can vary with frequency; the nominal 90 dB link was not independently calibrated at either frequency. Fixed-notch SINAD also includes transmitter distortion and tone/clock-offset effects. +10/+12/+14 requests repeat PA code 31. The LED1 dependency and oscillator frequency remain unresolved and unmeasured respectively.

[All 1240.1 MHz results](baselines/20261117-board4-1240p1-rx/results.csv) · [RSSI plots](baselines/20261117-board4-1240p1-rx/rssi.svg) · [SINAD plots](baselines/20261117-board4-1240p1-rx/sinad.svg) · [Matched 430.1 MHz comparison](baselines/20261117-board4-1240p1-rx/comparison-430p1.csv) · [Raw evidence and reproduction](baselines/20261117-board4-1240p1-rx/README.md)

## Additional qualification at 144.1 MHz — 17 November 2026

The same RX-rate/TX-power sweep at **144.1 MHz** **passed**: **120 points**, **112 transmitting points with fully valid SINAD**, and **0 points with nonzero transport counters**.

Receiver #4 remained on pi4f-trixie; the original CaribouLite on pi4g-trixie transmitted through the user-confirmed nominal 90 dB coax link. LED0 was preserved and LED1 explicitly set to 0 with checked readback before each RX rate. No reboot, EEPROM rewrite or test-app initialization was requested or performed for this sweep. No TX trial was run on #4.

TX stayed at 4 MS/s, RF24 144.100 MHz, with the same 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. Each of the eight native RX rates has a TX-off baseline and fourteen power requests from -12 to +14 dBm in 2 dB steps. Settling and measurement times remained 2 seconds and 5 seconds.

Cells show **mean raw RSSI dBm / mean estimated SINAD dB**.

| RX rate | -12 dBm TX request | 0 dBm TX request | +8 dBm TX request |
| --- | ---: | ---: | ---: |
| 4 MS/s | -93.2 / 12.96 | -83.8 / 21.77 | -70.8 / 26.04 |
| 2 MS/s | -95.2 / 12.38 | -84.6 / 21.67 | -71.4 / 25.13 |
| 4/3 MS/s | -96.0 / 12.02 | -85.0 / 21.43 | -71.6 / 25.19 |
| 1 MS/s | -96.6 / 11.91 | -85.1 / 21.09 | -72.1 / 24.92 |
| 800 kS/s | -97.0 / 11.77 | -85.0 / 21.03 | -71.6 / 24.69 |
| 2/3 MS/s | -95.7 / 11.77 | -85.1 / 20.90 | -71.8 / 24.70 |
| 500 kS/s | -96.3 / 11.68 | -85.1 / 21.05 | -71.7 / 24.42 |
| 400 kS/s | -97.0 / 11.74 | -85.2 / 20.89 | -71.8 / 24.91 |

### Verification and comparison

All **2877 retained measurement snapshots** independently reconcile with the point summaries. Mode/tone, requested/native rate readbacks, modem filters, board roles and transport counters were verified. LED setup checks cover all **8 RX rate starts**. Both radios were stopped and released afterward.

Raw RSSI, 144.1 MHz minus 430.1 MHz on the same board, averaged **+4.42 dB** across 112 matched transmitting settings (range -0.08 to +5.17 dB).
Estimated SINAD, 144.1 MHz minus 430.1 MHz on the same board, averaged **-2.28 dB** across 112 matched transmitting settings (range -5.76 to +0.74 dB).

These are end-to-end link differences, not isolated frequency-dependent receiver sensitivity. TX/mixer output, frontend gain, cables and attenuator response can vary with frequency; the nominal 90 dB link was not independently calibrated at either frequency. Fixed-notch SINAD also includes transmitter distortion and tone/clock-offset effects. +10/+12/+14 requests repeat PA code 31. The LED1 dependency and oscillator frequency remain unresolved and unmeasured respectively.

[All 144.1 MHz results](baselines/20261117-board4-144p1-rx/results.csv) · [RSSI plots](baselines/20261117-board4-144p1-rx/rssi.svg) · [SINAD plots](baselines/20261117-board4-144p1-rx/sinad.svg) · [Matched 430.1 MHz comparison](baselines/20261117-board4-144p1-rx/comparison-430p1.csv) · [Raw evidence and reproduction](baselines/20261117-board4-144p1-rx/README.md)
