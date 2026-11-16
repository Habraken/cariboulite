# Board #4 — Retest after manual test-app session — 16 November 2026

Scoped RX sweep outcome: **passed**. 120 points recorded; 0 points with nonzero cumulative transport counters; 0 rate startup failures.

Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f-trixie. Original pi4g transmitter UUID: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`. No EEPROM rewrite or TX test on #4.

## Setup and observations

Ran cariboulite_test_app, toggled LEDs to check function, then manually tested4MS/s RX with audio sounding okay

Different initialization history; cause of earlier startup and frame errors not established

No reboot was performed between the reported manual test and this run.

The original TX reference remained at 4 MS/s, RF24 430.100 MHz, 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. Each RX rate has TX off plus fourteen power requests from -12 to +14 dBm, two seconds settling and five seconds measurement. The nominal 90 dB link was inherited from the previous setup and not independently measured. Production DSP, driver and FPGA software were unchanged.

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
| 4 MS/s | -93.9 / 17.86 | -88.2 / 23.21 | -75.9 / 24.87 |
| 2 MS/s | -99.6 / 17.53 | -89.3 / 23.53 | -76.3 / 24.60 |
| 4/3 MS/s | -101.1 / 17.35 | -89.6 / 22.86 | -76.5 / 23.98 |
| 1 MS/s | -101.1 / 17.41 | -89.7 / 22.89 | -76.4 / 24.25 |
| 800 kS/s | -101.1 / 17.29 | -89.7 / 22.54 | -76.6 / 23.80 |
| 2/3 MS/s | -101.0 / 17.25 | -89.6 / 22.69 | -76.3 / 23.60 |
| 500 kS/s | -101.4 / 17.25 | -89.7 / 22.42 | -76.4 / 23.64 |
| 400 kS/s | -101.0 / 17.33 | -89.9 / 22.51 | -76.5 / 23.58 |

## Evidence and limits

All 2877 retained measurement snapshots independently reconcile with the summaries. Settings, role UUIDs, rates, tone, filters and transport counters were inspected. Commands and backend states were checked for absence of TX operation on #4. Both radios were stopped and released afterward.

The initial failed qualification remains valid evidence of the earlier startup/frame failures; a later successful run does not erase it or identify its cause. The user reported running cariboulite_test_app, toggling LEDs and receiving at 4 MS/s before the first retest. No oscillator-frequency measurement was made. These short, uncalibrated RX link tests do not establish oscillator or TX acceptance, calibrated sensitivity, or indefinite continuity. +10/+12/+14 dBm requests repeat PA code 31.

[Raw evidence and reproduction](baselines/20261116-board4-rx-retest/README.md) · [RSSI](baselines/20261116-board4-rx-retest/rssi.svg) · [SINAD](baselines/20261116-board4-rx-retest/sinad.svg) · [CSV](baselines/20261116-board4-rx-retest/results.csv)
