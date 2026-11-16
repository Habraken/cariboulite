# Board #4 — Full RX sweep after reboot with LED1=0 — 16 November 2026

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
