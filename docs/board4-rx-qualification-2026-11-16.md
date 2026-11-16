# Board #4 EEPROM and RX qualification — 16 November 2026

**Latest:** the [full post-reboot sweep with explicit LED1=0](board4-led1-low-rx-qualification-2026-11-16.md) passed all120points with zero recorded transport errors. This is conditional on LED1low; the original failures remain retained.

**Later tests:** the [full retest after manual test-app use](board4-rx-retest-2026-11-16.md) passed all 120 points. The [post-reboot test](board4-rx-reboot-2026-11-16.md) reproduced startup/frame failures and was stopped at the user's request to prioritize #5. Diagnosis is deferred. The results below describe the initial run.

EEPROM programming and boot recognition passed. RX qualification **failed**: 4 MS/s and 2 MS/s failed startup with SMI synchronization errors. The six lower native rates were measured in a 90-point NBFM power sweep, with transport failures retained. No transmitter test was performed on #4.

## Identity and programming

Receiver #4 on pi4f-trixie: UUID `c2440a8d-3852-4c44-b96f-1b64a962e7dc`. Original TX reference on pi4g-trixie: UUID `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`.

Two complete 4096-byte EEPROM backups agreed and were entirely FF. The checked blank-only writer used the Full variant, reproduction vendor string, fresh UUID, per-page verification and header-last ordering. Two full final readbacks matched SHA256 `a191dd25f1cb569fe3315132346fb334869edc88c4061f8d3ab7e59d528646e4`; independent image CRC validation and eepdump passed. After reboot, the expected HAT UUID/product/variant and /dev/smi were present.

## RX results

The first 4 MS/s sweep attempt failed before measurements. A separate startup check repeated the 4 MS/s failure and found the same failure at 2 MS/s; all six lower rates started. The initial lower-rate sweep then stopped on an invalid SMI frame at 800 kS/s. A continuation imported the first two complete rate blocks, repeated 800 kS/s and measured the remaining rates; it retained nonzero counters instead of aborting so failures could be characterized. Production software was unchanged. The cause is not established; oscillator frequency was not measured on #4.

The subsequent sweep held the original transmitter at 4 MS/s, RF24 430.100 MHz, 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. The nominal 90 dB link was inherited from the previous setup and was not independently measured or reconfirmed for this swap. Each supported RX rate has one TX-off baseline and fourteen power requests from -12 to +14 dBm, with two seconds settling and five seconds measurement.

Cells show mean raw RSSI dBm / estimated SINAD dB.

| RX rate | -12 dBm TX | 0 dBm TX | +8 dBm TX |
| --- | ---: | ---: | ---: |
| 4 MS/s | Startup failed | Startup failed | Startup failed |
| 2 MS/s | Startup failed | Startup failed | Startup failed |
| 4/3 MS/s | -99.1 / 18.20 | -88.5 / 23.56 | -76.0 / 25.27 |
| 1 MS/s | -100.0 / 16.99 | -89.2 / 23.39 | -76.2 / 24.92 |
| 800 kS/s | -100.2 / 17.79 | -89.1 / 23.28 | -76.2 / 24.90 |
| 2/3 MS/s | -100.6 / 17.60 | -89.2 / 23.30 | -76.2 / 24.58 |
| 500 kS/s | -101.0 / 17.52 | -89.4 / 23.73 | -76.3 / 25.35 |
| 400 kS/s | -100.9 / 17.56 | -89.6 / 23.60 | -76.3 / 25.50 |

The repeated 800 kS/s block reached invalid_frames=2; the 400 kS/s block reached invalid_frames=1. The 4/3 MS/s, 1 MS/s, 2/3 MS/s and 500 kS/s blocks had zero retained transport counters. Measurements in failing blocks are retained for diagnosis, not acceptance.

## Comparisons and verification

At the 84 matching transmitting settings, #4 minus #1 SINAD averaged +2.79 dB (range +1.70 to +4.10 dB).
At the 84 matching transmitting settings, #4 minus #1 raw RSSI averaged +4.04 dB (range +3.42 to +5.17 dB).

Transport counters were nonzero at 15 of the 90 retained points: [36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 86, 87, 88, 89, 90]. These are failures, including windows where a cumulative counter remained nonzero.

All 2166 retained measurement snapshots reconcile with the 90 summaries. Mode, tone, rates, filter readbacks, transport counters, board UUIDs and absence of TX operation on #4 were checked. Startup failures and the first invalid-frame run remain archived separately.

Both radios were stopped and released afterward. Backend SHA256: `25a9b8cf2c764fc4945e2fc5facb4b1143bf3bad9b7d606abc997cc4d3e390b9`; loaded/installed driver srcversion: `1AC65CD6AE81A1E2ED63F14`.

RSSI and fixed-notch SINAD are uncalibrated link observations. Reconnection, clock offsets and startup conditions can affect comparisons. +10/+12/+14 dBm repeat PA code 31. These results do not establish calibrated sensitivity, oscillator acceptance, TX acceptance or complete board acceptance.

- [Measurements and evidence](baselines/20261116-board4-rx/README.md)
- [RSSI plots](baselines/20261116-board4-rx/rssi.svg)
- [SINAD plots](baselines/20261116-board4-rx/sinad.svg)
- [All measured points](baselines/20261116-board4-rx/results.csv)
- [Per-board tracking](board-rx-validation.md)
