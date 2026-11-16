# Board #4 startup diagnosis — 16 November 2026

**Follow-up:** [backend-only LED control tests](board4-backend-led-control-2026-11-16.md) isolate the observed dependency to LED1 state: #4 starts RX with LED1=0 and fails with LED1=1. The original board works in either state. The findings below describe the earlier test-app sequence.

A repeatable recovery sequence was observed on two boots: 4 MS/s RX failed
before a test-app LED-toggle session and passed afterward. The cause remains
unconfirmed; no production workaround or further software change was applied.

Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f. The user confirmed
the original pi4g TX reference and 90 dB attenuated link. Both hosts used the
corrected FPGA retry handling and matching library SHA256
`9142777cced313c299936dd97126790e4472d1f883810b0f472bee0012899e0d`.
No EEPROM changes or TX trials on #4 were performed.

## First boot: isolate operations

Boot ID `95bf98e4-1bd8-433a-ad90-611686523713` followed the user's power-down
and installation of #4. No radio application was initially running.

| Operation | Observation |
| --- | --- |
| Backend idle initialization | FPGA programmed once, operational check passed, normal exit |
| 4 MS/s RX probe | SMI synchronization failure, no measurements |
| Baseline RX, soft reset, then full reprogram in one backend session | Each RX-start attempt failed; zero emitted samples |
| Test app: print versions, quit; repeat probe | RX still failed |
| Test app: toggle LED0, LED1, LED0, LED1; quit; repeat probe | 4 MS/s RX passed off/on measurement windows |

The four LED toggles returned both LED readbacks to their starting value (zero).
The initial digital readback showed LED0=0, LED1=0, BTN=0, CFG=0. After the first
write it showed LED0=1, LED1=0, BTN=1, CFG=15, and subsequent LED readbacks followed
the writes. This was readback evidence; no physical LED observation was made by
the automated test. The changing static-input readback warrants investigation.

## Second boot: reproduce the sequence

Pi4f was rebooted, producing boot ID `dcb56ad9-f8a8-4bac-984d-f771d08211ff`.
The first RX probe failed at 4 MS/s. The same four LED toggles were then performed
through the test app, without selecting its RX or programming menu. The next
identical probe passed. The digital readbacks followed the same sequence.

| Recovery probe | TX-off RSSI | 0 dBm TX RSSI | Estimated SINAD | Retained snapshots |
| --- | ---: | ---: | ---: | ---: |
| First boot, after LED session | -93.42 dBm | -86.75 dBm | 25.3255 dB | 48 |
| Second boot, after LED session | See raw points | -87.33 dBm | 25.3245 dB | 48 |

Each probe used RF24 430.100 MHz, RX/TX 4 MS/s, and the original reference's
1 kHz NBFM tone, amplitude0.4 and deviation setting2500Hz. Each of the TX-off
and 0dBm points had two seconds settling and five seconds measurement. Both
successful probes had zero retained RX/TX transport counters and valid SINAD
throughout the transmitting window. All96snapshots independently reconcile;
settings, roles, modes, filters, rates and absence of TX operation on #4 were
verified. Both radios were released afterward.

## Interpretation and next diagnostic boundary

This reproduces an initialization/history dependency and provides a bounded
recovery sequence. It does not establish that the LED register itself is the
cause: each intervention includes test-app initialization, read/write traffic,
cleanup and elapsed time. The first boot's test-app-only negative control helps,
but more isolation is needed. The final LED state was unchanged, so the observed
recovery did not require leaving the LEDs lit.

The source LED setter writes only the digital-pin register; the corresponding
RTL branch assigns only the two LED states. A permanent initialization write
would therefore be premature. Next isolate individual digital-register reads
and writes, inspect control readback consistency and trace SPI/FPGA reset and
clock behavior around the first successful write. No board-level hardware-fault
conclusion is established by these tests. RSSI/SINAD are uncalibrated short link
measurements and do not constitute full RX/TX/oscillator acceptance.

[Scripts, logs, raw archives and independent checks](baselines/20261116-board4-diagnosis/README.md)
