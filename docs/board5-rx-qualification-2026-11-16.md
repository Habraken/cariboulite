# Board #5 EEPROM and RX qualification — 16 November 2026

EEPROM programming and boot recognition **passed**. RX qualification **failed**:
all eight native rates failed startup with SMI data synchronization errors.
No RSSI/SINAD measurement windows were collected. Both radios were stopped and
released afterward. Diagnosis remains open.

## EEPROM and board identity

Receiver #5 on pi4f: UUID `b43713d5-7ebf-45ff-b327-aa2e0962c9e2`.
Original TX reference on pi4g: UUID `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`.
The user confirmed #5 installation and the 90 dB attenuated link.

Two full 4096-byte EEPROM reads agreed and were entirely FF. The checked
blank-only writer assigned a fresh UUID, used the Full variant and the existing
reproduction vendor/overlay settings, verified each written page and committed
the header page last. Two final full readbacks matched SHA256
`0d67faff849c9b4f32aefb53c7dae60a133f73a256903799b31019c0ce707df8`.
Independent image/atom CRC validation and eepdump passed. After reboot,
the new HAT UUID, product ID/version and /dev/smi were present.

## RX startup results

The collector was the first radio application run after the EEPROM reboot;
no manual test-app or LED initialization was performed. Each rate used a fresh
RX backend. All eight attempts initialized the backend but failed the RX-start
command with a hardware fault and logged SMI data synchronization failures:

| Native RX rate | Result |
| --- | --- |
| 4 MS/s | Startup failed |
| 2 MS/s | Startup failed |
| 4/3 MS/s | Startup failed |
| 1 MS/s | Startup failed |
| 800 kS/s | Startup failed |
| 2/3 MS/s | Startup failed |
| 500 kS/s | Startup failed |
| 400 kS/s | Startup failed |

Planned setup was RF24 430.100 MHz, original pi4g TX fixed at4MS/s,
1kHz NBFM tone, amplitude0.4 and deviation setting2500Hz, nominal90dB link.
No power sweep could proceed because no rate achieved RX. The reference was
started briefly during collector setup and then held off for the RX attempts.
No TX test was run on #5.

## Verification and limits

The retained evidence independently checks eight failed RX-start commands,
matching receiver/reference UUIDs and hosts, SMI synchronization messages in
every failed backend log, no TX-start commands on the receiver, and zero
measurement points. Final process and SMI descriptor checks confirm radio release.

Both radios used backend SHA256
`25a9b8cf2c764fc4945e2fc5facb4b1143bf3bad9b7d606abc997cc4d3e390b9`;
loaded and installed driver srcversion was `1AC65CD6AE81A1E2ED63F14`.
No production software change was made. The failure cause is not established;
no oscillator measurement or hardware-defect attribution is inferred from this
startup test. EEPROM success does not establish RX/TX hardware acceptance.

[EEPROM, boot, startup and release evidence](baselines/20261116-board5-rx/README.md)
· [Per-board tracking](board-rx-validation.md)
