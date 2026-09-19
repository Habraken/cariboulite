# CaribouLite development fork

This repository builds on the original **CaribouLabs CaribouLite** hardware,
software and FPGA project. Jan's development work focuses on practical RX/TX
operation, NBFM audio, better diagnostics, and more reliable sample streaming.

**Looking for the NBFM TX and RX work? Start with the
[`dev` branch](https://github.com/Habraken/cariboulite/tree/dev).**
The capabilities described here refer to that branch, not necessarily the
repository's default branch or the original CaribouLabs release.

## What has changed?

The original CaribouLabs project provides the foundation: board support, modem
and mixer control, FPGA interfaces, SMI transport and SDR APIs. This fork extends
that foundation and addresses defects found while developing and testing it.

| Area | Development work on `dev` |
| --- | --- |
| NBFM transmit and receive | NBFM modulation/demodulation and audio pipelines in the diagnostic application, including ALSA audio routing. |
| Interactive diagnostics | NBFM menu entries and a modem monitor with RX/TX controls, register displays and streaming instrumentation. |
| FPGA TX and diagnostics | Work on TX sample sequencing, stop/reset behavior, sample-gap control and an interface-loopback diagnostic, with a tested loopback image promoted into the repository. |
| Streaming reliability | Fixes for device ownership, partial writes, timeouts, cancellation and repeated RX/TX start/stop behavior. |
| API and resource handling | Corrections to C/C++ and Soapy streaming paths, buffer handling and SPI resource cleanup. |
| Kernel compatibility | Driver builds and recorded live checks on Raspberry Pi 4 with the 64-bit kernel `6.18.39+rpt-rpi-v8`; kernel API adaptations are included in `dev`. |
| Build and installation | Firmware dependency/failure handling, standalone builds without SoapySDR, and driver installation restricted to the kernel used for the build. |
| Tests and documentation | Software regressions, FPGA simulations, recorded hardware observations, a documentation audit and a development roadmap. |

These are additions and corrections made during this fork's development;
they are not a claim that every listed feature was absent from upstream.

## What works today?

Jan has reported successful **NBFM TX and RX operation** on the development
setup, including audio checks. The current diagnostic menu includes:

- **11 — NBFM TX Tone**
- **12 — NBFM RX**
- **13 — NBFM modem Self-Test**
- **14 — Monitor Modem Status**, with TX/RX and interface-loopback controls

Menu 14 uses the RF09 (Sub-1 GHz) modem path for TX and RX, with a default
frequency of 430.100 MHz and ALSA loopback audio. Pressing T
starts/toggles TX; it does not automatically select an internal test tone.
Some audio devices, rates and RF settings remain hardcoded, so these are
working development tools rather than a finished configurable transceiver UI.

The separate L interface-loopback diagnostic still uses its RF24 receive path.
FPGA interface loopback has simulation coverage and recorded hardware results.
It checks part of the digital sample path; it does not demonstrate an RF QSO.
Physical Soapy RX checks are recorded, while physical Soapy TX and complete
GNU Radio TX/RX workflows still need validation.

## Kernel compatibility

As of **16 September 2026**, Jan's development system runs the 64-bit Raspberry
Pi kernel **`6.18.39+rpt-rpi-v8`**. Driver builds, installation and live RX/TX
checks on this kernel are recorded in the
[review and validation log](https://github.com/Habraken/cariboulite/blob/dev/docs/code-review-2026-09-12.md).
The `dev` branch includes the kernel API adaptations needed by this setup.

This identifies the kernel actually used for development and testing. Other
kernel versions still need their own validation, and the SMI module must be
rebuilt against matching headers after a kernel update.

## Where to start

Use the [`dev` source tree](https://github.com/Habraken/cariboulite/tree/dev)
and read the [additional installation and usage notes](https://github.com/Habraken/cariboulite/blob/dev/ADDITIONAL-README.md).
They explain the development menu and audio setup, and identify historical
instructions that still need fresh installation testing.

After building, launch the application from the repository root:

```sh
./build/cariboulite_test_app 2>debug.log
```

Menu option 3 programs `firmware/top.bin` relative to that directory. Keep the
application, library, kernel driver and FPGA image consistent with the setup
being tested; selecting a Git branch does not update already installed binaries
or the image running on the FPGA.

## What is next?

The [roadmap](https://github.com/Habraken/cariboulite/blob/dev/ROADMAP.md) covers:

- Clearer hardware, streaming, DSP and application interfaces, with configurable settings and broader tests.
- Squelch and menu 15: a VHF/UHF transceiver, starting with NBFM and later adding SSB, AM, CW and PSK modes.
- FPGA antenna/PTT/external-PA sequencing, Pi Zero testing, Ethernet audio/PTT and GNU Radio integration.
- Re-creating the schematics and PCB as an editable KiCad project.
- Two practical milestones: a first on-air NBFM QSO and a QO-100 QSO using SSB, CW or PSK.

These are planned milestones, not completed capabilities.

## Evidence and current limits

This overview reflects the documentation reconciled on **16 September 2026**.
Successful operation on Jan's setup does not establish support for every Pi,
OS, sample rate, RF path or external application. Fresh installation recipes,
RF performance and several integration paths remain under validation.

For the details behind the overview:

- [Code review and fix status](https://github.com/Habraken/cariboulite/blob/dev/docs/code-review-2026-09-12.md)
- [FPGA study, hardware observations and promotion record](https://github.com/Habraken/cariboulite/blob/dev/docs/fpga-gap-study-2026-09-13.md)
- [FPGA simulation checks and coverage limits](https://github.com/Habraken/cariboulite/blob/dev/firmware/tests/README.md)
- [Documentation audit](https://github.com/Habraken/cariboulite/blob/dev/docs/documentation-audit-2026-09-16.md)

Feedback is most useful when it includes the source revision, Pi model,
OS/kernel, FPGA image, menu/API path and steps needed to reproduce the result.
