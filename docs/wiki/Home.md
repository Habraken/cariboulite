# CaribouLite development fork

This repository builds on the original CaribouLabs hardware, FPGA and software
project. This fork adds NBFM TX/RX, mono WBFM RX, audio pipelines, streaming
fixes, diagnostics and standalone radio control.

**Progress updated: 14 November 2026.** The latest validated development
snapshot is `e46ae5e`. The November additions below were tested on the
development build; they should not be assumed available in an older `main`
checkout or installed binary. The standalone backend, controller, passive
monitor and hardware-paced TX were previously merged into `main` in October.

## Start here

- [Installation](https://github.com/Habraken/cariboulite/blob/main/installation.md): current installer behavior, boot configuration and checks.
- [Additional setup and audio notes](https://github.com/Habraken/cariboulite/blob/main/ADDITIONAL-README.md): reported Pi Zero 2 W / 64-bit Trixie installation, audio routing and historical recipes.
- [Radio backend and controller](https://github.com/Habraken/cariboulite/blob/main/docs/radio-control.md): independent C applications for RX and half-duplex NBFM TX.
- [Passive monitor](https://github.com/Habraken/cariboulite/blob/main/docs/passive-monitor.md): optional Python status display in another terminal.
- [Hardware-free checks](https://github.com/Habraken/cariboulite/blob/main/docs/testing.md): software regression entry points and prerequisites.

## Latest development progress

### Streaming continuity and audio playback

Three separate causes of intermittent audio disturbances have been addressed:

- **RX sample alignment:** a stateful streaming decoder checks the IQ byte
  alignment and retains partial samples across reads. It replaces the earlier
  short alignment check and reports framing/acquisition counters in status.
- **Playback buffering:** the ALSA buffer now accounts for burst delivery from
  RX DMA. This prevents the old fixed 50 ms buffer from repeatedly underrunning
  at lower sample rates. The trade-off is additional receive latency, with
  about 194 ms requested buffering at 400 kS/s in the tested configuration.
- **TX transfer continuity:** finite SMI transfer restarts now preserve queued
  FIFO data, use complete DMA-period boundaries and serialize restart attempts.
  This removed periodic low-rate SINAD dips that did not appear in the DMA
  underrun counters.

Software regression checks cover fragmented reads, byte alignment, sample
counts, filtering, fractional-rate timing and partial writes. Hardware tests
were repeated after the fixes, including sustained before/after comparisons.

### All native RX and TX sample rates

NBFM RX, mono WBFM RX and NBFM TX now support all eight native sample rates
from 400 kS/s through 4 MS/s. Fractional rates use exact divider arithmetic;
integer-Hz command labels are rounded.

| Hexadecimal divider key | Native sample rate | Integer Hz label |
| --- | ---: | ---: |
| `1` | 4 MS/s | 4000000 |
| `2` | 2 MS/s | 2000000 |
| `3` | 4/3 MS/s | 1333333 |
| `4` | 1 MS/s | 1000000 |
| `5` | 800 kS/s | 800000 |
| `6` | 2/3 MS/s | 666667 |
| `8` | 500 kS/s | 500000 |
| `A` | 400 kS/s | 400000 |

Keys represent **4 MS/s divided by the hexadecimal key**. Dividers 7 and 9
are unsupported. With the relevant direction stopped, controller examples are:

```text
rx divider A
rx rate 0.4
tx divider 3
tx rate 1.333333
```

`rx rate` or `tx rate` without a value opens the interactive divider menu.
A direct rate argument remains a numeric value in MS/s: `rx rate 1` means
1 MS/s, while divider key `1` selects 4 MS/s. Use `rx divider A`, rather than
`rx rate A`, for a one-line hexadecimal selection.

The diagnostic application's rate menus use the same keys. TX and RX toggles
are `T` and `R` in their respective FM menus. RX was validated first, followed
by TX; both fractional TX rates preserve the correct long-term sample count
while alternating frame lengths as needed.

### Optional NBFM SINAD estimator

Add `--rx-sinad` to the receiving backend's launch command to enable the
**RX SINAD** reading in the passive monitor. It is off by default and requires
live NBFM reception of a known 1 kHz test tone. The transmitter's default tone
is 600 Hz, so select the test tone explicitly with TX stopped:

```text
tx source tone
tx tone 1000 0.4
tx start
```

Use matching RF frequencies and an appropriately attenuated test connection.
Stop with `tx stop`. At the default 2500 Hz deviation setting, amplitude 0.4
requests 1000 Hz peak modulation deviation.

The estimator uses the discriminator audio before playback volume,
de-emphasis and squelch muting, with 300–3000 Hz filtering and a fixed 1 kHz
notch. WBFM, stopped RX, initial settling and windows without a detectable
fundamental show an unavailable reading. It is an unweighted link-quality
estimate, not a calibrated or standards-compliant sensitivity measurement.
Tone-frequency error and transmitter noise/distortion can limit the result.

### Completed sample-rate and power sweeps

A Pi 4 test setup used a wired RF link at **430.100 MHz**, **90 dB nominal
attenuation** and a **1 kHz NBFM tone**. The configuration used a 2500 Hz TX
deviation setting, tone amplitude 0.4 and USB audio playback. The running
kernel was `6.18.60+rpt-rpi-v8` on 64-bit Raspberry Pi OS Trixie.

Two sweeps isolate the RX and TX rate changes:

- Every RX rate, with TX held at 4 MS/s.
- Every TX rate, with RX held at 4 MS/s.

Each rate block included TX off and requested TX power from **−12 to +14 dBm
in 2 dB steps**: **224 transmitting points and 16 TX-off baselines**. The
4 MS/s reference was measured independently in both sweeps. This covers the
two separate rate axes, rather than every possible RX/TX-rate combination.
After two seconds of settling, five seconds of fresh status snapshots were
recorded per point. Intentional power-change gaps and startup/tail cues were
excluded from the measurement windows.

Results after the fixes:

- All recorded RX framing/read, IQ/audio queue, playback and TX transport
  error counters remained **zero**.
- All **5760 saved snapshots** were independently reconciled with the point
  summaries.
- The largest SINAD range within a power step was **0.863 dB**.
- At the weakest requested TX power, mean SINAD across the tested rate pairs
  was **15.49–16.10 dB**; stronger-signal readings were generally around
  **22–23 dB**.
- Listening checks after the fixes reported clean tones.

![SINAD across native sample rates and TX power](https://raw.githubusercontent.com/wiki/Habraken/cariboulite/images/native-rate-sinad-2026-11-14.png)

![RSSI across native sample rates and TX power](https://raw.githubusercontent.com/wiki/Habraken/cariboulite/images/native-rate-rssi-2026-11-14.png)

[Download all point statistics as CSV](https://raw.githubusercontent.com/wiki/Habraken/cariboulite/data/native-rate-power-sweep-2026-11-14.csv).
Plot bands show observed minima/maxima, not uncertainty intervals. Repeated
status readings are not independent SINAD estimates.

Requested TX power and attenuation are not calibrated RF input measurements.
The +10, +12 and +14 dBm requests select the same PA code 31, so those steps
repeat one physical power setting. Raw RSSI noise floors also depend strongly
on RX rate/filter settings. The available range did not reach the 12 dB SINAD
point, so these results do not establish absolute receiver sensitivity or a
receiver SINAD ceiling. They validate the recorded configurations and runs,
rather than unlimited uptime or all startup conditions.

## Current applications and modes

The backend owns the radio, SMI/SPI, DSP and audio devices. The controller uses
a Unix-domain command socket for separate RX/TX frequencies and rates,
NBFM/mono-WBFM receive selection, squelch, microphone/tone TX and FPGA programming
or reset commands. Transmission requires an interactive controller to remain
connected; normal stop restores RX when RX was running before TX.
The passive monitor is optional and cannot key the transmitter.

RX starts stopped by default; use `--rx-start` to receive automatically.
Launch options use `--rx-` and `--tx-` prefixes, including independent
`--rx-deviation` and `--tx-deviation` settings. The controller displays compact
Receive/Transmit sections. With both directions stopped, bare `frequency`,
`mode` and `rate` commands update both where supported; `mode wbfm` changes
RX while TX remains NBFM. Use `rx frequency`, `rx mode`, `rx rate`,
`tx frequency` or `tx rate` for independent settings. The linked
[controller guide](https://github.com/Habraken/cariboulite/blob/main/docs/radio-control.md)
contains the current commands and launch examples.

NBFM TX now selects the narrowest modem transmit filters: an 80 kHz analog
cutoff and a digital cutoff of one eighth the sample rate. The passive
monitor's TX bandwidth reports the cached analog-filter readback; it does
not measure occupied signal bandwidth. Spectrum and achieved deviation
remain unmeasured for this filter profile.

The original diagnostic application remains available. Run it from the
repository root so menu 3 can resolve `firmware/top.bin`:

```sh
./build/cariboulite_test_app 2>debug.log
```

Its FM entries are 11 (NBFM TX tone), 12 (NBFM RX), 13 (NBFM modem self-test)
and 14 (modem monitor with TX/RX and digital interface-loopback controls).
Menu 14 selects RF24/HiF, defaults to 430.100 MHz and 4 MS/s, and uses ALSA
Loopback devices. Pressing `T` starts/toggles the configured TX source; it does
not automatically select a tone. Stopped-only controls select NBFM/mono-WBFM
and separate RX/TX frequencies.
Older builds provide 1/2/4 MS/s; the latest development build adds the native
divider keys described above.

NBFM supports noise and carrier squelch. WBFM supports carrier squelch; stereo
and RDS decoding are not implemented. CW listening, AM, SSB, PSK and the
planned menu-15 transceiver remain future work. The digital FPGA loopback
checks part of the sample path rather than an RF contact.

## Earlier validation and remaining limits

A fresh Pi Zero 2 W / 64-bit Raspberry Pi OS Trixie installation was reported
on 14 October. The subsequent hardware-paced TX build passed microphone/tone
transport trials at **1, 2 and 4 MS/s on both Pi Zero 2 W and Pi 4**, with
kernel **`6.18.50+rpt-rpi-v8`**, and same-backend RX restoration at 1/4 MS/s.
The user also reported successful modulation listening checks. See the
[Pi Zero record](https://github.com/Habraken/cariboulite/blob/main/docs/pi-zero-tx-queue-test-2026-10-15.md),
[Pi 4 record](https://github.com/Habraken/cariboulite/blob/main/docs/pi4-tx-validation-2026-10-15.md)
and [TX design](https://github.com/Habraken/cariboulite/blob/main/docs/tx-hardware-paced-design.md).

These October records identify exact tested configurations and used separately
staged builds and temporary driver reloads. The November validation used
matching rebuilt applications and an installed updated driver. Neither record
establishes that an unrelated installation has been upgraded. The current TX pipeline requires its
matching driver ABI and write-timing profile. Rebuild the SMI module against
matching headers after a kernel change, and keep the userspace library,
applications, driver and FPGA image consistent. Git checkout alone does not
update installed binaries or the loaded firmware.

Independent RF-clock accuracy and spectrum quality remain unqualified.
Physical Soapy RX observations exist, while complete Soapy TX and GNU Radio
workflows remain under validation. The native GNU Radio module supplies a
receive source only. Raspberry Pi 5 is not supported by this SMI hardware path.

## Further development

Further development includes additional modes, configurable transceiver
controls, runtime profiling, stereo/RDS decoding, FPGA antenna/PTT/PA sequencing,
Ethernet audio/PTT and GNU Radio integration. These remain separate from the
validated streaming and sample-rate improvements above.

For DSP interfaces, filtering and evidence behind the overview, see:

- [Audio/DSP interfaces](https://github.com/Habraken/cariboulite/blob/main/docs/audio-dsp-interfaces.md)
- [NBFM channel filter](https://github.com/Habraken/cariboulite/blob/main/docs/nbfm-channel-filter.md)
- [FM architecture and accepted extraction](https://github.com/Habraken/cariboulite/blob/main/docs/demodulator-architecture-research.md)
- [FPGA tests and coverage limits](https://github.com/Habraken/cariboulite/blob/main/firmware/tests/README.md)

Useful issue reports include the source revision, Pi model, OS/kernel, FPGA
image, application/API path and steps to reproduce the issue.
