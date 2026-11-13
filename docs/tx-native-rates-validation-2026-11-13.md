# Native TX sample-rate validation

TX now accepts the same eight native clocks as RX: 4, 2, 4/3, 1, 0.8,
2/3, 0.5 and 0.4 MS/s. Integer-Hz command labels are 4000000, 2000000,
1333333, 1000000, 800000, 666667, 500000 and 400000. The 666666 alias is
accepted and normalized to 666667. Timing uses exact 4 MHz/divider clocks.

The controller's `tx rate` menu and `tx divider KEY` use hexadecimal divider
keys 1/2/3/4/5/6/8/A. Explicit `tx rate 1` continues to mean 1 MS/s.
The legacy test application's TX/RX menus and monitor use those same divider
keys; TX/RX toggles move to T/R in their respective menus. The monitor's
loopback interlock covers every new rate key.

## Implementation

The modulator advances audio time with an integer accumulator against the
4 MHz clock. This avoids treating rounded fractional-rate labels as physical
sample clocks. Frequency deviation uses the exact native rate.

At 4/3 MS/s, consecutive 10 ms TX frames contain 13333, 13333 and 13334 IQ
pairs. At 2/3 MS/s they contain 6666, 6667 and 6667 pairs. Each frame carries
its actual length through partial/retried writes. Audio consumption remains
480 samples per frame on average with at most one sample carried in the
modulator's input queue. Buffer allocation uses the largest frame length;
tail padding uses the shortest length so RF shutdown remains conservative.

The requested rate is stored separately from frame length. Profile programming
and checked readback use the native modem divider and FPGA sample gap equal
to divider minus one. In particular, calculating a gap with integer division
by the rounded 666667 Hz label would give the wrong answer. The existing
FPGA gap register supports the required values; no firmware change is needed.

## Software checks

- All eight rates: 300 frames each under strict and production floating-point
  flags, exact cumulative counts, no held-audio ticks, waveform deviation,
  chunk-invariant IQ, output markers and reset reproducibility.
- Existing 1/2/4 MS/s modulation remains within one IQ count of the historical
  implementation, with existing recovered-tone and envelope checks passing.
- Real TX writer tests exercise all new rates through partial, zero-progress
  and retried writes, preserving fractional frame boundaries and completion.
- Existing lifecycle, hardware-paced transport, shutdown/padding and loopback
  interlock tests pass. The complete backend test target passes, including
  native CLI profiles, controller divider commands and both-direction controls.

## Hardware checks

pi4g-trixie transmitted through the user's wired nominal 90 dB attenuation
into pi4f-trixie, with RX fixed at 4 MS/s, RF24 at 430.1 MHz, a 1000 Hz
amplitude-0.4 tone and 2500 Hz configured deviation. Requested TX power was
0 dBm. Each rate used a fresh transmitter backend, settled for three seconds,
and measured for 20 seconds through the Jabra speaker.

The first matrix exposed recurring low-rate SINAD dips. They were traced to
[finite TX-window restarts](tx-window-restart-validation-2026-11-14.md), fixed
in the driver and verified with a sustained before/after comparison. The
complete eight-rate matrix below was then repeated with the final driver on
both radios, without capture interposers.

| TX key | Mean SINAD (dB) | Observed SINAD range (dB) | Measured TX IQ pairs/s |
| --- | ---: | ---: | ---: |

| 1 | 21.66 | 21.15–22.07 | 3999360 |
| 2 | 21.62 | 21.14–21.91 | 2000903 |
| 3 | 21.78 | 21.38–22.05 | 1334110 |
| 4 | 22.13 | 21.68–22.42 | 1000411 |
| 5 | 22.07 | 21.48–22.48 | 800680 |
| 6 | 21.99 | 21.51–22.32 | 666859 |
| 8 | 21.88 | 21.30–22.16 | 500315 |
| A | 21.86 | 21.39–22.18 | 399482 |

Every verified case passed the test's SINAD floor/spread, throughput and
profile-gap checks. Recorded RX alignment/read errors, IQ/audio drops,
playback underruns, TX writer errors, TX queue drops and TX DMA underruns
were zero. Short-window throughput includes admission-buffer movement and
is not a precision clock measurement. This validates the measured runs;
it does not establish sensitivity or indefinite continuity.

[Results and reproduction tools](baselines/20261113-tx-native-rates/) are
committed. Full logs remain under
`build/radio-backend/implementation-20261113/cariboulite-tx-rates-20261113T230639Z/`.
The separate three-second settling records are retained with their labels;
they include expected startup/cue transients and are excluded from the table.
