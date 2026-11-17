# Board #4 raw SMI framing comparison — 17 November 2026

LED1=1 reproducibly prevents RX startup, and the raw incoming data fails the
framing checks. Returning LED1 to 0 restores clean framing and successful RX.
This establishes a data-level failure, but does not yet localize its electrical
or logical origin or distinguish individual bit flips from intermittent slips.

## Setup

Run `20261117T211738Z`, pi4f-trixie, board UUID
`c2440a8d-3852-4c44-b96f-1b64a962e7dc`. RF24, 430.1 MHz, 4 MS/s,
USB audio playback. Existing 90 dB coax link remained connected to original
pi4g; no TX was started. No reboot, reset, FPGA programming command, or test app
was used. Each trial used a fresh backend process, preserved LED0=1, explicitly
set LED1 while idle, observed RX startup for three seconds, stopped, restored
LED1=0, and closed the backend to save the raw capture.

## Results

| Trial | LED1 | RX startup | Captured bytes | Best byte phase | Valid words / examined | Longest valid run |
|---|---:|---|---:|---:|---:|---:|
| 1 | 0 | Success | 1,048,576 | 2 | 262,143 / 262,143 | 262,143 |
| 2 | 1 | HARDWARE / FAULT | 262,144 | 1 | 57,940 / 65,535 | 401 |
| 3 | 0 | Success | 1,048,576 | 1 | 262,143 / 262,143 | 262,143 |

Both low-state captures have zero framing-bit mismatches at their correct
alignment. Arbitrary initial byte phase is normal: the recovery run's phase
also differs from the first successful run.

The failing capture has 7,595 invalid words (11.59%) at its best fixed alignment.
It contains no completely zero or all-one words, no globally stuck bits, and
only seven adjacent repeated words at that phase. This is not an empty or
constant stream. None of the tested byte/halfword permutations produces a
512-word valid run (maximum 401). A simple fixed ordering change does not
explain the failure.

At failing phase 1, framing mismatches are:

| Bit | Required value | Mismatches / 65,535 |
|---|---:|---:|
| 14 | 1 | 2,476 |
| 15 | 0 | 747 |
| 16 | 0 | 635 |
| 30 | 0 | 253 |
| 31 | 1 | 3,856 |

A word may violate multiple bits, so these counts must not be added to estimate
invalid words. Bit 31 has the most mismatches. These are **framing violations
at the best fixed alignment**, not proof of independent physical bit flips:
intermittent slips/reordering or malformed source words could also cause them.

The live decoder reported 31 acquisition failures, zero acquisitions, zero
ambiguous windows, and zero emitted samples for LED1=1. Its zero invalid-frame
counter is expected: that counter covers failures after acquiring lock, which
never happened. The decoder requires at least 512 consecutive valid words;
the failing capture never supplies that at any fixed byte phase. Each low
trial acquired once, had zero acquisition/invalid-frame errors, and emitted
over 12 million samples during the trial.

## Interpretation and next discrimination

The failure is visible before IQ decoding or audio processing. It cannot be
explained solely by RF signal quality or the SINAD estimator. Exposed control
registers previously changed only in LED1, but that does not exclude transient
or internal state.

Possible remaining origins include malformed FIFO input, FIFO/read sequencing,
and SMI bus sampling/timing/electrical errors. To discriminate, compare per-byte
framing signatures and possible slips, then consider a controlled FPGA-generated
known pattern through the same FIFO/SMI path. A known pattern would also expose
payload corruption; this capture has no known payload reference. Timing changes
or logic-analyzer measurements could then test the SMI path specifically.
No causal hardware diagnosis is asserted from these three captures alone.

## Evidence and cleanup

[Raw captures, analyzer JSON, status histories, wire commands and script](baselines/20261117-board4-smi-raw/).
Capture implementation and usage: [raw SMI diagnostic](smi-raw-capture.md).
The deployed library was rebuilt and installed on pi4f; its previous build is
preserved in `build/smi-raw-before/`. The original TX device was not modified.
LED0=1, LED1=0 were restored after every trial. Final release check found no
radio process and no open SMI descriptor on pi4f.
