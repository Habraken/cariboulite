# Residual AM amplitude-transfer baseline, 70 dB link

See [the report](../../am70-residual-distortion-2026-11-15.md). Source commit for
all production objects was `81a0f08`. pi4g is TX, pi4f RX, wanted RF 430.100 MHz.
Production binaries, modulation, RF defaults and normal FPGA firmware remain
unchanged. All temporary binaries/images are diagnostics, not deployed fixes.

## Evidence

- `levels.csv`: 16 baseline cases; `rxgain.csv`: eight cases with 21 dB less RX gain.
- `offset.csv`: eight 25.2 kHz rotating-IQ cases with compensated TX tuning.
- `phase45.csv`, `phase90.csv`: four cases each, static carrier rotations.
- `*-metadata.json`: host/source/binary/driver identity and launch settings.
- `envelope-analysis.json`: 12 actual-production-modulator/packetizer combinations,
  each measuring one million settled samples before and after legacy I-bit tagging.
  Null metrics indicate a vanished fundamental; do not interpret them as zero noise.
- `iq-analysis.json`: four independent raw-IQ envelope and relative spectral checks.
  An FFT band of ±6 kHz around the wanted carrier is selected before taking the
  envelope; the first/last 20 ms of decimated data are discarded. The analysis
  uses neither the production AM AGC nor calibrated audio playback. Spectral
  values integrate a 60 Hz band and are relative receiver observations, not
  calibrated conducted-emission measurements or a complete spectral mask test.
- `carrier-steps-analysis.json`: slow restarted carrier sweep, **inconclusive**
  because return-to-reference drift (~0.51 dB) exceeded the intended 0.3 dB span.
- `stair.csv`, `stair-analysis.json`: two continuous interleaved-reference captures,
  target odd I codes 1891..2101, reference I=2001, Q=0, each level held 2 ms.
  Raw IQ is filtered to ±10 kHz around the carrier and decimated to 50 kS/s.
  Periodic correlation aligns the known staircase; only each plateau's middle
  1 ms is measured. Each target is normalized to its two adjacent references.
  Per-level standard deviations describe repeated cycles, not instrument accuracy.
- `validation.json`, `release.json`: qualification limits and final idle checks.

PCM harmonic ratios include only the second/third harmonics. Tone/residual is a
fitted end-to-end audio metric, not standardized SINAD. Ratios are not meaningful
when the fundamental is nearly absent. The existing analyzer is
[`analyze.py`](../20261115-am-tx-link/analyze.py).

RF captures remain in timestamped `/tmp/am70-*` directories on pi4f. Local copies
are `/tmp/am70-*-recordings` on pi4g. No private speech or raw RF/audio recordings
are committed. The requested listening comparison is pending operator readiness;
no subjective pass is claimed.

## Changing-word digital observer

`sequence_probe.c`, `sequence_check.v`, `tb_sequence.v`, `observer.patch` and
`sequence.bin` retain the exact diagnostic. Apply the patch to a temporary copy
of the production `firmware/*.v`, copy `sequence_check.v` into that directory,
and add the production `io.pcf`. The normal TX collector, FIFO and LVDS generator
are retained. The unused SMI RX FIFO is omitted to fit this observer; it cannot
serve as a normal RF receiver image.

The checker acquires a word satisfying Q=rotate_left_13(I,5) XOR 0x1555, then
verifies I increments by 73 modulo 8192. It latches completion after 8191
transitions, covering a full cycle including the acquisition word. The SMI
collector disallows duplicates; modem return allows exact held words because
sample-gap operation repeats values. It does not count/qualify the holds.
Status bits: count[12:0], acquired[13], sticky_error[14], done[15]. SPI IOC 8
returns marker A6; IOC 9..12 returns modem status, IOC 13..16 collector status.
The host reads after one second, when completed status is stable. This does not
provide a coherent snapshot contract for arbitrary changing multi-bit values.
Clean status is 0xBFFF; deliberate Q-bit corruption produces 0xFFFF at both points.

Simulation command in the temporary source directory:

```sh
iverilog -g2012 -o tb_sequence tb_sequence.v
vvp tb_sequence
```

Build tools: Yosys `synth_ice40 -top top -json sequence.json`, then
`nextpnr-ice40 --lp1k --package qn84 --json sequence.json --pcf io.pcf
--asc sequence.asc --parallel-refine --opt-timing --seed 16 --no-promote-globals
--report timing.json`, followed by `icepack sequence.asc sequence.bin`.
`timing.json` retains utilization, timing and image SHA-256. All active clocks
meet their constraints, including LVDS 64 MHz and system 62.5 MHz.

`run_sequence_probe.py` expects the image at
`/tmp/am70-sequence-fpga/sequence.bin` and the compiled probe at
`/tmp/am70-sequence`. Compile the C probe against the existing SDK with includes
for `src`, `caribou_smi`, `caribou_fpga`, `at86rf215`, `io_utils`, `caribou_prog`
and `rffc507x`, link `libcariboulite`, pthread and libm. Run with both radio
applications stopped. The probe initializes the SDK and programs the diagnostic
inside its owning process. EXTLB is enabled with EEC off, RF09 off and RF24 RX
supplying the clock; it issues no RF TX command.

The parent uses the repository's checked FPGA loader and restores the normal
image in `finally`, including after a test failure. `sequence-load.json`,
`sequence-restore.json`, `sequence-clean.txt` and `sequence-negative.txt` retain
receipts and exact outcomes. Normal image SHA-256 is
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.

## RF and software fixtures

`radio_helper.py` updates the earlier helper for the current 33-field v2 reply.
The `run_*.py` scripts retain exact case settings and host ownership checks.
Their fixed `/tmp` paths correspond to this run; copy/rename fixtures accordingly
before reproducing. Each RF helper owns and shuts down its backend. The first
reduced-gain attempt requested an invalid 20 dB step and exited before RX started;
the qualified run used the supported 21 dB value.

`offset_mod.c`, `phase_mod.c` (compile with AM_DIAG_PHASE in radians) and
`stair_mod.c` replace only `nbfm_mod.o` in separate temporary backend executables.
Use the production build flags/objects and exclude stale duplicate objects.
Production executables were not overwritten. `rx-iq-capture.patch` adds a bounded
raw-IQ capture to a temporary RX backend; all qualified files contain two million
IQ pairs. Its blocking diagnostic writes completed without observed stream errors;
it is not a production recording design. Raw-Q/ADC-rail and counter checks are
retained in the analysis/results.

`envelope_probe.c` links the actual `nbfm_mod.c` and `mod_worker.c` with function
sections and `--gc-sections`; `analyze_envelope.py` measures the settled samples.
The scripts use fixed temporary paths for this hardware session. NumPy is needed;
SciPy is not required. No permanent speech processing is part of these fixtures.
