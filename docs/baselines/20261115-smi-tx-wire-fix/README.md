# TX wire-order correction baseline

See [the report](../../smi-tx-byte-order-fix-2026-11-15.md). The deployed fix is
SDK/driver commit `e458af2`; normal FPGA firmware is retained.

## Evidence

- `before-patterns.json` and `after-patterns.json`: 78 IQ patterns plus one
  nonzero SMI positive control. The former has 61 failures; all 79 records match
  the predicted reversed-byte/SOF-resynchronization model. The latter has none.
- `*-load.json` and `*-restore.json`: image/loader/library identities and verified
  restoration receipts for the qualified before/after runs.
- `postfix-levels.csv`: 20 direct-AM level/backoff/power cases. The TX driver was
  updated for this run; receiver TX-only quiet-padding changes were not yet loaded.
- `postfix-offset-diagnostic.csv`: eight exploratory 25.2 kHz TX-offset cases,
  with RF tuning compensated to retain the wanted carrier at 430.100 MHz.
  This experiment was not deployed.
- `native-am.csv`: 25 final AM cases, including all eight TX and eight RX rates.
- `native-nbfm.csv`: 16 final NBFM cases, all eight TX and RX rates. AM-only
  modulation/gain interpretation is not applicable to these PCM measurements.
- `*-metadata.json`: host, source commit, backend hash, kernel/driver and flags.
  Some level runs preceded the source commit; the source correction was already
  built on TX. The final native AM/NBFM runs use the committed fix on both hosts.
- `gain-staging.csv`: eight zero-backoff cases at power settings 0 and 6 dBm,
  covering amplitudes 0.03, 0.1, 0.25 and 0.5.
- `validation.json`: software/digital/RF checks and qualification limits.
- `release.json`: both radios idle, playback closed, no open SMI handles, installed
  and loaded driver source version `1AC65CD6AE81A1E2ED63F14`.

CSV harmonic ratio includes second and third harmonics only. Tone-to-residual
is a fitted PCM metric, not calibrated SINAD. Very weak/absent fundamentals
produce noise-dominated estimates. Full recordings remain in timestamped `/tmp`
directories on pi4f, not in the repository. Personal speech is not included.

## Digital observer and reproduction

`observer.patch` is a zero-context patch against the normal firmware sources
at `c3c4cd6` (unchanged by the serializer fix). It adds SPI access to the 26 IQ
payload bits at the SMI collector and modem-loopback receiver. The I/Q markers
and control bits in the recorded hex words are reconstructed for readability;
they are not additional observed metadata. Constant patterns are held while
SPI bytes are read; this is not an atomic snapshot interface for changing data.

The diagnostic omits the unused normal SMI RX FIFO and associated receive path
to fit the observer. It is unsuitable for normal radio reception. The TX
collector/serializer logic is unchanged. `lvds_rx` records complete words even
while its normal FIFO-full input is asserted. `tb_bittrace.v` checks 30 walking,
signed and mixed patterns with FIFO-full asserted. The existing loopback RTL
suite also passed its 1,024 register/gap/phase/TX/FIFO cases.

`bittrace.bin` is the tested diagnostic image, SHA-256
`79f2c1e698463e7c933359c5baaebaffb8772aa2412e87b2819a9aad3572484c`.
`timing-summary.json` records the 64 MHz LVDS and 62.5 MHz system timing passes.
The ignored constraint warning refers to the omitted SMI RX clock net.

To rebuild, copy the normal top-level HDL and PCF into an isolated directory
containing a `firmware/` subdirectory, then apply `observer.patch` with `patch -p1`
from that directory. In its `firmware/` directory run:

```sh
yosys -Q -T -p 'synth_ice40 -top top -json bittrace.json' top.v
nextpnr-ice40 --lp1k --package qn84 --json bittrace.json --pcf io.pcf \
  --asc bittrace.asc --parallel-refine --opt-timing --seed 16 \
  --no-promote-globals --report timing.json
icepack bittrace.asc bittrace.bin
```

The qualified test runs on pi4g with both normal radio applications stopped.
Compile `bittrace.c` against the SDK source and `src/at86rf215` include paths,
linking the built shared SDK, ALSA, pthread and libm, as `/tmp/am-bittrace`.
Place the tested image at `/tmp/am-bittrace-fpga/bittrace.bin`. Build the existing
`firmware/diagnostics/load_checked.c` utility as
`build/u25-clock-probe-20261106/load_checked` if absent. `run_bittrace.py` verifies
and programs the image with the checked loader, runs the test, and restores
`firmware/top.bin` in cleanup. The owning C process programs the diagnostic again
after SDK initialization and verifies its marker before testing.

The C fixture disables embedded RF TX start, keeps RF09 off and RF24 in RX for
its digital clock, and never issues TX/TX_PREP. It switches both FPGA and Pi SMI
directions explicitly, uses production-style two-period bounded buffering, and
requires fixed-pattern and SMI positive controls. The diagnostic is applied only
for the test and is always replaced with the verified normal image afterward.
