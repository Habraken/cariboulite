# Reserved FPGA pads and synthesis-warning cleanup

Implemented and checked on 2026-10-17 on `dev-fpga-pa-control`. The eight RTL
warning occurrences are resolved: one duplicated TX FIFO backpressure assignment
and seven undriven pad bits. The remaining five synthesis messages are the
reviewed SPI tri-state message and four ABC mapping diagnostics.

The edited source passes the digital simulations. Three CE2-serial builds pass
all reported clock constraints and produce identical bitstreams; three
default-serial builds fail placement. **CE2 serial is the provisional build
baseline for this cleanup candidate.** The earlier
[A2 default-serial recommendation](fpga-a2-build-matrix-2026-10-17.md) applies to
the earlier source and is superseded for this candidate. Production build
defaults and packaged firmware have not changed.
The candidate was subsequently
[loaded and checked over SPI](fpga-ce2-programming-2026-10-17.md), followed by
user-reported passing smoke tests in both applications on 2026-10-17. Physical
pad measurements and full A5 acceptance remain pending.

## Source changes and physical intent

- Preserve the user's removal of the duplicate assignment at the end of
  `smi_ctrl.v`. Its original `o_smi_write_req = !i_tx_fifo_full` assignment remains.
- Define seven reserved pads using explicit input-only `SB_IO` cells with
  `PIN_TYPE=6'b000001`, `PULLUP=0` and unused input-data outputs. The four
  `io_pmod_out` pads are reserved for external PA control; mixer FM/enable and
  the separate SMI write-request pad remain reserved as well.
- Retain existing port names and `io.pcf` assignments. The reserved ports are
  declared `inout` for the physical I/O cells, including historical `o_` names.
  No control register can currently enable these pad outputs.
- Preserve SPI MISO selection and the shared SMI request mux:
  `o_smi_read_req = direction ? rx_request : tx_request`. The internal TX
  backpressure signal remains functional. The host still reserves GPIO25 for
  SMI; the physical write-request pad has not become an available host GPIO.
- Leave mixer ownership through SPI and PMOD/register readback unchanged. This
  change introduces no PA output sequencing or physical mixer-pin control.

The frozen A2 designs configured these same pads input-only without pull-ups.
Explicit cells preserve that configuration instead of leaving it to undriven
net handling. Deleting a port and its constraint would enable the installed
nextpnr flow's unused-pin pull-up, changing the electrical state. The mapped
candidate pad configuration is checked against A2 in both routed JSON and the
actual ASC bits: all seven physical locations, pin types, receiver-disable and
pull-up-disable settings match. The decoder/source identities and results are
saved in `pad-verification.json`. Physical behavior on a board
and the unconfigured FPGA still require later qualification.

## Why the serial builds were repeated

Even an electrically equivalent cleanup can change the mapping presented to the
placer. ABC's extracted graph gained seven inputs (702 to 709), while its
2168 AND gates and 8324 wires were unchanged. The new explicit pad interfaces
therefore changed mapping and packing. A separate synthesis-only check retaining
the original `output` declarations still had 709 inputs and changed LUT mapping;
it did not establish a way to preserve A2's graph. The final source uses the
explicit `inout` interface and exactly matches the tested candidate snapshot.

No routing options, pin constraints, clock requirements or seeds were relaxed.
Three fresh builds of each serial synthesis setting used the same A1 toolchain,
LP1K/QN84, seed 16, `--opt-timing` and `--no-promote-globals`, omitting
`--parallel-refine`. The unchanged-source A2 matrix remains preserved separately.

| Source / synthesis | Passed | Logic cells / 1280 | System Fmax MHz | LVDS Fmax MHz |
| --- | ---: | ---: | ---: | ---: |
| A2 default serial | 3/3 | 1022 | 68.88 | 71.00 |
| Cleanup default serial | 0/3 | 1034 | Not reached | Not reached |
| A2 CE2 serial | 3/3 | 1035 | 77.72 | 71.16 |
| Cleanup CE2 serial | 3/3 | 1035 | 72.76 | 75.90 |

Each cleanup default run exits 255 during placement, failing to place
`sys_ctrl_ins.rx_sync_24_SB_DFFER_D_DFFLC` after 151801 attempts. No routing
timing result or candidate image exists for those runs. This is a placement
failure, not a measured clock-frequency failure.

All three cleanup CE2 runs pass synthesis, placement, routing, every reported
clock constraint and packing. They use 1035 logic cells (245 free), 16/16 RAM
blocks and 1/8 global buffers. Their worst system/LVDS Fmax is 72.7590/75.8956 MHz,
against requirements of 62.5/64 MHz. Other clocks also pass: input/global
463.6068/125 MHz, SPI 195.8097/5 MHz, and SMI request clock 137.5894/16 MHz.
Fmax-derived period margins are not actual worst-path slack for DDR paths.

Within each synthesis group, the netlist is identical across its three repeats.
The CE2 routed JSON, ASC and image are also identical across all repeats.
The CE2 candidate image SHA-256 is
`78f650fdf192f8a3c11fc45a253d08b90071e78671d59100ab9de7b5b38ff7c2`.

The six-run batch took 202.5 seconds; recorded build intervals were 23.5–27.0
seconds, with reference/tool verification overhead outside those intervals.
A diagnostic synthesis ran concurrently with part of this batch, so these times
are not a controlled performance comparison with A2. The batch returns exit 1
because all three default builds failed, retaining every failure.

This cleanup establishes explicit pad intent rather than an area optimization:
the passing CE2 build uses the same logic-cell count as A2 CE2. Seed sensitivity,
physical acceptance and the complete PA implementation remain future work. A3
can use this CE2-serial candidate as its provisional comparison baseline; neither
this record nor A2 physically accepts a newly generated image.

## Checks and reproducibility

All seven digital checks pass: the new actual-top-level reserved-pad test, build
dependency/failure check, TX sequence, TX stop, FIFO reset, 1024-case loopback and
driver quiet-IQ framing. The new test checks 33 pad scenarios across reset,
actual SPI control writes and both channels/directions. It checks released
high-impedance pads, externally driven levels without contention, elaborated
no-pull-up parameters, MISO selection and the shared FIFO request behavior.
The installed iCE40 simulation model does not simulate its pull-up parameter
electrically, so that parameter is checked explicitly and in mapped hardware.

All 25 experiment-runner tests pass, including the new uncommitted-candidate
snapshot workflow. Normal `freeze` still requires committed source. `candidate`
records exact existing HDL bytes, changed filenames, working-tree status, base
manifest and source diff in a fresh directory; it preserves PCF, Makefile,
toolchain and rollback files. The copied image/host/source archive and old check
logs remain baseline evidence, explicitly identified as such. Candidate outputs
and new checks are kept separately. See the
[runner guide](../firmware/tools/README.md#snapshot-an-hdl-candidate).
The frozen candidate manifest SHA-256 is
`3f7657e5ee579912ab6af08c41734e1d5f54ea775f93745ca8c12bab4035e982`.

Local artifacts are under
[`build/fpga-pa-control/warning-cleanup-20261017/`](../build/fpga-pa-control/warning-cleanup-20261017/):
`candidate/`, `plan.json`, `commands.sh`, `runs/`, `checks/`, `summary/`, batch
metadata, the synthesis-only diagnostic and verification receipts. An archive
and checksum are saved alongside that directory as
`warning-cleanup-20261017.tar.gz` and `.tar.gz.sha256`.

All retained run artifact hashes are verified. Final source hashes match the
candidate and digital-check snapshot. A1/A2 reference evidence remains intact,
and all six production FPGA artifact hashes are unchanged. The packaged image
remains
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.
No candidate image or header was installed during this build qualification.
The later volatile FPGA programming session is linked above.
