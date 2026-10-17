# A2 default/CE2 build matrix

Updated 2026-10-17 on `dev-fpga-pa-control` after the reserved-pad warning
cleanup. **CE2 synthesis with serial refinement is the provisional baseline for
the current source.** Default serial now fails placement. Production build
defaults and packaged firmware remain unchanged. The CE2 candidate was later
[programmed and checked over SPI](fpga-ce2-programming-2026-10-17.md), followed
by user-reported passing smoke tests in both applications on 2026-10-17.
Full A5 acceptance remains pending.

## Current source: results after warning cleanup

The [cleanup](fpga-warning-cleanup-2026-10-17.md) removes the duplicated SMI
assignment and explicitly configures seven reserved pads as input-only without
pull-ups. Because these HDL changes alter synthesis mapping, both serial
configurations were rebuilt three times from a frozen candidate snapshot.
Target, PCF, toolchain, seed 16, clock constraints, `--opt-timing` and
`--no-promote-globals` stayed fixed.

| Synthesis | Refinement | Passed | Logic cells / 1280 | System Fmax MHz | LVDS Fmax MHz | Outcome |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| Default | Parallel | Not rerun | — | — | — | No result for current source |
| Default | Serial | 0/3 | 1034 | Not reached | Not reached | Placement failed, exit 255 |
| CE2 | Parallel | Not rerun | — | — | — | No result for current source |
| CE2 | Serial | **3/3** | **1035** | **72.76** | **75.90** | All clock checks and packing passed |

Both serial groups use 16/16 RAM blocks and 1/8 global buffers. Default fails
before routed timing, so it has no measured Fmax or packed image. All three
CE2 runs produce identical netlists, routed designs and images; every reported
clock passes, including the required system 62.5 MHz and LVDS 64 MHz.

The current CE2 candidate leaves **245 logic cells free**. Its cell count equals
the earlier CE2 result; the cleanup provides explicit pad behavior without an
area saving. Synthesis warning occurrences decrease from thirteen to five, and
the actual routed pad configuration matches the earlier builds.

Current candidate image SHA-256:
`78f650fdf192f8a3c11fc45a253d08b90071e78671d59100ab9de7b5b38ff7c2`.
The source snapshot is identified by candidate manifest SHA-256
`3f7657e5ee579912ab6af08c41734e1d5f54ea775f93745ca8c12bab4035e982`.
See the [cleanup qualification record](fpga-warning-cleanup-2026-10-17.md) for
the six runs, simulation checks, pad verification and retained evidence.

Carry **CE2 serial** into further experiments on this source. The original
default-serial recommendation below applied to the earlier HDL. Seed
sensitivity, physical pad measurements and full A5 acceptance remain future work.

## Original A2 matrix before warning cleanup

The following twelve-run matrix records the earlier frozen source: nine builds
passed and three CE2-parallel builds failed LVDS timing. Both original serial
groups repeated identically. These measurements remain the historical comparison;
the parallel results do not qualify the edited source.

### Controlled inputs and execution

The [A1 reference](fpga-a1-reference-2026-10-17.md) remains frozen at source
revision `b190aea78fade5fa88ba863158d0a1895b6ea6d6`, manifest SHA-256
`07beab053c259f6e473ee8d282a032c6120a439bb40db56ab0d44a3e7ab3ce52`.
The checkout was clean at A2 start, revision
`c9d5a0517cb3c2163c02a4307f730395895294dc`, which contains the A1 tooling.
All fifteen frozen source files match their reference hashes in every run.

The prepared A1 `a2-plan/commands.sh` executed sequentially, using the frozen
runner and toolchain. Each run had a fresh work/output directory. Only these
two factors changed:

- Synthesis: current default versus CE2 (`-dffe_min_ce_use 2`).
- Refinement: current `--parallel-refine` versus serial, omitting that flag.

Target LP1K/QN84, seed 16, source/PCF, `--opt-timing` and
`--no-promote-globals` stayed fixed. Tools were Yosys `0.60+64` (`d523c88c3`)
and nextpnr-ice40 `0.9-49-g7bd1336f`, as recorded in A1. Tool executables,
ABC and synthesis data verified before and after the experiments. No clock
constraint was relaxed. The production Makefile was not invoked.

### Original results

Fmax ranges include every repeat, with final routed measurements. Requirements
are system **62.5 MHz** and LVDS **64 MHz**.

| Synthesis | Refinement | Passed | Logic cells / 1280 | System Fmax MHz | LVDS Fmax MHz | Run seconds |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Default | Parallel | 3/3 | 1022 | 75.90–84.15 | 65.80–81.66 | 31.0–34.5 |
| Default | Serial | 3/3 | 1022 | 68.88 | 71.00 | 32.0–33.0 |
| CE2 | Parallel | 0/3 | 1035 | 78.04–90.61 | **59.05–62.64** | 24.9–27.9 |
| CE2 | Serial | 3/3 | 1035 | 77.72 | 71.16 | 27.3–29.0 |

Every run used **16/16 RAM blocks** and **1/8 global buffers**. Default leaves
258 logic cells; CE2 leaves 245. CE2 costs thirteen more cells in this design.

All other reported clocks passed in every run: SPI at 5 MHz, the input/global
clock at 125 MHz, and `smi_ctrl_ins.soe_and_reset` at 16 MHz. The failed runs
completed synthesis with exit 0, then nextpnr exited 1 on final LVDS timing.
Packing did not run for those cases. Their routing reports, routed designs,
ASC files and logs remain evidence of failed experiments, not accepted images.

| Run | Result | System MHz | LVDS MHz |
| --- | --- | ---: | ---: |
| `default-parallel-s16-r1` | Pass | 75.90 | 74.96 |
| `default-parallel-s16-r2` | Pass | 84.15 | 81.66 |
| `default-parallel-s16-r3` | Pass | 82.43 | 65.80 |
| `default-serial-s16-r1` | Pass | 68.88 | 71.00 |
| `default-serial-s16-r2` | Pass | 68.88 | 71.00 |
| `default-serial-s16-r3` | Pass | 68.88 | 71.00 |
| `ce2-parallel-s16-r1` | Fail | 90.61 | 59.05 |
| `ce2-parallel-s16-r2` | Fail | 78.04 | 62.00 |
| `ce2-parallel-s16-r3` | Fail | 88.96 | 62.64 |
| `ce2-serial-s16-r1` | Pass | 77.72 | 71.16 |
| `ce2-serial-s16-r2` | Pass | 77.72 | 71.16 |
| `ce2-serial-s16-r3` | Pass | 77.72 | 71.16 |

The complete matrix took **460.7 seconds (7 minutes 41 seconds)** on this host.
Individual recorded run intervals were 24.9–34.5 seconds, averaging 30.1 seconds;
those intervals exclude the runner's initial reference/tool verification and
final verification/inventory overhead. The batch returned exit 1 because three
builds failed, while still attempting all twelve entries.

### Original repeatability and interpretation

Default synthesis produced one netlist SHA-256 across all six default runs:
`49a5ed48f5dfd7e4cae0f700c43c2835c6ec792d8eba951ab1ab62fa2d5e2c53`.
CE2 also produced one netlist across its six runs:
`873d6c6f695139563b9cc8c8567ea50f87ad66a5df9c606dd5edb3cdc8bc15ac`.
These match the historical netlist identities; the historical Fmax values are
not the exact outcome of every new repeat.

Each parallel group produced three different routed JSON and ASC hashes despite
identical source, synthesis output and seed. Default parallel also produced three
different images. Its worst observed LVDS result exceeded the requirement by
only 1.80 MHz. Selecting its best 81.66 MHz result would hide that variation.

Within each serial group, JSON/BLIF synthesis outputs, routed JSON, ASC and image
hashes were identical across all three repeats. This establishes repeatability
for the tested source/tool/flags/seed on this host; other seeds and hosts have
not been tested. Serial images are:

| Variant | Image SHA-256 |
| --- | --- |
| Default serial | `0fd8582b240baf3274dc78ad1c70461c0d6871586d9a08563e83feddf1ea5084` |
| CE2 serial | `d85db2a5636e3d38081173a6e3414b7095bf2520dafe6b52cc5fd6714a8c8052` |

The original matrix favored **default synthesis with serial refinement** as its
provisional comparison baseline: it used fewer cells and passed repeatably with
system 6.38 MHz and LVDS 7.00 MHz above their requirements. Original serial CE2
used thirteen additional cells for 8.84 MHz more system Fmax and 0.16 MHz more
LVDS Fmax. The current-source results above supersede that baseline choice.
A2 did not establish a need for an RX RTL rewrite; further experiments must use
the exact source and recipe they qualify. Production defaults remain unchanged.

All runs retained the same thirteen inherited synthesis warning occurrences
(twelve unique lines), including
duplicate SMI request drivers, undriven SMI/mixer/PMOD outputs and SPI tri-state
handling. A2 did not modify that RTL. Clock Fmax checks do not establish CDC
safety, external I/O timing or physical DDR/RF behavior. In particular, a
full-period margin derived from Fmax is not actual worst-path slack for a
half-cycle DDR path. A5 seed sensitivity and physical acceptance, and later
qualification of the integrated PA design, remain required.

### Original retained evidence and validation

The ignored local directory
[`build/fpga-pa-control/a2-20261017/`](../build/fpga-pa-control/a2-20261017/)
contains `before.json`, `batch.json`, `batch.log`, `verification.json` and
[`summary/summary.json`](../build/fpga-pa-control/a2-20261017/summary/summary.json)
plus CSV. Full original runs are under
`build/fpga-pa-control/a1-20261017/a2-plan/runs/`. A separate
`build/fpga-pa-control/a2-20261017.tar.gz` archive preserves the reports, tooling
and all twelve run directories, with a `.sha256` checksum alongside it.

The [summarizer](../firmware/tools/summarize_fpga_experiments.py) records every
measurement's origin. Failed nextpnr runs left `timing.json` available here;
the summary reads that report even though the runner stopped on exit 1. Log
fallback uses the final observation per clock and distinguishes estimates from
post-route measurements. Four summary regression tests passed, including real
padded nextpnr lines and failed/incomplete run handling. See the
[runner guide](../firmware/tools/README.md#summarize-the-matrix) for reproduction.

All recorded artifact hashes and source inventories were checked. A1 reference
and toolchain verification passed after the matrix. The six production FPGA
artifact hashes, including both active firmware headers, remain unchanged.
The original packaged image remains
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.
No candidate was installed or programmed and no radio application was launched.
A2 is complete; A3 has not started.
