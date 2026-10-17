# A3 controlled build-option experiments

Completed 2026-10-17 on `dev-fpga-pa-control`, starting at revision
`3caaad6`. Sixteen fresh builds produced twelve passes and four retained failed
screens. Retain the original **CE2 synthesis with serial refinement** for A5.
These experiments keep the receiver RTL unchanged.

## Controlled inputs

All runs use the reserved-pad cleanup source from
[A2 requalification](fpga-a2-build-matrix-2026-10-17.md), frozen candidate manifest
SHA-256 `3f7657e5ee579912ab6af08c41734e1d5f54ea775f93745ca8c12bab4035e982`.
The committed working-tree HDL matches that snapshot exactly. Toolchain remains
Yosys `0.60+64` (`d523c88c3`) and nextpnr `0.9-49-g7bd1336f`.

Fixed inputs are LP1K/QN84, PCF, serial refinement, seed 16, `--opt-timing` and
all clock constraints. The system requires 62.5 MHz and LVDS requires 64 MHz;
every other reported clock must also pass. No timing failures are waived.

The screen compares both default and CE2 synthesis with each independent change:

| Variant | Change from its synthesis baseline |
| --- | --- |
| Baseline | None |
| Ripup | Add nextpnr `--tmg-ripup` |
| Globals | Remove nextpnr `--no-promote-globals` |
| LP | Add Yosys `-device lp` |

Installed Yosys help says device selection affects ABC9, which is active in the
actual synthesis logs. The LP trial therefore changes synthesis optimization;
the nextpnr device remains LP1K in every run. Installed help is retained with the
experiment. No combination of factors is selected or inferred from separate runs.

The recorded policy runs one fresh screen per recipe, followed by two additional
fresh builds of each passing recipe. Failed screens remain recorded and are not
treated as qualified recipes. Every run has a separate directory and finite
stage timeout; the production Makefile is not invoked.

## Results

Each passing recipe has three fresh passing builds with identical source,
netlist, routed-design, ASC and image hashes within that recipe. Final routed
timing is shown; placement failures have no routed Fmax or packed image.

| Synthesis | Variant | Passed | Logic cells / 1280 | Globals / 8 | System MHz | LVDS MHz | Outcome |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| CE2 | Baseline | 3/3 | 1035 | 1 | 72.76 | 75.90 | Preferred; identical images |
| CE2 | Ripup | 3/3 | 1035 | 1 | 72.76 | 75.90 | Same image as baseline |
| CE2 | Globals | 0/1 | 1035 | 8 | 82.77 | **62.08** | Fails LVDS timing, exit 1 |
| CE2 | LP | 3/3 | 1037 | 1 | 72.39 | 67.82 | More cells and less timing margin |
| Default | Baseline | 0/1 | 1034 | 1 | — | — | Placement fails, exit 255 |
| Default | Ripup | 0/1 | 1034 | 1 | — | — | Same placement failure |
| Default | Globals | 0/1 | 1034 | 8 | — | — | Placement fails, exit 255 |
| Default | LP | 3/3 | 1033 | 1 | 76.56 | 65.80 | Passes, with little LVDS margin |

All variants use 16/16 RAM blocks. The default baseline and ripup fail to place
`sys_ctrl_ins.rx_sync_24_SB_DFFER_D_DFFLC`; default globals fails to place
`spi_if_ins.o_fetch_cmd_SB_DFFESR_Q_DFFLC`. Timing-driven ripup acts after
placement, so it cannot remedy those placement failures.

## Timing and selection

For the preferred recipe, the critical system path is RX FIFO pointer arithmetic
through the SMI read-request logic. The limiting LVDS path remains the RF24 DDR
falling-edge capture through receive-enable decoding. Routing dominates both.
Summing the structured report's delay components, including skew and setup:

| Recipe | System path / 16 ns budget | System setup margin | LVDS path / 7.8125 ns budget | LVDS setup margin |
| --- | ---: | ---: | ---: | ---: |
| CE2 baseline | 13.744 ns | 2.256 ns | 6.588 ns | 1.2245 ns |
| CE2 ripup | 13.744 ns | 2.256 ns | 6.588 ns | 1.2245 ns |
| CE2 globals | 12.081 ns | 3.919 ns | 8.054 ns | **−0.2415 ns** |
| CE2 LP | 13.815 ns | 2.185 ns | 7.373 ns | 0.4395 ns |
| Default LP | 13.061 ns | 2.939 ns | 7.599 ns | 0.2135 ns |

These are derived same-clock setup margins from the reported paths. The LVDS
budget uses nextpnr's nominal 50% duty-cycle model for opposite clock edges;
the full-period Fmax comparison would overstate this half-cycle margin by two.
Cross-clock/asynchronous paths receive no derived acceptance margin. Physical
waveform, external setup/hold and connector behavior remain hardware checks.

Default LP saves only two cells relative to CE2 baseline while losing about
1.01 ns of LVDS setup margin. CE2 LP uses two more cells and worsens both primary
clock margins. Global promotion exhausts global buffers and fails; ripup adds
no measured gain. The original CE2 recipe therefore offers the preferred balance:
**245 free cells**, 1/8 globals, and better LVDS margin than either LP variant.
These figures do not establish that the complete PA sequencer will fit; its
resource use and routed timing must be measured after integration.

The selected recipe also passes the other clocks: input/global 463.61 MHz
against 125 MHz, SPI 195.81 MHz against 5 MHz, and SMI request 137.59 MHz
against 16 MHz. All three fresh selected builds reproduce these measurements.

No measured failure in the selected recipe currently justifies RX staging or
the wide-enable refactor. The receiver-comparison work in A4 is conditional on
such an RTL change. Seed assessment and physical acceptance remain A5 work.

## Reproduction and retained evidence

The [runner guide](../firmware/tools/README.md#a3-compare-one-build-option-at-a-time)
documents the three explicit controls. The summary keeps option combinations
separate in JSON groups and CSV columns, including legacy defaults for A2.
All **27 runner tests and 6 summary tests pass**. The fresh baseline also matches
the earlier netlist, routed design, ASC and packed-image hashes exactly.

Evidence is retained under
[`build/fpga-pa-control/a3-20261017/`](../build/fpga-pa-control/a3-20261017/):
frozen tool scripts and tests, `installed-help.json`, `before.json`, screen and
confirmation plans, exact commands, execution receipts, every run's logs and
artifacts, summaries, and a same-clock path-analysis helper.
`provisional-recipe.json` captures the selected commands and identities;
`verification.json` records successful checks of every run's artifact hashes,
the frozen reference, source and constraint equality, exact experiment options,
strict clock/packing status, deterministic repeats and all six unchanged
production artifacts. An archive and SHA-256 sidecar are retained alongside the
directory as `a3-20261017.tar.gz` and `a3-20261017.tar.gz.sha256`.

The preferred image SHA-256 remains
`78f650fdf192f8a3c11fc45a253d08b90071e78671d59100ab9de7b5b38ff7c2`,
the image already [smoke-tested in both applications](fpga-ce2-programming-2026-10-17.md).
The packaged production image remains
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.
No image was installed and the FPGA was not programmed during A3. No production
HDL, pin constraints, Makefile or packaged artifacts changed. A5 seeds 1, 2 and 3
and the remaining physical acceptance checks are pending.
