# Step 2: FPGA build headroom and external-PA control

Proposed execution plan, 2026-10-17. Step 1 is committed at `47887c1`, with user
confirmation of NBFM TX/RX in both applications and working WBFM RX. This plan
does not record new FPGA builds or physical tests. A1 preparation is complete;
see the [reference and rollback record](fpga-a1-reference-2026-10-17.md).

The deliverable is a repeatable, timing-qualified FPGA image with the agreed
one- or two-output PA interface, integrated into the shared TX lifecycle and
verified on the connector. Build experiments come before PA integration.

## Constraints and starting evidence

- Target: iCE40 LP1K/QN84, 1280 logic cells, 16 RAM blocks.
- Preserve the actual clock requirements: system at least 62.5 MHz, LVDS at
  least 64 MHz, and every other constrained clock passing. Preserve real DDR
  edge relationships; do not waive timing failures to accept a candidate.
- RAM is already 16/16 in the recorded baseline; plan for no additional RAM.
- Historical default build: 1022 logic cells, system Fmax 73.10 MHz, LVDS
  Fmax 74.96 MHz. Historical CE2 build: 1035 cells, 85.70 MHz system and
  56.24 MHz LVDS, failing LVDS timing. These are reference results to reproduce,
  not present-run acceptance data.
- CE2 means `-dffe_min_ce_use 2`. It changes clock-enable mapping; it is not
  a guaranteed area reduction. Measure logic use, global resources, routing
  and timing together. Final headroom is demonstrated by the integrated PA
  design passing, rather than by an arbitrary free-cell percentage.

The packaged working `firmware/top.bin` currently hashes to
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.
Record candidate image hashes separately from any firmware version reported by
the board; a version byte does not identify the entire programmed image.

## A. Qualify the FPGA build

### A1. Freeze the reference and prepare isolated experiments

Completed 2026-10-17 on `dev-fpga-pa-control`. The
[A1 record](fpga-a1-reference-2026-10-17.md) identifies the frozen artifacts,
checks and prepared A2 commands. A2 has not started.

1. Record the source revision, clean/modified state, FPGA RTL and pin constraints,
   tool versions, build commands and known working image/header hashes.
2. Preserve a rollback image and its compatible host build.
3. Use an isolated source copy or development checkout and a separate output
   directory per variant/run. The current firmware Makefile also publishes the
   library firmware header, so redirect that destination in experiments and keep
   production artifacts intact.
4. Prepare an experiment runner that captures logs, machine-readable summaries,
   elapsed time, source/netlist/placement/image hashes, resource use, clock timing,
   warnings and synthesis/routing/packing exit status. Retain failed runs too.
5. Run the existing FPGA simulations and build-dependency checks as the reference.

Output: a reference manifest, rollback bundle and repeatable experiment commands.
No hardware programming is needed for A1 through A4.

### A2. Reproduce default and CE2 builds before changing RTL

Keep the source, PCF, target, seed 16 and other build options fixed. Run three
fresh full builds for each cell below, for twelve initial builds:

| Synthesis | nextpnr refinement | Fresh builds |
| --- | --- | --- |
| Current default | Current parallel refinement | 3 |
| CE2 | Current parallel refinement | 3 |
| Current default | Serial, removing `--parallel-refine` | 3 |
| CE2 | Serial, removing `--parallel-refine` | 3 |

Keep `--opt-timing` and `--no-promote-globals` fixed in this matrix. If results
vary, compare source and netlist hashes first, then placement and routed paths.
Distinguish synthesis variation from routing variation. Do not select a candidate
from its single best Fmax result.

Output: a table of resource use, timing range, repeatability and failures. Measure
run duration here before estimating the cost of further sweeps.

### A3. Resolve the measured limitation with controlled experiments

Choose experiments from the A2 results, one factor at a time:

- If unchanged RTL already gives sufficient timing margin for the PA budget,
  retain it and proceed to qualification; an RTL rewrite is not mandatory.
- Compare `--tmg-ripup` independently, then global promotion by removing
  `--no-promote-globals`. Keep whichever refinement mode is being evaluated fixed.
- Inspect the installed Yosys help and consider `-device lp` as a separate
  experiment, since nextpnr targets LP1K while the current synthesis command does
  not explicitly select a synthesis device. Do not assume this improves results.
- If the DDR-input-to-decoder path remains limiting, implement the receive
  staging candidate in the existing [LVDS plan](lvds-fmax-next-test-2026-10-08.md):
  register both bits of each DDR pair and matching sync metadata before decoding.
  Preserve bit order, RF24 inversion, reset validity and FIFO push/data alignment.
- Only if still needed, evaluate the wide-enable refactor separately. Test any
  selected combination explicitly instead of adding together independent gains.

Build and compare default synthesis and CE2 for each retained RTL candidate.
Revert experiments without a demonstrated benefit. Keep system and LVDS timing
visible together; improving one clock does not compensate for failing another.

### A4. Verify any receive-RTL change before selecting it

Add a comparison bench against the frozen receiver for both modem channels:

- Exact IQ words, DDR ordering, RF24 inversion, sync metadata and specified latency.
- Consecutive frames, idle periods, malformed sync and resynchronization.
- Startup/reset in each receive phase, without stale pushes.
- FIFO-full transitions at completion, push/data alignment and channel changes.

Evaluate FIFO fullness at actual completion; do not simply delay the full flag
with input data. Run existing sequence, stop, FIFO reset, loopback and quiet-IQ
framing tests after changes. Existing TX benches alone do not validate RX DDR.

### A5. Freeze and physically accept the build baseline

For the selected source/tool/flag combination:

1. Obtain three passing fresh builds at seed 16, then assess seeds 1, 2 and 3.
2. Require synthesis, placement, routing, all clock checks and packing to pass in
   each acceptance run. Record the worst margin and remaining resources.
3. Exercise the chosen candidate physically on RF09/RF24, with NBFM RX/TX, WBFM
   RX on a suitable route, and repeated reset/start/stop. Compare against the
   working image and record the exact candidate hash.
4. Save the accepted build recipe, logs, image and rollback instructions before
   introducing PA logic.

Output: an accepted build baseline. If a seed fails, investigate or change the
candidate; do not silently exclude it from the results.

## B. Specify and integrate PA control

### B1. Agree the external interface and failure behavior

This specification can progress alongside A, but must be settled before wiring
or PA-control RTL is finalized:

| Decision | Required definition |
| --- | --- |
| Output count and roles | One PA enable, or two explicitly named functions; do not assume the second is an antenna relay |
| Pins and electrical interface | Connector pins, logic levels, polarity, drive/interface requirements and disconnected state |
| Timing | PA settling/lead delay, RF-off guard, release delay and any relationship between two outputs |
| Ownership | Shared host TX request, FPGA output sequencing and any hardware RF permit/inhibit |
| Failures | Normal stop, abort, failed start, reset, controller loss, host stall and re-arm policy |
| Observability | Requested state, FPGA sequence/output state, timeout/fault reason; distinguish these from physical PA feedback |

Account for control registers, counters, failure handling and any RF gate in the
logic budget, not only the one or two output flip-flops. Specify a PA-off state
through reset and an appropriate external default when the FPGA is unconfigured.

The previously documented PMOD branch/stash is absent from this checkout. Recover
it from the original host/archive if useful, or implement the reviewed contract
afresh; its availability must not block unrelated build experiments.

### B2. Add the smallest real output interface and measure the increment

The current PMOD value registers store state, but their output connection is
disabled in `top.v`. Implement actual mapped outputs with explicit reset values
and capability/register reporting. Add one output first, then the second if
required, against the accepted baseline. Measure resource and timing changes at
each increment. Start with the PA disconnected.

### B3. Implement the agreed sequence and RF coordination

Use an explicit request and sequence state. The existing SMI direction signal
does not distinguish idle from TX, and DMA/FIFO activity can include padding;
neither is a sufficient PA request.

The proposed normal sequence, subject to the PA specification, is:

1. Keep modem RF inactive; request the selected PA output state and await the
   agreed settling/sequence-ready condition.
2. Allow modem TX activation and the existing opening cue/audio only afterward.
   Modem activation already starts SMI/DMA, so a hook before the first writer
   sample would be too late.
3. On normal stop, complete the existing closing cue/drain, stop SMI and disable
   modem TX, confirm the required RF-off condition/guard, then release the PA
   and any second output in the specified order.
4. On abort or failed start, use the agreed bounded shutdown without depending
   on the normal audio tail. Latch/report faults and define explicit re-arm.

Zero audio in FM and zero-IQ padding do not prove carrier-off. Define and test
"PA disabled" separately from "CaribouLite RF inactive."

A host-stall guarantee needs an FPGA-owned timeout/lease refreshed by the healthy
control owner, not by continuing DMA activity. If the requirement also includes
autonomous removal of modem RF drive, select and verify a hardware RF-permit path
(for example a correctly controlled embedded TX enable, or an external inhibit).
The FPGA currently does not own the modem's SPI-off command; a PA output alone
must not be described as proof of autonomous modem shutdown.

### B4. Integrate through the shared software lifecycle

Add capability checks, request/ready/release hooks and status to the shared TX
path used by both applications. Older firmware must receive an explicit supported
fallback or rejection according to the agreed interface. Surface requested and
applied sequence/output state through both apps without claiming physical PA
feedback when none exists. Add mocked tests for ordering, delayed readiness,
timeouts, disconnects and cleanup.

### B5. Qualify the complete design, then measure the pins

1. Simulate normal transitions, rapid PTT, reset in every state, failures,
   expired leases and required re-arm behavior.
2. Repeat routed timing/resource qualification on the complete PA design. A
   passing baseline without PA logic does not establish an integrated fit.
3. With the PA disconnected, measure connector output levels and lead/release
   ordering using suitable instrumentation; correlate them with actual RF start
   and stop. Test startup, stop, abort, reset and host-loss behavior.
4. Recheck NBFM TX/RX and WBFM RX, then validate the connected PA chain using the
   selected electrical interface and switching specification.

Output: the final image and host revision, repeatable build recipe, simulation
results, measured timing traces, operating configuration and rollback bundle.
Step 2 completes when the defined PA behavior and existing radio operation pass,
allowing the end-to-end NBFM transceiver qualification in roadmap step 3.

## Proposed reviewable changes

1. Experiment runner and result manifest, without changing production build defaults.
2. Selected build recipe and any separately verified RX timing change.
3. Defined PA register/output interface and sequencing RTL.
4. Shared host integration, regression tests and status.
5. Physical acceptance record and adoption of the tested image.

Primary tool references: [Yosys iCE40 synthesis options](https://github.com/YosysHQ/yosys/blob/main/techlibs/ice40/synth_ice40.cc)
and [nextpnr clock/IO constraints](https://github.com/YosysHQ/nextpnr/blob/main/docs/constraints.md).
Installed versions and their help output must be recorded for the experiments.
