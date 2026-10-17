# Isolated FPGA experiment runner

`fpga_experiment.py` prepares a verified reference and runs separate FPGA build
experiments. A1 covers freezing the reference, recording the toolchain and
rollback files, running the existing reference checks, and writing an A2 plan.
Real synthesis, placement/routing and packing belong to A2. Do not run `run` or
the generated `commands.sh` during A1. The runner has no programming target and
never launches a radio application or loads a kernel module.

Run the commands below from the repository root. Every output directory must be
new: existing directories and symlinks are rejected, including failed attempts.
Outputs are also rejected inside `firmware/`, `software/`, `docs/`, `.git/`, or
inside or around the frozen reference. Choose a fresh suffix for another attempt;
preserve the previous attempt as evidence.

## A1: freeze, verify and plan

The existing reference checks are documented in
[`firmware/tests/README.md`](../tests/README.md). `--checks-json` optionally copies
an already-produced JSON report and the logs named by its `checks[*].log` fields;
`freeze` does not run those checks itself.

```sh
python3 firmware/tools/fpga_experiment.py freeze \
  --repo . \
  --output /tmp/cariboulite-fpga-a1-reference \
  --checks-json /tmp/cariboulite-a1-reference-checks.json
python3 firmware/tools/fpga_experiment.py verify \
  --reference /tmp/cariboulite-fpga-a1-reference
python3 firmware/tools/fpga_experiment.py plan \
  --reference /tmp/cariboulite-fpga-a1-reference \
  --output /tmp/cariboulite-fpga-a2-plan
```

Omit `--checks-json` when no report is available. The reference requires the
firmware HDL, pin constraints, Makefile, packaged image and both active headers
to match the recorded Git commit. Other preparation work may be uncommitted;
the manifest retains the complete checkout status. The byte array in each active
header must also match `top.bin` exactly.

The packaged reference inspected during A1 has these full SHA-256 hashes:

| File | SHA-256 |
| --- | --- |
| `firmware/top.bin` | `dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6` |
| `firmware/h-files/cariboulite_fpga_firmware.h` | `07179d788221c275da0c0a72005f77cf679a3ab30b5453308e9db9c292aac330` |
| `software/libcariboulite/src/cariboulite_fpga_firmware.h` | `07179d788221c275da0c0a72005f77cf679a3ab30b5453308e9db9c292aac330` |

The reference manifest records the full source commit and a full hash, byte count
and mode for each artifact. Its contents include:

- `source/firmware/`: committed HDL, `io.pcf` and Makefile inputs.
- `rollback/`: the image, both active headers, available host executables and
  shared libraries, build metadata, monitor sources and installed module files.
- `archives/`: the complete superproject source archive and available submodule
  archives at their recorded commits. Missing submodules remain explicit.
- `toolchain.json`: tool paths, binary hashes, version/help output and exit codes,
  OS/kernel/Python identity, and available Yosys ABC/data-file identities.
- `host-dependencies.json`, `modules.json` and `RESTORE.md`: dependency records and
  same-host restoration requirements.
- `reference.json`, `reference.sha256` and `SHA256SUMS`: the manifest and checksum
  indexes. `verify` checks the exact inventory, content, modes and indexes, and
  rejects added files or symlink artifacts.

Tool version probes perform no build. `icepack -h` may return 1 while printing
valid usage text; the recorded probe exit code is retained. Tool binaries and
system runtime libraries are identified rather than bundled. A reference records
the packaged image; it does not read back or verify the currently programmed FPGA.

`plan` writes `plan.json` and `commands.sh`, without building anything. It produces
twelve fresh-output commands: default/CE2 synthesis × parallel/serial refinement
× three repetitions, all at seed 16. Each command uses the runner copied into the
verified reference. The generated script continues after failed runs, retains
every result, and returns failure if any run failed.

## Snapshot an HDL candidate

Use `candidate` for edits to the existing HDL input files while keeping the
committed reference intact. This permits experiments before committing the
candidate, without describing its source as the committed baseline:

```sh
python3 firmware/tools/fpga_experiment.py candidate \
  --reference /tmp/cariboulite-fpga-a1-reference \
  --repo . \
  --output /tmp/cariboulite-fpga-candidate
python3 /tmp/cariboulite-fpga-candidate/tools/fpga_experiment.py verify \
  --reference /tmp/cariboulite-fpga-candidate
```

The candidate copies the reference into a separate, fresh directory and replaces
its existing HDL inputs with a stable working-tree snapshot. The recursive HDL
filename set, `io.pcf` and Makefile must remain unchanged. New/deleted HDL files
or different build constraints require a separately reviewed scope.

`reference.json` identifies it as `fpga-candidate`, records working-tree status
and changed files, and marks `firmware_source_matches_revision` false.
`candidate/hdl.patch` and `candidate/base-reference.json` preserve the source diff
and original reference identities. `verify` checks the inherited artifacts and
the candidate's complete inventory. Run experiments with the runner frozen into
that candidate, using its directory as `--reference`.

The inherited packaged image, headers, host files, source archive and reference
check logs still describe the original baseline. The image is explicitly
`rollback_only`; it was not built from candidate HDL. Store new simulation and
build results separately. Normal `freeze` continues to require committed inputs.

## A2: execute an isolated experiment

The following command is an A2 operation. It synthesizes, places/routes and packs
one experiment using frozen inputs; it does not program the resulting image.

```sh
python3 /tmp/cariboulite-fpga-a1-reference/tools/fpga_experiment.py run \
  --reference /tmp/cariboulite-fpga-a1-reference \
  --output /tmp/cariboulite-fpga-a2-default-parallel-s16-r1 \
  --synthesis default --refinement parallel --seed 16 --repeat 1
```

For the planned matrix, execute its generated script only during A2:

```sh
sh /tmp/cariboulite-fpga-a2-plan/commands.sh
```

`--synthesis` accepts `default` or `ce2`; `--refinement` accepts `parallel` or
`serial`. The default seed is 16 and repeat label is 1. `--repeat` labels one
attempt; it does not repeat a build internally. `--timeout` sets a finite positive
timeout per stage, in seconds, defaulting to 1800. Optional `--yosys`, `--nextpnr`
and `--icepack` paths must identify binaries with the frozen hashes. The runner
also checks any recorded Yosys ABC/data files and `YOSYS_DATDIR` before and after
the experiment. A changed reference or toolchain cannot publish a passing result.

For default synthesis, seed 16 and parallel refinement, the current commands in
the isolated `work/` directory are as follows. Executable paths come from the
frozen toolchain; CE2 adds `-dffe_min_ce_use 2` to `synth_ice40`, and serial
refinement omits `--parallel-refine`.

```sh
yosys -Q -T -p 'synth_ice40 -top top -json top.json -blif top.blif' top.v
nextpnr-ice40 --lp1k --package qn84 --json top.json --pcf io.pcf \
  --asc top.asc --write routed.json --report timing.json \
  --opt-timing --seed 16 --no-promote-globals --parallel-refine
icepack top.asc top.bin
```

These commands do not invoke the production Makefile or copy a generated header
into the library. Each result directory retains `work/`, `logs/`, `tmp/` and
`result.json`. The summary records exact argv, working directories, stage exit
codes, durations, warning lines, metrics and full hashes of retained artifacts,
including source inputs, netlists, routed JSON, reports, logs and any bitstream.
Failures, timeouts, interrupted stages and missing outputs remain available for
inspection; downstream stages stop after a failure. A zero tool exit code alone
does not make a stage pass: required outputs must be nonempty. Failed timing
prevents packing.

## A3: compare one build option at a time

The `run` command also accepts these explicit experiment options:

| Runner option | Tool command change | Default when omitted |
| --- | --- | --- |
| `--tmg-ripup` | Add nextpnr `--tmg-ripup` | Timing-driven ripup disabled |
| `--promote-globals` | Remove nextpnr `--no-promote-globals` | Automatic global promotion disabled |
| `--synthesis-device lp` | Add Yosys `-device lp` | Inherited synthesis device selection |

Use a fresh output directory for every run. Compare each option independently
against a fixed synthesis/refinement/seed recipe before testing combinations.
Installed Yosys help describes device selection as relevant to ABC9; inspect the
actual synthesis log when interpreting this experiment. These switches change
isolated commands, without editing the production Makefile or programming FPGA
hardware. The original A2 commands remain identical when the switches are omitted.

`result.json` records `tmg_ripup`, `promote_globals` and `synthesis_device` alongside
the exact stage commands. A frozen runner predating these options cannot accept
them: preserve a separate copy of the updated runner and its hash with the A3
plan, while continuing to use the verified frozen HDL/toolchain reference.

## Summarize the matrix

After the matrix completes, generate JSON and CSV comparisons in a fresh output
directory. This command only reads the plan, reference and retained runs:

```sh
python3 firmware/tools/summarize_fpga_experiments.py \
  --plan /tmp/cariboulite-fpga-a2-plan/plan.json \
  --output /tmp/cariboulite-fpga-a2-summary
```

The summary includes every planned run, stage status/exit codes/durations,
resource and clock ranges, warnings, artifact hashes and counts of distinct
source, netlist, routed-design and image hashes. Missing and incomplete runs
remain visible. A successful summary command means the report was written;
inspect its `status_counts` to determine build outcomes.

Groups and CSV columns include all three A3 options so distinct recipes remain
separate. Older plans and results use the original defaults. For completed runs,
recorded result options take precedence over plan options; planned options also
label missing runs.

Metrics prefer `result.json`, then the retained `work/timing.json`, then the
last observation of each clock in `logs/routing.log`. This preserves measurements
when nextpnr exits on failed timing before the experiment runner parses its
report. Each measurement records its evidence source; log measurements before
routing completes are marked as estimates. A failing run remains failed even
when it produced placement or report files. The summarizer also checks retained
artifact hashes against each completed run's inventory.

## Timing interpretation

Routing must report every clock meeting its constraint, include the required
62.5 MHz system and 64 MHz LVDS constraints, and keep resource use within capacity.
The runner never adds `--timing-allow-fail`. Read the complete logs, retained
warning lines, timing report and available critical paths alongside the summary.

`clock_period_margin_ns` is calculated as
`1000 / constraint_MHz - 1000 / achieved_MHz`. It is a full-period comparison
derived from reported Fmax, **not true worst-path slack**. `worst_slack_ns` stays
`null`; structured critical paths may also be absent. In particular, this margin
does not establish slack for the receiver's half-cycle DDR paths. Neither passing
the existing simulations nor passing routed constraints establishes physical
DDR capture, complete asynchronous-interface behavior or RF performance. See the
[LVDS timing plan](../../docs/lvds-fmax-next-test-2026-10-08.md) for the subsequent
acceptance checks.

## Rollback requirements

Keep the frozen bundle separate from every experiment and verify it before use.
`RESTORE.md` identifies the original paths for the preserved image, active headers
and compatible host files. Restore the image and both headers together when
restoring the repository firmware baseline. No runner command automatically
restores files, installs modules, programs the FPGA or starts an application.

The preserved host executables depend on their recorded shared libraries and
system runtime dependencies. Their RUNPATH may refer to the original build
directory; running from the bundle requires `LD_LIBRARY_PATH` to point at its
`rollback/build` library directory. Copied installed modules require the recorded
compatible kernel. Check `missing_host_files`, submodule availability and module
records before relying on the bundle for recovery. The copied local binaries and
available compiler/dependency metadata are rollback evidence, not a complete
reproducible-build or self-contained operating-system image.
