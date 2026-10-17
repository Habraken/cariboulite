# A1 reference and isolated experiment preparation

Completed 2026-10-17 on `dev-fpga-pa-control`, for
[PA-control plan A1](fpga-pa-control-plan-2026-10-17.md#a1-freeze-the-reference-and-prepare-isolated-experiments).
The reference checks passed and the twelve A2 commands are prepared. No real
synthesis, placement/routing, packing, hardware programming or RF tests ran.
The subsequent [A2 matrix is now complete](fpga-a2-build-matrix-2026-10-17.md);
the text below records the A1 preparation state.

## Frozen reference

The source baseline is `b190aea78fade5fa88ba863158d0a1895b6ea6d6`. The checkout
was clean when A1 began; `baseline-start.json` preserves that receipt. The freeze
manifest also records the then-uncommitted preparation files. Every frozen HDL,
PCF, Makefile, image and active-header input matches the committed baseline;
frozen input bytes come from Git rather than a later working-tree copy.

Local artifacts live under the ignored directory
[`build/fpga-pa-control/a1-20261017/`](../build/fpga-pa-control/a1-20261017/):

| Artifact | Purpose |
| --- | --- |
| `reference/reference.json` | Revision, working-tree state, target, source and artifact inventory |
| `reference/reference.sha256`, `reference/SHA256SUMS` | Manifest and complete reference checksums |
| `reference/source/firmware/` | Frozen HDL, pin constraints and Makefile |
| `reference/rollback/` | Image, active headers, compatible host artifacts and installed SMI modules |
| `reference/archives/` | Superproject and IIR submodule source archives |
| `reference/toolchain.json` | Tool versions/help, paths, executable and synthesis-data identities |
| `reference/checks/` | Six reference-check results and preserved logs |
| `reference/tools/fpga_experiment.py` | Frozen runner used by the generated plan |
| `a2-plan/plan.json`, `a2-plan/commands.sh` | Twelve explicit A2 commands, not executed |
| `validation/`, `a1-validation.json` | Runner regression evidence and A1 verification receipt |

The reference manifest SHA-256 is
`07beab053c259f6e473ee8d282a032c6120a439bb40db56ab0d44a3e7ab3ce52`.
The complete reference contains 52 inventoried payload files, plus its manifest
and checksum indexes. A portable copy is saved alongside the directory as
`build/fpga-pa-control/a1-20261017.tar.gz`, with its own `.sha256` checksum file.
These local rollback artifacts are intentionally outside Git; copy the archive
and checksum to separate storage for an independent backup.

## Image, host build and tools

| Reference | SHA-256 |
| --- | --- |
| `firmware/top.bin` (32,220 bytes) | `dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6` |
| Both active firmware header files | `07179d788221c275da0c0a72005f77cf679a3ab30b5453308e9db9c292aac330` |

Both active header byte arrays and the image embedded in the preserved
`libcariboulite.so` match `top.bin`. The test app, radio backend/controller,
programmer, monitor sources, shared libraries and available host build metadata
are preserved. `build/compile_commands.json` was unavailable and is recorded as
missing; the available CMake cache and test-app compile/link flags were saved.
The IIR submodule was clean at `fc8634d15b470a00ba4859915fec0047bb486dd4`
and its committed source was archived.

The host executables use the original build directory in RUNPATH, so rollback
requires the matching library and system dependencies. The bundle includes
dependency metadata, same-kernel installed SMI modules and `RESTORE.md`.
This is a same-host rollback bundle, with local binaries and available build
metadata; it does not attest to a reproducible host rebuild. Installed module
files do not themselves identify a loaded module. The currently programmed
FPGA image was not read back or changed.

| Tool | Recorded version |
| --- | --- |
| Yosys | `0.60+64`, commit `d523c88c3` |
| nextpnr-ice40 | `0.9-49-g7bd1336f` |
| icepack | No version option; executable hash and usage captured |
| Icarus Verilog / vvp | `12.0` |
| GNU Make | `4.4.1` |
| C compiler / CMake | GCC `14.2.0` / CMake `3.31.6` |

The kernel is `6.18.50+rpt-rpi-v8`, architecture `aarch64`. Full tool hashes and
help output are in `toolchain.json`, including `yosys-abc` and 318 Yosys data
files under `/usr/local/share/yosys`. Runtime binaries and tool data are identified
rather than bundled. Python identity is recorded; the runner verifies the FPGA
executables, recorded ABC/data files and `YOSYS_DATDIR` before and after a run.

## Validation

All six existing reference checks passed:

| Check | Result |
| --- | --- |
| Build dependency/failure checks using fake tools | Pass |
| TX sequence simulation | Pass |
| TX stop simulation | Pass |
| FIFO reset simulation | Pass |
| Loopback simulation, 1,024 combinations | Pass |
| Driver quiet-IQ framing check | Pass |

All 20 new runner regression tests passed in 35.430 seconds. They use temporary
repositories and fake FPGA tools, covering stage ordering and failures, isolation,
timing rejection, timeouts, immutable inventories, changed tools, JSON failure
records and continuation through all twelve matrix entries after a failed run.
The test source and final log are saved under `validation/`.

The reference verifier and `sha256sum --check` passed. The frozen runner matches
the working copy, source archives match frozen inputs, and all six pre-existing
production artifact hashes remained unchanged. These checks establish the A1
reference and runner behavior; new routed timing and physical acceptance remain
in later steps. Historical Fmax values have not been reproduced by A1.

## Verify and continue

From the repository root, verify the saved bundle using its frozen runner:

```sh
python3 build/fpga-pa-control/a1-20261017/reference/tools/fpga_experiment.py verify \
  --reference build/fpga-pa-control/a1-20261017/reference
```

A2 will execute `a2-plan/commands.sh` explicitly. It contains three fresh runs of
each default/CE2 and parallel/serial combination at seed 16. Each run retains its
source, logs, netlists, routing report, routed design and any packed image in its
own directory. Existing outputs are never overwritten and failed runs remain
available. The runner invokes tools directly, avoiding the production Makefile's
firmware-header publication. No production build default changed.

The [runner guide](../firmware/tools/README.md) documents the commands, failure
handling and timing fields. The derived clock-period margin is not actual
worst-path slack; every clock must pass its real constraints, including system
62.5 MHz and LVDS 64 MHz. A2 was the next unstarted step at this freeze;
its later results are linked above.
