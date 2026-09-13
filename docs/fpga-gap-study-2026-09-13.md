# FPGA gap feasibility study — 2026-09-13

No repository HDL, working bitstream, or embedded firmware header was changed.
No FPGA was programmed. Icarus Verilog was installed for simulation.

Local study directory:
`installations/fpga-gap-study/20260913T165728Z/` (ignored by Git).
It contains full firmware copies in `baseline/` and `gap-only/`, a `rollback/`
copy of the working bitstream and both firmware headers, and `hashes.json`.
All three original artifact hashes were rechecked after the study and match.
The working bitstream SHA-256 remains
`fe894f203f8c2fed4861ddac22890cbdc7d748ccf7fb6f1ac32bb950decc048d`.

## Isolated change and synthesis

Only the study's `gap-only/sys_ctrl.v` adds:

```verilog
assign o_tx_sample_gap = tx_sample_gap;
```

Loopback remains disconnected. Each copy was synthesized with:

```sh
yosys -Q -T -p 'synth_ice40 -top top -json study.json' top.v
```

| Synthesis cells | Baseline | Gap connected |
| --- | ---: | ---: |
| SB_LUT4 | 626 | 638 |
| SB_DFF variants, total | 581 | 585 |
| SB_RAM40_4K | 16 | 16 |

These are fresh source builds, not reconstruction of the exact saved bitstream.
They retain the existing reset/undriven-signal issues. Synthesis success does
not resolve those issues.

## Preliminary simulation

Icarus rejected the original `lvds_tx.v` because `INIT` is undefined and its
variable range part-select is not legal standard syntax. The simulation-only
copy uses `IDLE` for reset and `r_fifo_data[2*r_phase_count +: 2]` for the
two-bit select. These adjustments were not applied to the PNR copies.

The bench supplies a constant FIFO word, holds TX enabled with FIFO nonempty,
and reconstructs words from the serializer's two-bit output. For gap values
0 through 9, successive matching data words appeared every `gap + 1` word
slots, with zero spacing mismatches. Bench, simulation source and output are
in `simulation/`.

This is diagnostic steady-state evidence only. It does not validate the
unchanged reset behavior, FIFO latency/data ordering, all intervening idle-word
contents, dynamic gap changes, underruns, or physical DDR timing.

## Place and route

Both isolated builds used:

```sh
nextpnr-ice40 --lp1k --package qn84 --json study.json --pcf io.pcf \
  --asc study.asc --parallel-refine --opt-timing --seed 16 \
  --no-promote-globals --report study-timing.json
```

`--timing-allow-fail` was deliberately omitted. Logs are `study-pnr.log`.

- Gap-connected build: exit 0; 1,027/1,280 logic cells, 16/16 RAM blocks.
  Final LVDS clock maximum 65.70 MHz, passing the 64 MHz constraint.
- Baseline rebuild: exit 1; final LVDS maximum 61.37 MHz, failing 64 MHz.

This run demonstrates that connecting the gap need not exhaust resources.
It does not establish robust timing closure: the passing LVDS margin is small,
and the baseline failure shows sensitivity to placement/routing. Timing passes
apply only to the constraints and paths analyzed by this build.

## Next steps and rollback

Before programming: resolve reset/simulation syntax explicitly, extend HDL
tests to FIFO consumption and zero-word contents, and rerun synthesis/PNR on
the exact candidate source. Review resource/timing results before packing or
programming. No candidate binary or embedded header was generated here.

There is no active HDL change to undo. Study directories can be retained for
comparison; the running FPGA and working firmware files remain unchanged.

## Exact-source follow-up

`candidate/` preserves the gap connection plus `INIT` → `IDLE` and the legal
indexed part-select. A registered-output FIFO model feeding distinct words
exposed dropped samples: the pending load could be overwritten by state-machine
assignments. Constant-word spacing alone had hidden this problem.

`candidate-v2/` preserves a proposed correction:

- Always enter TX_FRAME after the first FIFO pull, allowing its pending load
  to reach the output before scheduling gap frames.
- Preserve a pending final word when the FIFO becomes empty in continuous TX.
- Update the previous TX-enable state at frame boundaries, where edge handling
  occurs, so a one-clock edge indication cannot disappear between boundaries.
- Use a blocking assignment consistently for the `next_sync` temporary.

Tests compile the exact candidate `lvds_tx.v` with Icarus:

```sh
iverilog -g2012 -s tb -o sequence-sim tb_sequence.v lvds_tx.v
vvp sequence-sim
iverilog -g2012 -s tb -o stop-sim tb_stop.v lvds_tx.v
vvp stop-sim
```

The sequence test passes for all gap values 0–15: 60 distinct words per setting,
in order, no missing/extra words, exact zero-word counts between data words
within each burst, initial transmission, empty/refill and settled stop/restart.
The active-stop test passes all 16 phase offsets at all 16 gap settings: TX
settles idle, stops pulling, and resumes after re-enable. It does not guarantee
delivery of in-flight samples across an active stop. Gaps outside the modem's
documented sample-rate choices are tested as HDL cases only.

The FIFO model matches registered read-data behavior; it is not a simulation
of the full asynchronous FIFO or the physical LVDS I/O cells. Dynamic gap
changes and a complete top-level simulation remain untested. Debug input is
held low in the benches; its top-level output remains disconnected.

Exact candidate synthesis: 637 LUT4 cells, 587 flip-flops, 16 RAM blocks.
PNR with the same options and seed 16 failed during placement (exit 255),
unable to place `r_counter_SB_DFFSR_Q_D_SB_LUT4_O_LC` after 149,605 attempts.
No timing acceptance or candidate bitstream exists for this corrected source.
Logs, benches and `source-hashes.json` remain in `candidate-v2/`.

Next work is placement/resource investigation of this fixed-source candidate,
with any alternative build settings recorded explicitly. Do not program it
based on the earlier gap-only PNR pass: that was different HDL.
Repository HDL and working firmware artifacts remain unchanged.

## Alternative placement trials

Three additional runs used the unchanged `candidate-v2/candidate.json`, the
same device, pin constraints and timing requirements. All exited 255 during
placement; none reached routing or timing acceptance.

| Log prefix | Change from original command | Result |
| --- | --- | --- |
| `candidate-seed1` | `--seed 1` | Could not place `smi_ctrl_ins.r_dir_SB_DFFER_Q_DFFLC` after 149,605 attempts. |
| `candidate-sa16` | `--placer sa`, seed 16 | Relative-constraint legalization failed for a chain starting at `lvds_rx_09_inst.o_fifo_data_SB_DFFESR_Q_D_SB_LUT4_O_LC`. |
| `candidate-heap16-timeout1` | Seed 16, `--placer-heap-cell-placement-timeout 1` | Increased limit to 1,196,836 attempts per cell; still could not place all cells. |

All other options remained `--lp1k --package qn84 --pcf io.pcf
--parallel-refine --opt-timing --no-promote-globals`; each run had separate
ASC/report output paths and a `<prefix>-pnr.log` in `candidate-v2/`.
No timing constraints were relaxed and `--timing-allow-fail` was not used.

Reported utilization is 1,026/1,280 logic cells (80%) and 16/16 RAM blocks
(100%). Aggregate free logic does not guarantee legal placement under the
device's cell-sharing and connectivity constraints. These failures do not
prove that the design can never fit, but seed changes and a larger placement
attempt limit have not resolved the problem.

Next step: inspect packing and constrained cell chains, then seek a small
resource/placement improvement in another isolated candidate. Rerun the
sequence and stop tests on any HDL change before evaluating PNR. Avoid
removing loopback/debug logic blindly: disconnected logic may already have
been optimized away. The corrected candidate is not ready to program.

The recorded hashes of all candidate Verilog files and all three working
firmware artifacts were verified again. No repository HDL was changed and
no bitstream was packed or programmed during these trials.

## PMOD RX-marker experiment

`candidate-no-pmod/` copies the candidate-v2 Verilog and pin constraints.
Only two assignments in `top.v` change: `w_rx_sync_input_09` and
`w_rx_sync_input_24` now take their software marker values directly, rather
than selecting PMOD inputs when the corresponding source bits are set.
Software marker support remains. External RX markers no longer function in
this candidate, although their configuration bits remain readable/writable.
The unused TX-marker wiring and PMOD port declarations are unchanged.

Both existing TX benches pass. These check sample order, gaps and stop/restart;
they do not exercise top-level RX metadata or physical PMOD pins.
`source-hashes.json` records the candidate Verilog files.

Synthesis and PNR used the same commands as candidate-v2, with seed 16 and
the default heap placer/attempt limit. Outputs are `candidate.json`,
`candidate.asc`, `candidate-timing.json`, `candidate-synthesis.log` and
`candidate-pnr.log` in the new directory. Both tools exited zero.

| Metric | candidate-v2 | Software-only RX markers |
| --- | ---: | ---: |
| LUT4 | 637 | 645 |
| Flip-flops | 587 | 587 |
| RAM blocks | 16 | 16 |
| Packed logic cells | 1,026 | 1,031 |
| Seed-16 PNR | Placement failed | Passed |

Final routed timing passes all reported clock targets. LVDS reaches
68.68 MHz against 64 MHz; the system clock reaches 78.49 MHz against
62.5 MHz. This is one passing run under the existing constraints, not a
hardware validation or proof of robust closure across placements.

Removing the input selection did not reduce aggregate logic utilization;
instead, the changed synthesized/packed design happened to place and route
successfully. This supports further evaluation of this candidate, but does
not establish PMOD routing as the sole cause of previous failures.

Working bitstream/header hashes were verified unchanged. No repository HDL
was changed, no binary was packed and no FPGA was programmed. Before hardware
use, review the loss of external-marker selection and prepare an explicit
candidate image and rollback procedure.

## Two-LED hardware-test image

`candidate-leds-on/` derives from `candidate-no-pmod/`, changing only
`io_ctrl.v`'s `led0_state` reset value to 1. Both LED registers now reset to
1, which lights both LEDs according to schematic page 4 (active-high outputs).
Software LED control remains available and can change this indication.
Earlier low-output experiments in `candidate-leds/` had the wrong polarity
and must not be used as the requested two-LED image.

The exact two-LED source passes both TX simulation suites, synthesis and
seed-16 PNR with unchanged constraints. Final LVDS Fmax is 75.60 MHz
(required 64); system Fmax is 79.26 MHz (required 62.5). `icepack candidate.asc
candidate.bin` produced SHA-256
`8441beeb12c2484d98bea8c5108b9bcc7cf61ba59943adf8af0f794b8454aaea`.
Source hashes and all build/test logs are saved with the image.

`gap-program.c` and its compiled helper are saved in that directory. The
helper uses production initialization to claim device ownership before GPIO
setup, forces file-based programming, checks the result and releases resources.
It links to the local build library. With the radio released, use:

```sh
study=installations/fpga-gap-study/20260913T165728Z
"$study/candidate-leds-on/gap-program" "$study/candidate-leds-on/candidate.bin"
# Rollback with the same helper:
"$study/candidate-leds-on/gap-program" "$study/rollback/firmware/top.bin"
```

After the user released their app, programming succeeded (`PROGRAM_RESULT=0`,
`program.log`). The local test app initialized and exercised option 11 TX
briefly at its displayed 430099936 Hz / -3 dBm, option 12 RX, and option 14
TX/RX toggles. It stopped the streams and exited normally. The monitor's
terminal rendering was incomplete in the tool output; no instrumentation
values are asserted from that screen.

`hardware-test.log` contains repeated RX timeouts in both receive runs.
The user subsequently confirmed that both LEDs are lit and the TX and RX
tests succeeded. This completes the basic hardware smoke test with user
confirmation of the visible indicator and radio operation. The RX timeouts
remain an unresolved log observation; successful audio does not establish
loss-free streaming. Nonzero sample gaps have not yet been tested on
hardware. The candidate remains loaded with the app
exited and transmission stopped. Original firmware/header files remain
unchanged; no image has been installed over the normal option-3 firmware path.

## 4 MS/s versus 2 MS/s TX experiment

The standalone `tx-rate-test.c` and executable in the study directory generate
a periodic 600 Hz FM tone with 2.5 kHz deviation at the selected RF sample
rate. They use S1G at 430099936 Hz and -3 dBm, transmit for approximately
three seconds, count samples accepted by the radio write API and stop TX.
The first 0.75 seconds are excluded from the reported steady measurement
to reduce initial buffer-fill bias. Partial writes retain the source offset.
This measures host-to-driver acceptance, not physical FPGA FIFO counters or
RF waveform fidelity. The option-14 puts/gets counters count software frames;
with 10 ms frames they should remain approximately 100/s at either rate.

The initial 2 MS/s attempt aborted before TX because gap readback was zero.
Investigation found a library return-code mismatch:
`io_utils_spi_transmit` returns zero on success, whereas
`caribou_fpga_spi_transfer` expects two transferred bytes and returns 1.
Consequently, the gap setter aborts its read-modify-write before writing.
The radio rate setter ignores that error. Delayed retries confirmed this.
Logs: `tx-rate-2m.out/.log` and `tx-rate-2m-retry.out/.log`.

The final isolated helper bypasses that wrapper for system register 6,
checks zero-success SPI returns, preserves the upper nibble when setting
the gap, and verifies readback before TX. No normal library source was
modified. Its cleanup restores modem 4 MS/s and writes gap/source register
zero (software sync sources); the normal application's source was unchanged.

| Test | Modem SR | FPGA gap | Accepted samples/s after warmup |
| --- | ---: | ---: | ---: |
| 4 MS/s baseline | 1 | 0 | 3,998,293.410 |
| 2 MS/s direct-register test | 2 | 1 | 1,999,154.792 |

Both runs exited zero and reported zero zero-length write returns. The
measured 2 MS/s rate is approximately 50% of baseline. Logs/results are
`tx-rate-4m.out/.log` and `tx-rate-2m-direct.out/.log`. The initial baseline
preceded the helper's direct-register workaround; its default gap was zero.
Finite buffering and short test duration limit precision; these results do
not independently prove absence of hardware underflow or sample loss.

The candidate FPGA remains loaded, TX is stopped and the app is released.
User confirmation of the reduced-rate tone remains pending. Next software
work is correcting and testing the FPGA SPI wrapper's success convention
before integrating adjustable TX rate into the normal audio pipeline.

## Intermittent TX startup follow-up

The user confirmed hearing a tone consistent with 600 Hz on the successful
2 MS/s repeat (`tx-rate-2m-repeat.out`, 1,999,168.327 accepted samples/s).
They heard nothing on the next repeat (`tx-rate-2m-repeat2.out`): only
524,288 samples were accepted, followed by 29 zero-length write returns
and zero steady-state progress. This is an intermittent functional failure,
despite earlier successful reduced-rate operation.

The helper previously returned zero after cleanup even for stalled traffic.
It now returns 7 when steady acceptance is outside 90–110% of the requested
sample rate. This is a coarse smoke-test threshold, not RF validation.

A subsequent 4 MS/s / gap-0 comparison on the same candidate also stalled:
524,288 accepted samples, 29 zero returns, zero steady progress, exit 7.
Results are `tx-start-4m.out/.log`; `tx-start-4m-kernel.log` preserves the
kernel tail. Thus the failure is not restricted to nonzero sample gaps.
This does not rule out a shared FPGA or host-side startup problem.

Kernel diagnostics for the failed 2 MS/s run showed ACTIVE with SMIL=0
immediately after TX startup and repeated watchdog reports of that state.
The failed 4 MS/s run instead showed SMIL=0x3f just after startup and
0x24e at shutdown, but no steady host-data consumption. These observations
are not identical and do not yet establish the root cause.

The driver initializes its TX kfifo from six 512 KiB buffers; kfifo uses the
largest fitting power-of-two capacity, 2 MiB, matching 524,288 four-byte
samples. The plateau is therefore consistent with filling the host queue
without sustained DMA consumption, rather than evidence of transmitted data.

No driver or FPGA change was made in this follow-up. TX was stopped and
the helper restored 4 MS/s / gap 0. Next investigation should examine SMI/DMA
startup and callback progress, including the current ordering that issues
DMA before prefilling its TX buffers. A cause or correction is not yet proven.

## Startup-order experiment and reconfiguration recovery

`driver-start-order/` contains an isolated driver copy, build log and module.
It moves DMA preparation/issue after TX-buffer prefill, SMI FIFO clearing
and transfer-length setup; it also propagates programmed-transfer setup
failure and logs period count / TX queue occupancy on stop. It built against
the running 6.18.39+rpt-rpi-v8 headers. The installed module file was untouched.

Temporarily loading this module with parameters 6/2/3 did not recover TX:
`tx4m.out` reports the same 524,288-sample plateau, 29 zero returns and exit 7.
`kernel.log` records zero completed DMA periods and 2,097,152 queued TX bytes
at stop. Startup ordering is a real buffer-ownership concern, but this
experiment does not demonstrate it as the cause of this persistent stall.

The installed driver was restored with modprobe and the same parameters.
Then the exact two-LED FPGA binary was reprogrammed successfully
(`reprogram.log`), without HDL changes. The next 4 MS/s test passed:
3,998,331.284 accepted samples/s, no zero returns, exit zero
(`tx4m-reconfigured.out/.log`).

This narrows the investigation toward persistent FPGA/interface state or
reset/handshake interaction. It does not prove an FPGA-only cause: driver
reload and programming also affect the surrounding hardware state. No
startup-order patch has been promoted to the repository or installed.
The installed driver and experimental two-LED FPGA are currently loaded;
TX is stopped and the helper restored 4 MS/s / gap 0.

Next: inspect reset coverage and SMI request generation, then compare soft
reset with full reconfiguration if the stall recurs. Keep the previously
passing gap logic unchanged until the failure mechanism is better understood.

## Reset-clock defect reproduced in simulation

`top.v` holds `r_counter` at zero during reset; this also stops `w_clock_sys`.
`complex_fifo.v` resets pointers and flags synchronously, so the FIFO side
clocked by `w_clock_sys` receives no rising edges while reset is asserted.
The opposite, LVDS-clocked side can reset, leaving inconsistent pointers.
This affects the TX write side and RX read side. SMI request generation uses
FIFO full/empty flags, so stale FIFO state can affect the interface handshake.

`reset-study/tb_reset.v` uses the actual FIFO module with the top-level clock
divider behavior: after four writes, reset leaves the write pointer at 4
and the FIFO reports nonempty afterward (`result.txt`). The counterpart
`tb_reset_fixed.v` keeps the divider running: both pointers clear and the
FIFO is empty (`fixed-result.txt`). These are controlled reset simulations,
not proof that this defect caused the observed hardware stall.

`candidate-reset-clock/` copies the two-LED source and changes only the
divider behavior in `top.v`: initialize it to zero at configuration and
toggle it regardless of soft reset. Other logic in that block remains
gated by reset. This preserves clock edges for synchronous FIFO reset.
Both TX sequence/stop benches pass, and synthesis plus seed-16 PNR pass
with unchanged timing constraints (`synthesis.log`, `pnr.log`, `timing.json`).
The candidate has not been packed or programmed. No tracked HDL or installed
driver was changed. Hardware testing must explicitly exercise reset and
restart before claiming this resolves the intermittent failure.

### Reset-clock hardware checks

The candidate was subsequently packed and programmed successfully. Binary
SHA-256: `a90a908f0e4e57a7a2f5e26e97303a3f43cadcaeb5b1be39eca4a604e0e00bfe`.
The installed driver remains in use. `tx-reset-test.c` extends the standalone
test with an explicit FPGA soft reset and 10 ms settling interval after
driver initialization, before configuring the tone, modem rate and gap.
Each run transmits for approximately three seconds and then stops/restores
4 MS/s and gap zero. The SPI return-code workaround remains local to the test.

The 4 MS/s run measured 3,998,232.831 accepted samples/s, and the first
2 MS/s run measured 1,999,171.858. Both exited zero with no zero-length
write returns. Logs and outputs are `reset4m.*`, `reset2m.*` and
`reset2m-repeat.*` in `candidate-reset-clock/`.

These checks exercise explicit soft reset in addition to normal app startup.
They are short throughput smoke tests, not proof of long-term reliability,
RF waveform quality or normal-app restart behavior without an explicit reset.
The reset-clock candidate remains loaded; previous images remain available.
The user confirmed hearing all three tones clearly and at the same perceived
pitch. This supports correct audible tone behavior at both sample rates;
it is not an instrumented frequency or modulation-quality measurement.
The repeated 2 MS/s run measured 1,999,167.044 accepted samples/s, with no
zero-length write returns and exit zero.

### Normal FPGA API restored

The repository FPGA SPI wrapper now returns zero after a successful
`io_utils_spi_transmit`, matching that layer's status contract instead of
expecting a two-byte return value. Negative failures remain failures.
`test_fpga_spi_status.py` compiles the actual wrapper and gap API against
a simulated SPI register, checking changed/unchanged gaps, preserved upper
sync bits, readback and injected read/write failures. It passes, as does
the local CMake build and `git diff --check`.

`candidate-reset-clock/tx-api-test.c` removes the direct-register workaround
from the reset test and verifies restoration through the normal getter.
Its 2 MS/s hardware run read back modem SR=2 and FPGA gap=1, measured
1,999,100.350 accepted samples/s without zero returns, exited zero and
read back restored gap=0 (`api2m.out/.log`). It still performs an explicit
soft reset before setup; normal-app repeated startup remains a separate
validation step. The local build library/app were rebuilt; nothing was
installed system-wide or committed in this step.

The user subsequently completed their usual application test cycle and
reported TX and RX working normally. This adds user-confirmed normal-app
functional coverage to the API test above; the number of process restarts
and long-duration reliability were not established by that report.

## Option 11 selectable TX rate

The local test app now offers `2` (2 MS/s) and `4` (4 MS/s) inside option 11.
Rate changes are rejected while TX is running. A stopped rate change destroys
and recreates the pipeline so all workers use the same immutable frame size:
20,000 or 40,000 samples per 10 ms. The FM modulator uses the selected rate;
audio remains 48 kHz. Setup verifies modem rate and FPGA gap through the
normal APIs and displays the verified settings. Returning from option 11
restores its radio to 4 MS/s/gap 0. Other TX pipeline callers default to
4 MS/s; RX processing is unchanged.

Lifecycle tests now mock the rate APIs and check both RF rates, frame sizing
and modulator output length. Those checks and the local CMake build pass.
A live UI check selected 2 MS/s, started/stopped TX, rejected a rate change
while active, selected 4 MS/s and started/stopped again, then exited cleanly.
`option11-rate-test.log` records that run. Audible quality and repeated
user-driven restarts still require user confirmation.

To test: run `build/cariboulite_test_app 2> debug.log`, choose `11`, choose
`2`, and toggle TX with `1`. Stop TX before choosing `4` for comparison.
The currently loaded experimental FPGA supports gaps; option 3 still loads
the older saved image, so do not use it to prepare this experiment.

### Pitch correction and remaining startup failure

User testing found correct 4 MS/s audio, approximately 300 Hz at 2 MS/s,
and a failed subsequent start. `nbfm4m_pull_iq` still hard-coded the 250/3
audio-to-RF ratio for 4 MS/s despite accepting a configurable RF rate.
It now advances the resampling phase using the configured audio and RF
rates. `test_nbfm_rate.py` demodulates generated IQ and measures 599.59 Hz
at both 4 and 2 MS/s, checks all audio blocks are accepted, and passes.
The build and pipeline lifecycle tests also pass.

Two live option-11 starts at 2 MS/s were attempted after this correction.
Both UI starts returned ON, but both shutdown tone injections aborted;
the second start again showed ACTIVE with SMIL=0. Logs are
`option11-pitch-fixed.log` and `option11-pitch-fixed-kernel.log`.
These are not successful hardware validations of the pitch correction.
TX was stopped, the app exited, and option 11 restored 4 MS/s / gap 0.
The sample-rate-independent modulator defect is corrected; the intermittent
stream startup defect remains unresolved and no changes were committed.

### Option-11 clean-start correction

Before each option-11 TX start, the app now destroys/joins the previous
pipeline, soft-resets the FPGA, and recreates the pipeline with the selected
rate before activating TX. This clears stale FPGA FIFO state and DSP/audio
queues and prevents old workers from overlapping the reset. It relies on
the loaded reset-clock candidate for effective synchronous FIFO reset.
The monitor pipeline is unchanged. This is a controlled reset-based recovery
sequence, not a complete proof of the underlying stall mechanism.

After rebuilding and passing lifecycle tests, three consecutive 2 MS/s
starts and a 4 MS/s start through option 11 completed without start/tail
tone aborts. Kernel logs show continuing DMA callbacks. The 4 MS/s run
reported one missed-buffer event, so loss-free streaming is not established.
Logs: `option11-clean-restart.log`, `option11-clean-restart-kernel.log`.
TX was stopped and the app exited; 4 MS/s/gap 0 was restored by option 11.
Changes are uncommitted, pending user confirmation of pitch and repeated
start/stop behavior. Original FPGA images and installed driver remain intact.

The user subsequently confirmed successful TX start/stop and sample-rate
changes in their test cycle. This validates the option-11 clean-start and
rate-selection workflow on the currently loaded reset-clock FPGA candidate.
Changes remain uncommitted; this confirmation does not establish long-term
or loss-free streaming performance.

## Promotion of the tested app and FPGA

The user authorized committing both after confirming successful TX start/stop
and rate switching. The reset-clock candidate HDL, binary, routed ASC and
synthesized JSON were copied to `firmware/`; BLIF was exported from that same
JSON. Both embedded headers were regenerated from the tested binary and their
payloads verified byte-for-byte. Its SHA-256 remains
`a90a908f0e4e57a7a2f5e26e97303a3f43cadcaeb5b1be39eca4a604e0e00bfe`.
No synthesis or routing change was introduced during promotion.

The earlier warnings that option 3 loads the old image are historical:
**option 3 now loads the promoted image**, and the rebuilt local library
embeds it too. Both user LEDs default on. External PMOD RX markers are
removed; software markers remain. Loopback remains disconnected.

FPGA benches are preserved under `firmware/tests/`. The modulator pitch,
pipeline lifecycle, SPI regression and FPGA simulations passed, and the
local application/library rebuild passed. The installed kernel driver was
not changed. System-wide userspace libraries were not installed.

The previous committed firmware can be recovered from the parent of the
promotion commit; the ignored study rollback directory also retains the
original artifacts. Earlier candidate results and limitations above remain
part of the experiment record, not promises of exhaustive validation.
