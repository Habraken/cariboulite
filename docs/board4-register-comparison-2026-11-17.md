# Board #4 LED1/register comparison — 17 November 2026

The intended before/after tests are complete. **Only the intended LED1 bit changed
in the exposed, defined register bits.** No unintended change was found in the
readable synchronization configuration, frontend/debug mode, RF control state or
stopped SMI status. Brief RX checks still reproduced success → failure → success
with LED1=0 → 1 → 0. The underlying cause remains unresolved.

Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc`, pi4f-trixie. Pi4g remained
idle. No test app, reboot, reset or programming command was used; the startup
log confirms the FPGA was already operational and was not reprogrammed.

## Stopped-state register comparison

The backend was initialized normally and kept in IDLE for every register
snapshot. This is the backend-initialized baseline, not a capture before driver
initialization. Initial versions/manufacturer values and idle controls matched
the current RTL and backend setup.

| Register | Initial / LED1=0 | LED1=1 | Defined-bit change |
| --- | ---: | ---: | --- |
| SYS.module_version | 0x01 | 0x01 | None |
| SYS.system_version | 0x01 | 0x01 | None |
| SYS.manufacturer | 0x01 | 0x01 | None |
| SYS.gap_sync_source | 0x00 | 0x00 | None |
| IO.module_version | 0x01 | 0x01 | None |
| IO.mode | 0x00 | 0x00 | None |
| IO.digital | 0xF9 | 0xFB | **Bit1 only: LED1** |
| IO.rf_state | 0x56 | 0x56 | None |
| SMI.module_version | 0x01 | 0x01 | None |
| SMI.fifo_channel_direction | 0x14 | 0x14 | None |

LED0 was preserved at 1. Three low/high cycles were performed, with three snapshots
after every state write. Together with initial and restored-low snapshots, this
provides 20 stopped-state snapshots. The sequence was not interleaved with RX
operations, so RX start/stop could not confound that comparison. All ten reads
succeeded in every snapshot. The only observed defined-bit difference was
IO.digital XOR 0x02. Undefined bits were excluded, and physical-input/FIFO flags
were classified as dynamic; no other defined bits changed, including those flags.

## RX correlation after the register-only comparison

Using the same backend process, the receiver was checked at4 MS/s, RF24
430.100 MHz. Registers were read before and after each trial while stopped.
No TX was started; these tests check sample acquisition, not RF RSSI/SINAD.

| LED1 | Start command | Observed state | Sample acquisition |
| ---: | --- | --- | --- |
| 0 | OK | RX | Active; over 11 million new emitted samples during observation |
| 1 | ERROR HARDWARE | FAULT | No emitted samples during observation; 15 alignment failures accumulated during startup |
| 0 | OK | RX | Active again; over 12 million new emitted samples during observation |

Observation windows were three seconds. Counters are cumulative across trials;
the15failures from the high-state trial remain visible after recovery. Neither
successful observation window added alignment failures. No physical sample-clock
frequency measurement is inferred from these sample counts.

## What this establishes

The measurements do not support a persistent unintended change in the exposed
control registers as the explanation for the LED1-dependent RX failure. They do
not rule out unexposed/write-only state, transient behavior between reads,
internal FPGA timing/logic effects, electrical loading/coupling or an assembly
fault. No cause or permanent workaround is established by this comparison.

The complete run contains 27 snapshots and 270 successful register reads, including
the additional pre/post-RX and cleanup snapshots. Commands, replies, raw readbacks
and observation snapshots are retained. Independent analysis checks the register
masks, expected baseline, repeated LED states and RX outcomes.

LED0 was left 1, LED1 was restored to 0, RX was stopped and the backend exited.
Final process and SMI descriptor checks confirm both radios released.

[Raw evidence, analysis and reproduction](baselines/20261117-board4-register-comparison/README.md)
