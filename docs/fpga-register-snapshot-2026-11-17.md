# Explicit FPGA register snapshots — 17 November 2026

Implemented and deployed on pi4f-trixie and pi4g-trixie without changing the FPGA
bitstream. The existing controller now accepts `fpga registers` while RX and TX
are stopped. It asks the owning backend to read a defined ten-register subset.
No snapshot reads are added to startup, ordinary get/status handling or polling.

## Behavior and scope

The new library API performs one SPI read per whitelisted address, with no
retries, writes, programming, soft/hard reset, status-recovery calls or cached
version/status changes. The backend holds its hardware lock around the sequence.
It rejects active RX/TX, transitions and FPGA_NEEDS_PROGRAM. A stopped pipeline
fault permits diagnosis without clearing that fault or restarting anything.
Configuration and pipeline generations do not change on a diagnostic read.

| Module | Readable items |
| --- | --- |
| SYS | Module/system/manufacturer versions; packed TX gap and sync-source bits |
| IO | Module version, frontend/debug mode, LED/BTN/CFG, internal RF control state |
| SMI | Module version; FIFO empty/full, channel and direction status |

The RTL implements these reads without clear-on-read behavior. Write-only debug
and soft-sync controls, the unimplemented SYS error-state read and unused PMOD
state are excluded. This is not a complete FPGA state dump or simultaneous sample.

The controller displays raw bytes, defined-bit masks and dynamic-bit masks.
Undefined IO.mode bits7..5 and IO.digital bit2 must not be compared. BTN/CFG inputs
and FIFO flags can change independently. RF state is an internal control mirror;
the forced top-level mixer pad need not equal its bit0 value. A validity bitmap
records SPI completion per register; failed entries display UNAVAILABLE and the
command returns HARDWARE with partial data. A transport-successful value still
needs comparison with the expected firmware and context.

Only explicit diagnostic replies append `fpga_regs MASK BYTE0 ... BYTE9` to the
normal v2 reply. Other wire replies, JSON status and FPGA LED command behavior
remain unchanged. The updated controller accepts the previous reply formats.
The new library type/API is separate; existing public library structures and
function behavior are unchanged. Makefile dependencies include the new shared
readback header so the controller and protocol tests cannot retain stale masks.

## Verification

- Read-only C mock: exactly ten expected read opcodes, no write opcodes; each
  possible SPI failure retains other valid data and leaves cached state unchanged.
- Protocol roundtrips/bounds and control-server checks pass.
- 15 FPGA-control, 16 RX-control, 18 TX-control and 37 controller tests pass.
  The live handler fault-injection fixture checks partial reads, state guards and
  unchanged generations; ordinary paths assert that they never request a snapshot.
- Idle hardware checks on both Pis compare get → snapshot → get → snapshot → get.
  Normal replies are unchanged, all ten reads succeed and defined non-dynamic
  control bits remain stable. The built controller prints the real register table.
  No LED, reset, programming, RX-start or TX-start commands were issued by the
  hardware checks. Existing backend initialization remains the normal setup path.

These checks support opt-in isolation and compatibility; they are not an
exhaustive RF/audio qualification of every application mode. Snapshot reads
necessarily generate SPI traffic when explicitly requested.

[Verification evidence](baselines/20261117-fpga-register-snapshot/README.md)
contains both-host command/reply records, controller output, source hashes and
release checks. Previous installed libraries/binaries are retained under
`build/fpga-register-snapshot-20261117/before/` on the corresponding host where
available. Both radios were released after verification. #4's LED1 remained0;
no before/after LED fault experiment was performed as part of this extension.
