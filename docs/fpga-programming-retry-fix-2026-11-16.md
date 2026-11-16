# FPGA programming retry error propagation — 16 November 2026

Fixed and installed on pi4f-trixie and pi4g-trixie. Embedded-bitstream programming
now returns success only after operational FPGA status is observed, and returns
failure after all three unsuccessful attempts. The prior post-decrement loop
left its retry counter at -1 after exhaustion, bypassing the zero check. It also
incorrectly rejected successful programming on the third attempt.

The existing programming/reset regression test now checks configuration failure
exhaustion, configuration success without operational FPGA communication, forced
reprogramming failures and success on the third attempt. It fails against the old
source and passes with the fix. FPGA SPI tests and all eleven backend FPGA-control
tests pass. Both hosts rebuilt the library, test app and backend; the shared
runtime was installed at /usr/local/lib/libcariboulite.so. Matching build and
installed library SHA256 on both hosts:
`9142777cced313c299936dd97126790e4472d1f883810b0f472bee0012899e0d`.

Hardware startup verification, without starting RX or TX:

- pi4f, board #5: three attempts fail operational verification; the backend logs
  “FPGA not operational after 3 attempts” and “Radio initialization failed”,
  exits with status1 and never reaches RX idle.
- pi4g, original reference: initialization reaches RX idle and exits normally
  with status0 after the timed check.

The lower-level “FPGA programming - Success!” log describes completion of the
configuration transfer/CDONE check. It is followed by the operational check;
its wording was not changed by this fix. The test app's explicit file-based
programming path uses a separate final status check and did not have this retry
counter bug. This change fixes error reporting, not #5's underlying hardware or
FPGA communication failure. Diagnosis of #4 remains deferred.

Both radios were released afterward. [Verification evidence](baselines/20261116-fpga-retry-fix/)
contains source hashes, regression result, both-host startup results and release
checks. Prior installed libraries are backed up under
`build/fpga-retry-fix-20261116/before/` on each Pi; pi4f also retains its prior
build library and original source there.
