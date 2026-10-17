# CE2 serial candidate programming and smoke test

The reserved-pad cleanup candidate was loaded into the FPGA on `pi4f-trixie`
on 2026-10-17 at 15:24:34–15:24:36 UTC. Programming and basic SPI communication
passed. The user subsequently reported successful smoke tests in both
applications on the same date. Full A5 acceptance, including physical pad
measurements and other-seed qualification, remains pending.

## Exact artifacts

The image comes from the three matching, passing CE2-serial seed-16 builds in
the [cleanup qualification](fpga-warning-cleanup-2026-10-17.md).

- [Synthesized BLIF](../build/fpga-pa-control/warning-cleanup-20261017/runs/ce2-serial-s16-r1/work/top.blif)
- [Packed programming image](../build/fpga-pa-control/warning-cleanup-20261017/runs/ce2-serial-s16-r1/work/top.bin)

Programming uses `top.bin`, not the BLIF netlist. The 32,220-byte image SHA-256 is
`78f650fdf192f8a3c11fc45a253d08b90071e78671d59100ab9de7b5b38ff7c2`.
The saved rollback image remains
`build/fpga-pa-control/a1-20261017/reference/rollback/firmware/top.bin`, SHA-256
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.

## Programming and checks

No radio applications were running. The loader claimed `/dev/smi` exclusively
before GPIO/SPI access, initialized the FPGA/programmer context, and configured
from the verified candidate file. Setup held the modem and mixer in reset;
no radio initialization, RX/TX pipeline or RF test was launched.

The configuration log records reset/CDONE preparation, all 32,220 bytes sent,
and successful completion. After a soft reset and settling interval, the loader
restored SMI GPIO functions and checked:

| Check | Result |
| --- | --- |
| CDONE | High |
| System version | 1 |
| Manufacturer ID | 1 |
| SYS / IO / SMI module versions | 1 / 1 / 1 |
| Frontend mode write/readback | Debug 0, low-power mode 0 |

Each identity read checks the SPI transfer return code and expected value.
These shared version values establish communication, not a unique image ID.
The SHA-256 identifies the file sent; no bitstream readback was performed.
The loader would restore the saved image on candidate failure; rollback was
not needed. I/O resources and the ownership claim were released afterward.

The existing standalone programmer omits FPGA-context initialization and
ignores errors, so this session used a checked loader under the ignored
`build/fpga-pa-control/physical-smoke-20261017/` directory. That directory retains
`load_candidate.c`, its executable, `programming.log` and `programming.json`,
including command, image/library/loader hashes and exit status. The production
programmer source was not changed.

## Application smoke test

On 2026-10-17, following candidate programming, the user reported:

> smoke test for both applications completed. All seems to be working.

Record the application smoke test as **passed for both applications** with this
CE2 serial candidate. The report does not enumerate individual modes, channels
or reset sequences; those details and instrumented pad measurements remain part
of the broader qualification record to complete before final A5 acceptance.

## Reloading and remaining qualification

Normal test-app and backend startup use `force_fpga_reprogramming=false`; they
can retain the operational image already loaded. Avoid menu option 3 or the
backend's `fpga program` command from the repository root during candidate
testing: those explicitly reload the unchanged `firmware/top.bin`. Forced
embedded-firmware programming also selects the older packaged image. FPGA
configuration reset, reboot or power loss requires loading the candidate again.

The full acceptance checklist covers NBFM TX/RX on RF09 and RF24 in both
applications, WBFM RX on a suitable path, repeated start/stop/direction changes
and appropriate reset/reload sequences, with the external PA disconnected.
Record detailed coverage alongside physical reserved-pad measurements and
other-seed build qualification when completing acceptance.

All six packaged production FPGA artifact hashes remained unchanged after this
programming session. No packaged binary, firmware header or boot image was
replaced; only the FPGA's volatile configuration was loaded.
