# Read-only FPGA diagnostic verification

See the [implementation report](../../fpga-register-snapshot-2026-11-17.md) and
[command documentation](../../radio-control.md#explicit-read-only-fpga-register-snapshot).

- `pi4f-smoke.json`, `pi4g-smoke.json`: real get/snapshot sequence, board identities,
  binary/library hashes, unchanged ordinary replies and stable defined control bits.
- `pi4f-controller.txt`, `pi4g-controller.txt`: built-controller register tables.
- `smoke.py`: actual idle-only hardware check; acquires hardware through the backend,
  never starts RX/TX or writes LEDs/reset controls.
- `tests.txt`: regression results; `source.sha256`: source/test hashes.
- `release-check.json`: final process/SMI descriptor checks on both hosts.

The original TX reference remains pi4g; #4 is on pi4f. No RF measurements or
LED-toggle fault reproduction are claimed here. Masked/partial readbacks and
volatile status must be interpreted using the source register table and RTL.
