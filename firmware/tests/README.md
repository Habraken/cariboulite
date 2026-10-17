# FPGA simulation checks

Run from the repository root with Icarus Verilog installed:

```sh
iverilog -g2012 -s tb -o /tmp/tx-sequence firmware/tests/tb_sequence.v firmware/lvds_tx.v
vvp /tmp/tx-sequence
iverilog -g2012 -s tb -o /tmp/tx-stop firmware/tests/tb_stop.v firmware/lvds_tx.v
vvp /tmp/tx-stop
iverilog -g2012 -s tb -o /tmp/fifo-reset firmware/tests/tb_reset.v firmware/complex_fifo.v
vvp /tmp/fifo-reset
iverilog -g2012 -s tb -o /tmp/fpga-loopback firmware/tests/tb_loopback.v firmware/sys_ctrl.v firmware/lvds_tx.v
vvp /tmp/fpga-loopback
```

Sequence and stop tests use a registered-output FIFO model. The reset test
uses the real FIFO with a model of the free-running system-clock divider;
it does not instantiate the full top level. These simulations do not cover
physical LVDS timing or complete asynchronous-interface behavior.

See the [FPGA study](../../docs/fpga-gap-study-2026-09-13.md) for historical
build settings, hardware results, limitations and rollback hashes. The current
baseline and image hash are recorded in [firmware/README.md](../README.md).

Build dependency/failure checks (no FPGA tools or hardware required):

```sh
python3 firmware/tests/test_build.py
```

The isolated experiment runner has separate checks using temporary repositories
and fake synthesis, placement/routing and packing tools:

```sh
python3 firmware/tests/test_experiment_runner.py
```

These exercise frozen source/image/header and host-file preservation, full hash
verification, the twelve-command seed-16 plan, isolated stage ordering, fixed
default/CE2 and parallel/serial options, timing rejection, missing outputs,
failures/timeouts, retained logs, tool/reference tampering, protected source
paths, and refusal to overwrite existing output. The generated matrix-script
test also verifies that a failed run does not prevent later results from being
recorded. These tests do not run real FPGA builds or access hardware. See the
[runner guide](../tools/README.md) for A1 preparation, A2 execution and rollback
requirements.

The loopback bench connects the actual register controller and LVDS transmitter.
It checks register decoding, 1,024 gap/phase/TX/FIFO combinations, exact debug
frames, no settled-loopback FIFO consumption, exit/resumption and reset.
It does not instantiate SPI pins, physical DDR cells or the modem.

The driver also checks its actual zero-IQ padding word against the unchanged
SMI collector, including the expansion caused by legacy raw-zero padding:

```sh
python3 driver/tests/test_tx_quiet_framing.py
```

This check lives with the [driver tests](../../driver/README.md) because
hardware-paced TX changed driver startup/refill behavior without changing RTL.
RX decoder equivalence and physical DDR capture remain outside these benches;
the [LVDS timing plan](../../docs/lvds-fmax-next-test-2026-10-08.md) specifies
additional checks before any future receive-RTL optimization.
