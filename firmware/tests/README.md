# FPGA simulation checks

Run from the repository root with Icarus Verilog installed:

```sh
iverilog -g2012 -s tb -o /tmp/tx-sequence firmware/tests/tb_sequence.v firmware/lvds_tx.v
vvp /tmp/tx-sequence
iverilog -g2012 -s tb -o /tmp/tx-stop firmware/tests/tb_stop.v firmware/lvds_tx.v
vvp /tmp/tx-stop
iverilog -g2012 -s tb -o /tmp/fifo-reset firmware/tests/tb_reset.v firmware/complex_fifo.v
vvp /tmp/fifo-reset
```

Sequence and stop tests use a registered-output FIFO model. The reset test
uses the real FIFO with a model of the free-running system-clock divider;
it does not instantiate the full top level. These simulations do not cover
physical LVDS timing or complete asynchronous-interface behavior.

See `docs/fpga-gap-study-2026-09-13.md` for build settings, hardware results,
limitations, and the previous firmware hash for rollback.

Build dependency/failure checks (no FPGA tools or hardware required):

```sh
python3 firmware/tests/test_build.py
```
