# Board #4 before/after FPGA register tests

Read the [report](../../board4-register-comparison-2026-11-17.md) for results.

`experiment.py` is the actual hardware driver: it uses the backend control
socket, captures an initial snapshot, performs three repeated LED1-low/high
cycles while stopped, then checks RX at LED1=0/1/0. It preserves LED0 and restores
LED1=0 during cleanup. It never starts TX or issues reset/program commands.
`analyze.py` independently checks the completed data against the readable masks,
expected initialized-idle baseline and requested sequence.

- `comparison.json`: verified comparison and RX outcome summary.
- `snapshots.json`: all 27 register snapshots, including 20pure stopped comparisons.
- `events.json`: exact commands/replies; `rx-checks.json`: short RX observation windows.
- `metadata.json`: #4 identity, boot ID, binary/library hashes and backend arguments.
- `release-check.json`: final process/SMI descriptor checks on both hosts.
- `source.sha256`, `raw-artifacts.sha256`: script and raw archive hashes.

Full backend log and original JSON records remain in
`build/board4-register-comparison-20261117/raw.tar.gz` on pi4g. The run directory
is `20261117T202632Z`, also retained on pi4f under the same build folder.

Offline reproduction from the repository root:

```sh
python3 docs/baselines/20261117-board4-register-comparison/analyze.py \
  build/board4-register-comparison-20261117/20261117T202632Z \
  /tmp/board4-register-comparison-analysis
```

Readback validity means SPI completion, not automatic proof of correct internal
logic. Only currently exposed registers are observable. Masks and API semantics
are documented in the [snapshot implementation report](../../fpga-register-snapshot-2026-11-17.md).
The dynamic FIFO/input bits did not change in this run, but can change without
register writes. The comparison is not a simultaneous capture of internal state.
