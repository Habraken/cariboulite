# Board #4 EEPROM and RX evidence

Receiver on pi4f: `c2440a8d-3852-4c44-b96f-1b64a962e7dc`.
Original TX reference on pi4g: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`.

EEPROM programming and reboot recognition passed. RX startup failed twice at
4 MS/s and once at 2 MS/s with SMI synchronization errors. The other six rates
started successfully, but the subsequent power sweep encountered invalid SMI
frames at 800 kS/s. The original run was preserved. Invalid frames also occurred at 400 kS/s. A continuation imported the
first two complete rate blocks, repeated 800 kS/s and measured the remaining
rates, retaining counter failures rather than aborting. RX qualification failed. No TX test was performed on #4. Its oscillator frequency
was not measured and the cause of the startup failures is not established.

The nominal 90 dB wired link was inherited from the preceding board setup;
it was not independently measured or reconfirmed for this swap.

## Evidence

- `eeprom-preflight.json`: two matching full blank reads.
- `manifest.json`, `programming-receipt.json`: unique identity, image hashes,
  verified pages, header-last ordering and repeated final readback.
- `board4-boot-identity.json`: boot HAT identity and software provenance;
  captured during active RX collection, so open radio descriptors are expected.
- `startup-check.jsonl`: all eight native-rate startup outcomes, with statuses.
- `summary.json`, `results.csv`: 90 measured points, six off and 84 transmitting.
- `raw-verification.json`, `settings-verification.jsonl`: independent statistical
  reconciliation, rates/mode/tone/filter/counter and role/identity checks.
- `release-check.json`: post-test radio release and driver provenance.
- `comparison*.csv`: matching-rate/power comparisons against #1, #2 and #3.

Full EEPROM evidence and raw runs remain in `build/board4-20261116/` on pi4g;
EEPROM backups are also retained on pi4f. Both failed sweep attempts and per-rate
startup probes are preserved separately from the supported-rate sweep.
See `raw-artifacts.sha256` for archive hashes.

## Reproduction

The collector and helper run on pi4f and use SSH to the original pi4g reference.
They acquire radio hardware and start RF transmission on pi4g. The UUID/host
guards prevent using another board, and #4 receives only. The continuation collector imports 30 completed points from the prior run,
records the two failed startup rates and retains all nonzero counters. It uses two-second settling and five-second measurement
windows as the preceding tests. Production DSP, driver and FPGA were unchanged.

Offline analysis from the repository root:

```sh
BASE=docs/baselines/20261116-board4-rx
RAW=build/board4-20261116/cariboulite-board4-rx-20261116T173153Z
python3 "$BASE/verify_native_sweep_raw.py" "$RAW" /tmp/board4-analysis 90
python3 "$BASE/verify_settings.py" "$RAW"
python3 "$BASE/analyze_board4.py" "$RAW" /tmp/board4-analysis
python3 "$BASE/write_report.py" /tmp/board4-analysis /tmp/board4-report.md
```

The analyzer reads `source.sha256` and `release-check.json` beside its source
and prior board summaries in neighboring baseline directories. Raw RSSI and
fixed-notch SINAD are uncalibrated; repeated status estimates are not independent
samples. Plot bands are observed min/max, not uncertainty bounds. These tests
do not qualify TX, the oscillator, calibrated sensitivity or emissions.
