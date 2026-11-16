# Board #4 retest RX sweep

Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f.
Original TX reference: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` on pi4g.

This is a fresh eight-rate sweep with no imported measurements or EEPROM rewrite.
The earlier failed runs remain in `../20261116-board4-rx/` and their archives.
`setup.json` records the user-reported manual test-app session and the reboot
context. The retest follows manual test-app use; the reboot sweep follows an
explicit pi4f reboot, with no test-app/LED initialization afterward.

The collector checks board UUID/host roles, uses pi4g for TX and pi4f for RX,
records startup failures and retains nonzero counters as failures. It runs
120 points: TX off plus fourteen power requests at each native RX rate. TX is
fixed at 4MS/s, 430.1MHz, 1kHz NBFM tone, amplitude0.4, deviation setting2500Hz.
Each point has2seconds settling and5seconds measurement. The nominal90dB link
was inherited and not independently measured. Production software was unchanged.

## Evidence and offline reproduction

Raw role metadata, command/status logs and snapshots are archived under
`build/board4-retest-20261116/`. Transmitter-side logs are included in the archive.
`raw-artifacts.sha256` identifies the archive; `source.sha256` records analysis
and collector scripts. `release-check.json` verifies radio release after the run.
`raw-verification.json` reconciles every snapshot with its summary;
`settings-verification.jsonl` checks mode, tone, filters, rates, counters and roles.

```sh
BASE=docs/baselines/20261116-board4-rx-retest
RAW=build/board4-retest-20261116/cariboulite-board4-rx-retest-20261116T175734Z
python3 "$BASE/verify_native_sweep_raw.py" "$RAW" /tmp/board4-retest-analysis 120
python3 "$BASE/verify_settings.py" "$RAW"
PYTHONPATH=/tmp/cariboulite-plot-tools python3 "$BASE/analyze_retest.py" "$RAW" /tmp/board4-retest-analysis
```

Matplotlib is required for plots; this host's existing isolated copy is at
`/tmp/cariboulite-plot-tools`. The analyzer reads source/release evidence beside
its script and earlier board summaries in neighboring baseline directories.
RSSI/SINAD are uncalibrated link estimates; bands show observed min/max rather
than uncertainty. This is scoped RX evidence, not oscillator/TX qualification
or proof of indefinite continuity.
