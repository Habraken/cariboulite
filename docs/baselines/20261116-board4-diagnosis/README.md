# Board #4 startup/LED recovery investigation

Read the [report](../../board4-startup-diagnosis-2026-11-16.md) for sequence,
results and limits. Five short RX probes span two boots: three fail startup,
two collect TX-off/on points after test-app LED toggles. Soft reset and full
programming trials also failed RX before the first recovery.

`probe-results.json` retains the five probe summaries/failures; test-app logs
retain digital readbacks. `reset-trial.json` retains reset/program commands,
replies and statuses. `startup-check.json` is the first idle initialization.
`first-recovery/` and `second-recovery/` contain independent raw/statistics and
settings checks. `release-check.json` confirms no remaining radio processes
or open SMI descriptors on either host. Sources and archives are hashed.

Full raw logs/snapshots and transmitter-side logs remain in the three archives
under `build/board4-diagnosis-20261116/` on pi4g. Extracted successful run IDs:

- `cariboulite-board4-diagnosis-20261116T190415Z`
- `cariboulite-board4-diagnosis-20261116T190727Z`

Offline checks use the existing board4 verification scripts, e.g.:

```sh
RAW=build/board4-diagnosis-20261116/cariboulite-board4-diagnosis-20261116T190727Z
python3 docs/baselines/20261116-board4-rx/verify_native_sweep_raw.py "$RAW" /tmp/board4-recovery-check 2
python3 docs/baselines/20261116-board4-rx/verify_settings.py "$RAW"
```

The collector/helper acquire hardware and transmit briefly on the original
pi4g reference. #4 receives only; UUID guards identify each role. No full sweep,
permanent workaround, hardware change or EEPROM rewrite was performed.
