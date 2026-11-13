# Recorded native-rate power sweeps

`results.csv` and `summary.json` contain all 240 measured points. `metadata.json`
records setup, exact native profiles, backend hashes and installed driver
images. Plot bands are observed minima/maxima, not uncertainty intervals.
`raw-verification.json` reconciles all 5760 snapshots with the summaries.
`release-check.jsonl` records both radios idle, with no SMI owner and closed
playback devices.

Full inputs, including raw snapshots, role-ready metadata, command logs and
backend logs, are retained in the ignored build artifact directory:

`build/radio-backend/implementation-20261113/cariboulite-native-power-sweep-20261113T231340Z/`

Separate receiver and transmitter tar archives also remain in its parent.
Original temporary logs remain on each radio under `/tmp` with that run name.

To regenerate analysis with Python 3 and Matplotlib, from the repository root:

```sh
python3 docs/baselines/20261114-native-power-sweep/analyze_native_sweep.py \
  build/radio-backend/implementation-20261113/cariboulite-native-power-sweep-20261113T231340Z \
  /tmp/native-sweep-analysis
python3 docs/baselines/20261114-native-power-sweep/verify_native_sweep_raw.py \
  build/radio-backend/implementation-20261113/cariboulite-native-power-sweep-20261113T231340Z \
  /tmp/native-sweep-analysis
```

`native_power_sweep.py` is the actual collector, run on pi4f-trixie with the
helper at `/tmp/cariboulite-diag-radio-helper.py`; it uses SSH to pi4g-trixie.
It requires the stated wired/attenuated setup and exclusive radio access.
It starts/stops RF and playback; it is not an offline analysis script.
`source.sha256` records the deployed source check made before collection.
