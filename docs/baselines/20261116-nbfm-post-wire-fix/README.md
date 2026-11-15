# NBFM sweep after TX wire corrections

See [the report](../../nbfm-post-wire-fix-sweep-2026-11-16.md) for interpretation.
`results.csv` and `summary.json` retain all 240 original points, including the
isolated low-power SINAD dip. `comparison.csv` matches each point with the
historical `20261114-native-power-sweep` by axis, RX rate, TX rate and requested
power, and verifies matching PA codes. Deltas are current minus historical.

The supplementary `recheck-results.json` contains one TX-off baseline and
three repeats of the affected TX 1 MS/s, RX 4 MS/s, -12 dBm condition. These do
not replace or enter the original 240-point sweep/comparison statistics.

Full raw snapshots, helper/backend logs, command replies and role-ready
metadata are retained in the ignored local artifact directory:

```
build/radio-backend/implementation-20261116/
  cariboulite-nbfm-retest-20261115T231139Z/
  cariboulite-nbfm-recheck-20261115T234659Z/
  nbfm-retest-rx.tar.gz
  nbfm-retest-tx.tar.gz
  nbfm-recheck-rx.tar.gz
  nbfm-recheck-tx.tar.gz
```

The UTC run IDs began on 15 November; the client date in Europe/Brussels was
16 November. `raw-artifacts.sha256` records hashes of both role archives.
No private speech recordings are included; the test used a generated tone.

## Reproduce analysis

Python 3 and Matplotlib are required for figures. From the repository root:

```sh
BASE=docs/baselines/20261116-nbfm-post-wire-fix
RAW=build/radio-backend/implementation-20261116/cariboulite-nbfm-retest-20261115T231139Z
python3 "$BASE/analyze_native_sweep.py" "$RAW" /tmp/nbfm-analysis
python3 "$BASE/verify_native_sweep_raw.py" "$RAW" /tmp/nbfm-analysis
python3 "$BASE/verify_settings.py" "$RAW"
python3 "$BASE/compare.py" /tmp/nbfm-analysis/summary.json \
  docs/baselines/20261114-native-power-sweep/summary.json /tmp/nbfm-analysis
python3 "$BASE/write_report.py" /tmp/nbfm-analysis /tmp/nbfm-report.md
```

The analyzer reads `source.sha256` and `release-check.json` beside its script.
The report's baseline links are relative to its final location under `docs/`.
For the four-point supplementary run, pass `4` as the third argument to
`verify_native_sweep_raw.py`. `verify_settings.py` accepts either raw directory.
Both checks reconcile every saved snapshot rather than just plot summaries.
The primary run has 5,767 retained snapshots; the recheck has 97.

`estimator-provenance.json` verifies the historical/current estimator header
is byte-identical. `source.sha256` records selected current production sources
and the backend binary. `metadata.json` records active native profiles, exact
observed rates, backend hashes, setup and driver versions. `release-check.json`
records final checks after the supplementary run: no backend/SMI owners and
all playback PCMs closed on both hosts.

## Collection scripts

`native_power_sweep.py` is the actual collector run on pi4f-trixie as root,
using `/tmp/cariboulite-diag-radio-helper.py` copied from `radio_helper.py`.
`recheck.py` uses the same helper for the targeted follow-up. These are active
hardware tools: they start RF/playback and use SSH to pi4g-trixie. Use only
with exclusive access and the stated wired, attenuated setup. Host-role guards
keep pi4g as TX and pi4f as RX. The helpers stop their backends on normal exit,
exceptions and input EOF. No production software or defaults changed.

Two axes cover eight RX rates with TX fixed at 4 MS/s, then eight TX rates with
RX fixed at 4 MS/s. Each block has TX off and fourteen requested powers from
-12 to +14 dBm. The tone is 1 kHz at amplitude 0.4, deviation setting 2500 Hz;
settling is two seconds and measurement five seconds. All fresh snapshots are
retained, without filtering on SINAD. The collector checks cumulative counters
and aborts on transport/state faults. SINAD changes alone are recorded.

`source.sha256` is not an assertion that only those files changed since the
historical run. The comparison includes the entire intervening implementation
and hardware state. Raw RSSI/power are uncalibrated, repeated snapshots are not
independent, and the fixed-notch estimator can be limited by clock offsets.
