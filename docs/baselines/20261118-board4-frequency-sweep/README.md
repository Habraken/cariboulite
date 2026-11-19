# Board #4 frequency sweep — 18 November 2026

Original pi4g transmitter → existing nominal 90 dB coax link → board #4 on
pi4f, both RF24. Both sample rates 1 MS/s; TX request 0 dBm. NBFM source:
1 kHz tone, amplitude 0.4, deviation setting 2500 Hz. LED0 is preserved and
software LED1 explicitly set low with checked readback before every RX start.

See [report](../../board4-frequency-sweep-2026-11-18.md) for final results.

`plan.json` contains the 50 MHz base grid, band anchors, RF path transition
anchors and coverage ranges: 172 frequencies. At least three are planned in
each listed amateur/ISM range. Region 1 is the main context; 222/915 MHz
Region 2 ranges and European 863–870 MHz SRD points are supplementary. SRD is
not labeled as an ITU ISM allocation. Band labels organize laboratory coverage;
they are not a statement of national operating authorization.

The software's 6 GHz upper boundary is exclusive. The last requested frequency
is 5,999.999 MHz; 6,000 MHz itself was not measured. Both radio roles retune
while stopped. TX is stopped before changing RX frequency. Each point has two
seconds settling and five seconds measurement, with full fresh snapshots
retained. Freshness uses each host's own sequence and monotonic clock; no
comparison of monotonic times across different Pis is made.

`frequency_sweep.py` and `radio_helper.py` acquire hardware and cause TX on the
original board; do not run them for offline inspection. The helper is copied to
`/tmp/board4-frequency-helper.py` on both hosts before collection. The runner
executes on pi4g as pi and uses sudo for device access and SSH to pi4f.
`make_plan.py` regenerates the acquisition plan. `analyze.py` independently
checks raw snapshots, settings, statistics and band coverage and generates CSV
and publication-style static plots.

Offline reproduction, after locating the raw run directory recorded in the
report and metadata:

```sh
PYTHONPATH=/tmp/cariboulite-plot-tools python3 \
  docs/baselines/20261118-board4-frequency-sweep/analyze.py \
  /path/to/raw-run /tmp/board4-frequency-analysis
```

Use `--no-plots` to run verification without Matplotlib. Raw backend/status/
command logs and paired measurement snapshots are archived under the local
`build/board4-frequency-sweep-20261118/` directory, outside Git. The committed
summaries, independent verification, settings/provenance and plots provide the
reviewable measurement record. Raw archive and source hashes identify the
retained inputs.

RSSI and SINAD describe the uncalibrated end-to-end link. Neither actual TX RF
output nor attenuation versus frequency was measured. Mixer/front-end gain,
coax/attenuator response, transmitter distortion, noise and tone/clock offsets
can all affect the curves. No TX qualification of board #4 or FPGA modification
is part of this experiment.

Band references:

- [ITU ISM frequency designations](https://www.itu.int/net/ITU-R/terrestrial/faq/index.html)
- [RSGB amateur band plans](https://rsgb.services/public/bandplans/)

## Audio-gate follow-up

The original run used the existing noise-squelch setting. Frequencies with any
closed gate or unavailable SINAD are selected into `retry-points.json` and
repeated with `repeat_open_squelch.py` / `radio_helper_open.py`, which disable
both squelches explicitly. `merge_runs.py` retains original open-gate points
and replaces only selected points with complete open-squelch measurements.
`source-selection.json` records every final point's origin. Both complete raw
runs are archived unchanged; original gated/missing values are not erased.
`verify_roles.py` checks both runs' identities, LED writes and absence of RX-side
TX commands.

The open-squelch follow-up established that suppressed SINAD readings persist
with the gate open. Code inspection confirms SINAD is measured **before** the
gate, and requires a >1 dB total/residual ratio plus non-silent finite audio.
Noise-squelch closure and SINAD suppression are associated indicators of poor
received tone quality here, not a causal chain. Unavailable estimates remain
null; their presence is a result, not an unmeasured frequency or invented zero.
