# Board #1 frequency sweep with laptop distant — 18 November 2026

This is a separate repeat of the
[nearby-laptop board #1 experiment](../20261118-board1-frequency-sweep/README.md).
The user reported moving the laptop to a large distance before authorizing the
run; distance and laptop emissions were not measured. Previous data is retained.

Receiver: board #1 UUID `d8810044-c444-46f4-a548-703412cbf971` on pi4f-trixie.
TX: original `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` on pi4g-trixie. The existing
nominal 90 dB coax link, RF24, 0 dBm requested TX (PA code 12), 1 MS/s on both
radios, NBFM 1 kHz tone/amplitude 0.4/deviation setting 2500 Hz, two-second
settling and five-second measurement windows match the preceding experiment.
Backend/library hashes, board identities and boot IDs were checked against the
previous ready records before measurement. Software LED1 is set low with
readback before every RX start while preserving LED0. Neither radio retunes
while active; TX stops before RX retuning.

`plan.json` is byte-identical to the nearby-laptop plan: 172 points, 50 MHz base
grid plus amateur/ISM anchors, European SRD supplements and RF path boundaries.
The upper software limit is exclusive; the endpoint is 5,999.999 MHz. The 21
original follow-up points in `nearby-retry-points.json` are repeated with both
squelches disabled. Additional points needing a diagnostic follow-up are listed
in `repeat-selection.json`. SINAD is measured before the audio gate; unavailable
results failing the 1 dB tone criterion are never replaced with zero.

`merge_runs.py` selects follow-up measurements at repeated frequencies;
`source-selection.json` identifies every final row's origin. Both original raw
runs remain unchanged. `analyze.py` independently reconciles statistics,
settings, observed filters, gate conditions, freshness and band coverage.
`verify_roles.py` checks identities, LED writes and no receiver-side TX activity.
`compare_nearby.py` verifies the paired settings and generates CSV differences
and overlaid linear/logarithmic plots, including the 2–3 GHz detail. `finalize.py`
assembles the offline record and report. Collectors/helpers acquire hardware and
transmit, so they are not used for offline analysis.

[Report and overlays](../../board1-laptop-distant-frequency-sweep-2026-11-18.md).
Raw records are archived under
`build/board1-laptop-distant-frequency-sweep-20261118/`; source and archive hashes
identify the retained evidence. To reanalyze a selected raw directory:

```sh
MPLCONFIGDIR=/tmp/cariboulite-matplotlib \
PYTHONPATH=/tmp/cariboulite-plot-tools python3 \
  docs/baselines/20261118-board1-laptop-distant-frequency-sweep/analyze.py \
  /path/to/selected-data /tmp/board1-distant-analysis
```

Use `--no-plots` to verify without Matplotlib. Measurements are uncalibrated
end-to-end results. RF output power, attenuation versus frequency, external RF
sources and thermal drift were not independently measured. This time-separated
comparison tests whether features persist after moving the laptop; it does not
by itself establish or exclude a physical source of interference.
