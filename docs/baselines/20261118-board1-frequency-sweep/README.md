# Board #1: repeat of board #4 frequency sweep — 18 November 2026

[Full report and board comparison](../../board1-frequency-sweep-2026-11-18.md).

Board #1 UUID `d8810044-c444-46f4-a548-703412cbf971` replaces board #4 on
pi4f-trixie. The original pi4g reference remains TX through the existing nominal
90 dB coax link. The backend and shared-library hashes on both hosts were
checked against board #4's ready records and match. Kernel/driver versions and
radio roles are unchanged. No EEPROM rewrite or FPGA firmware change is part
of this test.

The 172-frequency `plan.json` is byte-identical to
[board #4's plan](../20261118-board4-frequency-sweep/plan.json). RF24, both sample
rates 1 MS/s, requested TX power 0 dBm (PA code 12), NBFM 1 kHz tone at amplitude
0.4, deviation setting 2500 Hz, two seconds settling and five seconds measurement.
Software LED1 is held low with readback before every RX start, preserving LED0,
to match board #4's conditions. This does not imply board #1 requires that
workaround. TX is stopped before each RX retune; both roles retune while stopped.
The upper tuning boundary remains exclusive, so the endpoint is 5,999.999 MHz.

The main sweep retains the same default noise-squelch configuration. The 21
original board #4 open-squelch follow-up points are listed in
`board4-retry-points.json` and are also repeated here. Any further board #1
points with closed-gate observations or unavailable estimates are included in
`retry-points.json`. Both squelches are disabled in these follow-ups. SINAD runs
**before** the audio gate and suppresses estimates failing its 1 dB tone
criterion; absent values are not replaced with numeric zeros.

Both complete raw runs are retained. `merge_runs.py` selects the follow-up
measurements at repeated points and leaves the main run unchanged elsewhere;
`source-selection.json` identifies each final point's origin. `analyze.py`
independently reconciles every selected snapshot with the summary, verifies
settings/filters and band coverage, and generates plots/CSV. `verify_roles.py`
checks identities, LED writes, and absence of RX-side TX commands/states.
`compare_board4.py` verifies matched frequencies, plans, software and settings
before generating paired differences and overlaid plots, including a 2–3 GHz
zoom. No timing comparisons are made between different hosts' monotonic clocks.

The collectors acquire hardware and cause TX; they are not needed for offline
inspection. After locating the archived raw run directories:

```sh
MPLCONFIGDIR=/tmp/cariboulite-matplotlib \
PYTHONPATH=/tmp/cariboulite-plot-tools python3 \
  docs/baselines/20261118-board1-frequency-sweep/analyze.py \
  /path/to/selected-data /tmp/board1-frequency-analysis
```

Use `--no-plots` for verification without Matplotlib. The full raw records are
archived under `build/board1-frequency-sweep-20261118/`; the report identifies
run IDs and results. Source and archive hashes preserve provenance.

The measurements are uncalibrated end-to-end link results. Actual connector
power, attenuation versus frequency, oscillator frequencies and external RF
interference were not independently controlled/measured. Similarity across
boards cannot by itself locate a fault in the receiver versus the common TX,
coax path or measurement method. Previous board #4 data remains separate.

Band labels are laboratory coverage references with a Region 1 emphasis and
explicit Region 2 supplements, not national operating permissions. References:
[ITU ISM designations](https://www.itu.int/net/ITU-R/terrestrial/faq/index.html),
[RSGB amateur band plans](https://rsgb.services/public/bandplans/).
