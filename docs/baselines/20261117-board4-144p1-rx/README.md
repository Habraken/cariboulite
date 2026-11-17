# Board #4 RX qualification at 144.1 MHz — 17 November 2026

Receiver #4 UUID `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f-trixie.
Original TX reference UUID `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` on pi4g-trixie.
The user confirmed the hardware setup is unchanged, including the nominal 90 dB coax link, and requested the same full
RX-rate/TX-power sweep at 144.1 MHz. LED0 is preserved; LED1 is explicitly set
and verified low before each RX rate. No reboot or test-app initialization was
performed for this run. No EEPROM change or TX trial on #4 was performed.

See the additional 144.1 MHz section of the
[board #4 qualification report](../../board4-led1-low-rx-qualification-2026-11-16.md).
The 430.1 MHz results remain separate and unchanged.

## Measurements and evidence

The collector/helper are frequency-only variations of the previous LED1-low
sweep. TX remains fixed at 4 MS/s, RF24 144.100 MHz, 1 kHz NBFM tone, amplitude
0.4 and deviation setting 2500 Hz. Each of eight native RX rates has TX off and
fourteen power requests from -12 to +14 dBm in 2 dB steps, with two seconds
settling and five seconds measurement: 120 points. These are requested powers;
+10/+12/+14 repeat PA code 31, not distinct measured RF output powers.

- `summary.json`, `results.csv`: retained point statistics, requested/read-back
  settings and transport counters.
- `metadata.json`, `setup.json`: roles, frequency, profiles and software provenance.
- `raw-verification.json`, `settings-verification.jsonl`: independent snapshot
  reconciliation, actual settings, rate/filter observations and counters.
- `led-verification.json`: explicit LED1-low setup before all RX-rate starts.
- `release-check.json`: radio process/SMI descriptor release on both hosts.
- `rssi.svg`, `sinad.svg`: per-rate curves, observed min/max bands and differences
  against the preceding 430.1 MHz sweep on the same receiver.
- `comparison-430p1.csv`, `comparison-summary.json`: matched requested settings
  across frequencies, with PA codes retained separately.
- `source.sha256`, `raw-artifacts.sha256`: source and archived raw-evidence hashes.

Full status/command/backend logs, helper-ready records and raw snapshots from
both hosts are archived on pi4g under `build/board4-144p1-20261117/`; run ID
`cariboulite-board4-144p1-rx-20261117T192345Z`. Transient IPC sockets are excluded.
The collectors acquire hardware and transmit on pi4g; they are not needed for
inspection or offline analysis.

## Offline reproduction

From the repository root:

```sh
BASE=docs/baselines/20261117-board4-144p1-rx
RAW=build/board4-144p1-20261117/cariboulite-board4-144p1-rx-20261117T192345Z
python3 "$BASE/verify_native_sweep_raw.py" "$RAW" /tmp/board4-144p1-analysis 120
python3 "$BASE/verify_settings.py" "$RAW"
python3 "$BASE/verify_led_setup.py" "$RAW"
PYTHONPATH=/tmp/cariboulite-plot-tools python3 "$BASE/analyze.py" "$RAW" /tmp/board4-144p1-analysis
```

Matplotlib is required for plots. This host's existing isolated installation is
under `/tmp/cariboulite-plot-tools`. The analyzer reads source/release provenance
beside its source and the committed 430.1 MHz summary in the neighboring folder.
`write_section.py` renders a report fragment for insertion into the existing
qualification report, using links relative to `docs/`.

RSSI and fixed-notch SINAD are uncalibrated link measurements. The nominal link
attenuation was not independently calibrated at either frequency; changes in
TX/mixer output, receiver frontend, attenuators and cables can contribute to
cross-frequency differences. These data do not isolate receiver sensitivity,
resolve the LED1 dependence, qualify TX on #4 or measure its oscillator frequency.
