# Board #4 full post-reboot LED1-low RX sweep

Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f.
Original pi4g transmitter: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`.
The user requested reboot, LED1=0, and the full RX-rate/TX-power sweep, with the
90dB link confirmed. This is a fresh run, not a continuation or imported result.

The helper preserves LED0, explicitly reads/writes the digital register through
the backend and verifies LED1=0 before each RX-rate start. Each RX helper owns
its backend; no test app, EEPROM programming or automatic production workaround
is involved. `metadata.json`, ready records and command logs retain LED readback,
boot IDs and source/binary/library provenance. `led-verification.json` checks
all eight startup sequences independently.

The sweep has eight native RX rates, TX fixed4MS/s, RF24 430.100MHz, a1kHz
NBFM tone, amplitude0.4 and deviation setting2500Hz. Each rate has TX off and
fourteen requested powers from-12to+14dBm in2dB steps, with2seconds settling and
5seconds measurement:120points. +10/+12/+14 requests repeat PA31. Results are
uncalibrated link RSSI and fixed-notch SINAD estimates; repeated status values
are not independent samples. The oscillator frequency was not measured.

## Retained evidence

- `boot-identity.json`, `setup.json`: post-reboot receiver and user-confirmed setup.
- `summary.json`, `results.csv`: all point statistics.
- `raw-verification.json`: independent reconciliation with every snapshot.
- `settings-verification.jsonl`: rates, filters, mode/tone, roles and counters.
- `led-verification.json`: checkedLED1=0 and preservedLED0 before all RX starts.
- `release-check.json`: both backends exited and SMI descriptors released.
- `comparison*.csv`, `rssi.svg`, `sinad.svg`: matched-setting comparisons with
  earlier receivers; these do not isolate causal receiver-quality differences.
- `source.sha256`, `raw-artifacts.sha256`: source and raw archive hashes.

Raw role/ready metadata, command/status logs and snapshots from both hosts are
archived under `build/board4-led1-low-20261116/` on pi4g, with successful run ID
`cariboulite-board4-led1-low-rx-20261116T193415Z`. The LED command additions and
retry fix changed the backend/library since earlier baseline sweeps; they did
not change the DSP profile, driver or FPGA firmware for this run. Earlier
LED1-high failures and the test-app/LED experiments remain separate evidence.

Offline reproduction from repository root:

```sh
BASE=docs/baselines/20261116-board4-led1-low-rx
RAW=build/board4-led1-low-20261116/cariboulite-board4-led1-low-rx-20261116T193415Z
python3 "$BASE/verify_native_sweep_raw.py" "$RAW" /tmp/board4-led1-low-analysis 120
python3 "$BASE/verify_settings.py" "$RAW"
python3 "$BASE/verify_led_setup.py" "$RAW"
PYTHONPATH=/tmp/cariboulite-plot-tools python3 "$BASE/analyze_retest.py" "$RAW" /tmp/board4-led1-low-analysis
```

Matplotlib is required for plotting; the host's existing isolated copy is under
`/tmp/cariboulite-plot-tools`. Analyzer provenance files are read beside its source;
previous receiver summaries are read from neighboring baseline folders. Collector
scripts acquire hardware and transmit on pi4g; they are not needed for inspection.
This is conditional RX qualification, not an explanation of the LED1 dependency,
oscillator/TX acceptance, calibrated sensitivity or indefinite continuity.
