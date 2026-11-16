# Board #2 EEPROM and RX-only evidence

Receiver #2 UUID: `36b67ac4-b313-4369-99bd-4b3e09caabfa` (pi4f).
Previous receiver #1: `d8810044-c444-46f4-a548-703412cbf971`.
Original TX reference: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` (pi4g).

The user confirmed #2 installation and the nominal 90 dB direct link. Board #2
has the user-reported incorrect U25 oscillator; its actual frequency was not
measured and no TX trials were performed on it. Read the
[report](../../board2-rx-qualification-2026-11-16.md) for measurements and limits.

## Identity evidence

- `eeprom-preflight.json`: two complete identical blank reads and hash.
- `manifest.json`: unique image identity, atom/overlay validation and tool output.
- `programming-receipt.json`: blank backup hash, verified page offsets,
  header-last write and matching complete final-readback hash.
- `board2-boot-identity.json`: UUID/product/variant and SMI presence after reboot.
- `setup.json`: board roles, restrictions, implementation provenance and the
  initial launch's SSH failure before collection.

Full EEPROM backup/images/settings/decoded readback remain in
`build/board2-20261116/eeprom/` on pi4f and in
`build/board2-20261116/board2-eeprom-evidence.tar.gz` on pi4g. The prior verified
image preparation/writer scripts remain on pi4f under
`build/eeprom-init-20261106/`; the local inspected copies are in
`/tmp/next-board-rx/`. The blank-only writer refuses to overwrite a nonblank
EEPROM. UUIDs identify boards, not installations or individual test runs.

## RX collection and verification

`rx_power_sweep.py` and `radio_helper.py` are the actual collectors. These start
RF and audio, require exclusive hardware access, and run on pi4f using SSH to
pi4g. The helper checks the receiver UUID and each role's hostname before
starting the backend. There is no TX-rate sweep on #2. Commands sent to the
receiver are checked to contain no TX-start operation, and its saved states
are checked for absence of TX operation.

`summary.json` and `results.csv` retain all 120 points; `comparison.csv` matches
the RX axis of the 16 November #1 sweep by RX rate, TX rate/power and PA code.
Eight rates span 400 kS/s through 4 MS/s. TX stays at 4 MS/s with a 1 kHz tone,
amplitude 0.4 and deviation setting 2500 Hz. Each rate has TX off and -12 to
+14 dBm requests in 2 dB steps; +10/+12/+14 repeat PA code 31. Settling is two
seconds and measurement five seconds. Fresh RX snapshots are retained at 5 Hz;
500 ms SINAD values may repeat and are not independent observations. Plot
bands show observed min/max, not uncertainty.

Raw snapshots, role-ready metadata, backend logs and command logs are archived
under `build/board2-20261116/` on pi4g; see `raw-artifacts.sha256` for archive
hashes. The extracted successful run directory is
`cariboulite-board2-rx-20261116T155843Z/`. The failed launch
`cariboulite-board2-rx-20261116T155759Z/` is retained separately: reboot cleared
a temporary SSH host-key file, restored using pi4g's existing public host keys.
No board/RF failure was inferred from that SSH error.

To reproduce the analysis with Python 3 and Matplotlib from the repository:

```sh
BASE=docs/baselines/20261116-board2-rx
RAW=build/board2-20261116/cariboulite-board2-rx-20261116T155843Z
python3 "$BASE/verify_native_sweep_raw.py" "$RAW" /tmp/board2-analysis 120
python3 "$BASE/verify_settings.py" "$RAW"
python3 "$BASE/analyze_board2.py" "$RAW" /tmp/board2-analysis
python3 "$BASE/write_report.py" /tmp/board2-analysis /tmp/board2-report.md
```

The analyzer reads source/release evidence beside its script and #1's committed
summary from the neighboring baseline directory. The report links are relative
to its final `docs/` location. No installation of tools or script execution is
required to inspect the CSV/JSON records. Verification reconciles every retained
snapshot and checks the actual mode/tone/filter/rate readbacks and counters.

Raw RSSI is uncalibrated and has rate-dependent noise floors. Fixed-notch SINAD
can differ with clock/frequency offsets; stronger readings do not isolate an
intrinsic hardware improvement. Native rate readbacks are register evidence,
not physical clock metrology. This records scoped RX operation; it does not
qualify TX, the incorrect oscillator, sensitivity/emissions or indefinite uptime.
