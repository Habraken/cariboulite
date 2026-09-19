# Physical audio/RF baseline runner

Run after building the app with the install script. This is a new repeatable
procedure for the updated Raspberry Pi OS; its first successful run was
confirmed by Jan on 2026-09-19. Earlier H0 acceptance remains a separate recorded result.

## Recorded setup

- 50 ohm dummy load connected to the **HiF** antenna port.
- Jabra GN 510 USB for audio capture and playback.
- TX monitoring: SDRPlay RSP2PRO, 2 MS/s, tuned to 430.1 MHz, NFM,
  10 kHz audio bandwidth, squelch −120 dBm.
- App sessions: HiF/RF24, 430.1 MHz, requested TX power −3 dBm,
  48 kHz audio, 2.5 kHz FM deviation. Physical RF coupling to the SDRPlay
  has not been specified; retain the same arrangement for comparisons.

## Run

From the repository root after rebuilding:

```sh
python3 software/libcariboulite/tools/physical_baseline.py --dry-run
python3 software/libcariboulite/tools/physical_baseline.py
```

Use the same hardware permissions as the normal app. If it requires root, run
the second command with `sudo`. The script does not install, rebuild, or change
system audio configuration. By default it uses the existing Jabra audio bridges:

```text
Jabra microphone -> arecord -> Loopback playback 0,1 -> app capture 1,1
app playback 0,0 -> Loopback capture 1,0 -> aplay -> Jabra speaker
```

Keep those bridges running. The runner opens `plughw:Loopback,1,1` for capture
and `plughw:Loopback,0,0` for playback, matching the interactive app.
Use `--audio-route direct` only when the Jabra is not owned by those bridges or
another audio process. Direct mode discovers the Jabra by ALSA card ID.
To override either route explicitly:

```sh
python3 software/libcariboulite/tools/physical_baseline.py \
  --capture 'plughw:CARD=YOUR_CARD_ID,DEV=0' \
  --playback 'plughw:CARD=YOUR_CARD_ID,DEV=0' \
  --seconds 10
```

Find card IDs with `arecord -l` and `aplay -l`. Check speaker/microphone mute and
volume before starting. Sessions last 10 seconds by default, with two-second
separations; `--seconds` accepts 2–300. `--firmware` overrides `firmware/top.bin`;
`--app` overrides `build/cariboulite_test_app`.

The runner launches the app once, performs FPGA hard reset, programs the chosen
firmware and performs the post-programming soft reset. It then runs:

| Order | Menu pipeline | Direction | RF rate | Listen for |
| --- | --- | --- | --- | --- |
| 1 | 11 | TX | 2 MS/s | 600 Hz tone; Quindar start/end |
| 2 | 11 | TX | 4 MS/s | Same 600 Hz pitch; Quindar start/end |
| 3 | 12 | RX | 2 MS/s | Receiver noise from Jabra |
| 4 | 12 | RX | 4 MS/s | Receiver noise from Jabra |
| 5 | 14 | TX | 2 MS/s | Quindar start/end; Jabra microphone audio |
| 6 | 14 | TX | 4 MS/s | Quindar start/end; Jabra microphone audio |
| 7 | 14 | RX | 2 MS/s | Receiver noise from Jabra |
| 8 | 14 | RX | 4 MS/s | Receiver noise from Jabra |

All pipelines are stopped/destroyed before rate changes. A soft reset between
sessions clears shared FPGA FIFO state. Normal TX startup/shutdown supplies
2525 Hz / 2475 Hz Quindar tones through existing pipeline code.

## What this tests

The app's `--baseline-test` mode calls the same TX/RX pipelines as options 11,
12 and 14; option 14 initializes both pipelines through its existing helper.
It deliberately bypasses menu input and curses rendering. It does not test UI
key handling, live retuning, or the monitor's concurrent register display.
Options 11 and 12 normally select `radio_low`; this runner explicitly selects
`radio_high` to match the HiF dummy load. Interactive menu defaults are unchanged.
All sessions use the selected audio routes; the default loopback routes match
the menu and reach the Jabra through the existing bridges. These differences are explicit so results are not mistaken for tests of
unchanged interactive menu defaults.

The runner checks initialization/start results and RF FIFO consumption progress.
It records valid RSSI readings once per second for each RX session; an invalid
reading fails the run rather than reporting a cached value as a fresh sample.
RSSI is the modem-reported measurement on the HiF path, not a calibrated
connector power measurement. Existing pipeline code does not propagate every
hardware operation's return code; automation completion is not proof of RF output
or audible success. No known RF receive signal is supplied by this dummy-load test.

## Results and confirmation

Each run creates a timestamped directory under `build/physical-baseline/`:

- `metadata.json`: source revision/status, app/firmware SHA-256, OS/kernel/Pi,
  ALSA device listings, exact command and declared external setup.
- `source.diff`: tracked source changes relative to HEAD (untracked files are
  listed in metadata but not included; commit the runner before establishing a
  long-term reference).
- `app.log`: app diagnostics and baseline events.
- `events.csv`: ordered session events and individual RX RSSI samples.
- `summary.json`: per-session RSSI count/minimum/mean/maximum and automation result.
- `listening-checklist.md`: audible checks awaiting Jan's confirmation.

Copy accepted result directories outside `build/` before deleting it for a clean
rebuild, or use `--output /path/to/persistent/results`.

Report whether both option 11 tones had the correct 600 Hz pitch, all four TX
sessions had both Quindar tones, and all four RX sessions produced noise. Identify
any failure by option and rate. Automation never marks the listening checks passed.
Ctrl-C requests cleanup. The supervisor enforces an overall deadline and, if
cleanup hangs, escalates termination and records that the radio state must be
checked before retrying. On errors it stops the sequence and retains logs.

## Software-only checks

```sh
python3 software/libcariboulite/tests/test_physical_baseline.py
python3 software/libcariboulite/tests/test_rx_lifecycle.py
python3 software/libcariboulite/tests/test_tx_stop_deadline.py
python3 software/libcariboulite/tests/test_nbfm_rate.py
```

The reporting tests use a fake app and do not validate physical reset/programming,
RF, or audio. The hardware run and listening confirmation remain pending.

## First-run failure and correction

The run `20260919T073841.346127Z` completed both option 11 TX sessions, then
failed before starting option 12 RX: `Device or resource busy` opening
`plughw:CARD=USB,DEV=0`. Existing `arecord` and `aplay` bridges already owned
the Jabra capture and playback devices. The runner now defaults to their
Loopback endpoints instead of opening the Jabra exclusively. No bridge process
was stopped and no system audio configuration was changed. The subsequent physical retest passed (see below). This correction only changed
the Python runner, so no rebuild was needed.

## Accepted run — 2026-09-19

Run `20260919T074311.996531Z` completed all eight sessions with exit code 0.
Jan confirmed: “Perfect test. I heard the tones at the correct pitch, as well as
 the receiver noise.” This is a functional listening confirmation, not an
instrumented pitch measurement.

| RX pipeline | Rate | RSSI min / mean / max (dBm) | Samples |
| --- | --- | --- | --- |
| 12 | 2 MS/s | −107 / −104.6 / −103 | 10 |
| 12 | 4 MS/s | −107 / −103.7 / −102 | 10 |
| 14 | 2 MS/s | −110 / −107.0 / −105 | 10 |
| 14 | 4 MS/s | −108 / −105.0 / −102 | 10 |

[Archived metadata](baselines/20260919T074311.996531Z/metadata.json),
[results and confirmation](baselines/20260919T074311.996531Z/summary.json), and
[individual events/RSSI](baselines/20260919T074311.996531Z/events.csv) are preserved
outside the build directory. The tested checkout was `9927d03` plus the
uncommitted runner changes; binary and firmware hashes are in the metadata.
This supplements the earlier H0 with the updated OS and repeatable test setup.

## Step 1 retest — 2026-09-19

Run `20260919T075122.967948Z` completed all eight sessions with exit code 0 after
the ALSA source rename. Jan confirmed: “Test running completed successfully,
all the tones were noticed at the correct pitch.” Each RX session recorded ten
RSSI samples. Receiver-noise listening was not separately stated in this retest.

[Metadata](baselines/20260919T075122.967948Z/metadata.json),
[results](baselines/20260919T075122.967948Z/summary.json), and
[events/RSSI](baselines/20260919T075122.967948Z/events.csv) are archived outside
`build/`. The metadata identifies the tested revision plus uncommitted changes
and the binary/firmware hashes.
