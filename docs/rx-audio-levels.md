# Calibrated RX audio, live volume and level metering

The radio backend applies a shared playback stage to NBFM, WBFM mono and
AM voice. It runs after each mode's channel/audio filters and FM de-emphasis,
before signed-16-bit conversion. The existing standalone demodulator PCM API
and legacy application paths retain their former scaling.

## Calibration reference

At default volume (0 dB), a 1 kHz sine at **50% nominal peak modulation**
produces **−12 dBFS RMS**. Nominal peak modulation means:

| Mode | 100% nominal modulation |
| --- | --- |
| NBFM | Configured RX deviation, default ±2.5 kHz |
| WBFM mono | ±75 kHz deviation |
| AM voice | 100% envelope modulation, with carrier normalization enabled |

The audio-filter response at 1 kHz is compensated, including the selected
FM de-emphasis. Filters retain their intended frequency response. Calibration
is checked end to end using synthetic IQ at every native RF rate. This is a
digital reference, not an acoustic calibration of a speaker or a guarantee of
identical perceived loudness across programme material. Full nominal modulation
has approximately 3 dB peak headroom at 0 dB volume with unity trim.

AM's carrier estimator needs settling time. With `--rx-am-agc off`, its manual
IQ scaling remains available, but carrier-independent modulation calibration
no longer applies.

## Live volume

```text
rx volume -6
rx volume 0
rx volume 3
rx volume mute
```

Volume is absolute, in dB relative to the calibrated default. Allowed range is
−60 to +12 dB; `mute` selects zero output. Setting a numeric value unmutes.
Changes take effect while RX is running, using a 20 ms linear gain ramp. They
do not retune RF, restart the pipeline, or alter the saved TX settings. The
setting is retained across mode/rate changes, RX stop/start and TX/RX transitions
within the backend session. Reconnection of the controller does not reset it.

Use `--rx-volume -6` (or `--rx-volume mute`) to set the startup volume. The
existing `--rx-gain` is retained as a linear trim: 8000 is unity; 4000 reduces
playback by about 6 dB. Its historical direct PCM scaling is replaced in the
backend by calibrated scaling followed by this trim. Prefer the dB volume
control for normal operation. Speaker/ALSA volume remains a separate control.

The v2 request is `v2 ID rx_volume DB`, where −120 is the mute sentinel. Control
reply layouts are unchanged. Volume acknowledgements appear in the controller;
the current saved setting and meter values are published in passive status.

## Level meter

The passive monitor shows RX volume, mute, RMS and peak in dBFS, and the
cumulative number of clipped samples. JSON exposes `rx.audio_output`:

- `volume_db`, `muted`, `calibrated` and calibration reference metadata.
- `valid`, `rms_dbfs`, `peak_dbfs`, `window_ms` (100).
- `clipped_samples`, reset on worker reset or a new pipeline.

RMS/peak describe generated PCM after volume, squelch ramp and hard clipping,
before the playback queue and ALSA mixer. A closed gate reads silence after its
ramp. Silence has a −120 dBFS display floor. Measurements remain unavailable
before a complete window, while stopped or transitioning, and in transport-only
dry runs. A disconnected speaker is not diagnosed by these digital levels.

There is no automatic audio leveling, compression or soft limiter. Positive
volume or extra trim can exceed available headroom: PCM is safely hard-clipped
and the clipping counter exposes that. Reduce volume/trim if clipping occurs.
The audio floating-point tap avoids clipping in the legacy demodulator PCM
conversion before the shared output stage gets control.

Raw NBFM squelch and SINAD taps bypass the playback stage. Changing volume or
muting does not alter those measurements or improve RF signal-to-noise ratio.

## Validation

`tests/test_rx_audio_output.py` runs independent synthetic IQ measurements with
strict and production floating-point builds. It covers all three modes and
eight rates, 0/50/75 µs de-emphasis, configured NBFM deviation, the reference
RMS/peak, floating-point tap headroom, meter values, ramping, mute, gate closure,
invalid volume and clipping counts. Protocol/controller/backend tests exercise
live volume without pipeline generation changes and truthful dry-run metering.

The first live check on pi4f-trixie used AM at 119.050 MHz and 400 kS/s.
Changing volume from −12 to −6 dB increased median background-audio RMS by
6.17 dB; the changing RF signal was not a calibrated source. Mute reached the
silence floor, volume edits retained the same pipeline generation, and no
clipped samples, queue drops, alignment failures, read/write errors or underruns
were observed during the check. TX stayed off. Playback was left at −6 dB.
[Recorded software and live results](baselines/20261115-rx-audio-output/).
