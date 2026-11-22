# CTCSS transmit tone and receive tone squelch

The standalone radio backend supports independent TX and RX CTCSS settings.
Both default to **off** and apply only in **NBFM**. Saved settings survive mode
changes in a backend session; AM, USB, LSB and WBFM RX bypass CTCSS. Frequencies
can be selected anywhere from 67.0 to 254.1 Hz, including the usual 50 tones.
The [Kenwood TS-990S manual](https://www.kenwood.com/usa/Support/pdf/TS-990s.pdf)
provides a reference table of those standard tone frequencies.

## Launch options

Append, for example, these options to your usual backend command:

```sh
--tx-ctcss 88.5 --tx-ctcss-deviation 500 --rx-ctcss 88.5
```

For repeater access, TX tone alone is often sufficient. Enable RX tone squelch
only when the station or repeater you want to hear transmits that tone. TX and
RX frequencies can differ. Use `--tx-ctcss off` / `--rx-ctcss off` to bypass.
Your RX gain profile and mode trims still apply normally.

`--tx-ctcss-deviation` specifies **peak RF deviation in Hz**, not an audio
percentage. Its default is 500 Hz; the allowed range is 50–1000 Hz. It must be
strictly smaller than `--tx-deviation` whenever TX CTCSS is selected. This is a
configurable implementation default, not a calibration for every repeater.

## Controller

Stop the relevant direction and wait for transitions to finish before changing
its settings:

```text
rx stop
rx ctcss 88.5
rx start

tx stop
tx ctcss 88.5
tx ctcss deviation 500
tx start
```

Use `rx ctcss off` or `tx ctcss off` to disable independently. An RX tone change
while RX is running is rejected; TX tone/deviation edits are rejected during
TX or its transitions. Changes take effect on the next start. Controller
commands use Hz directly, unlike frequency tuning commands which use MHz.

The monitor displays selected frequencies, mode-dependent enable/bypass,
TX deviation, and RX detection/coherence. JSON adds `config.ctcss`, with
`rx_frequency_hz`, `tx_frequency_hz`, `tx_deviation_hz`, `rx_enabled`,
`tx_enabled`, `valid`, `detected` and `coherence`. Frequency zero means off.
`detected` is null until an enabled NBFM receiver has a complete measurement;
coherence is a 0–1 tone-energy fraction, not received RF strength. Existing
v1/v2 reply layouts are unchanged. New v2 requests are `rx_ctcss HZ`,
`tx_ctcss HZ` (zero disables) and `tx_ctcss_deviation HZ`; readback is in JSON.

## Signal path and timing

TX high-passes voice at 300 Hz when CTCSS is selected, then adds a continuous
sine **after** voice processing and pre-emphasis. The microphone compressor
never processes the sub-tone. The tone is also present when the normal audio
source is the synthesized test tone. Voice deviation is clipped to the total
configured NBFM deviation minus the selected tone deviation before summing.
With the defaults, voice has a 2000 Hz peak budget and CTCSS has 500 Hz, giving
a nominal total of ±2500 Hz. IQ quantization and hardware characteristics
still require physical modulation/spectrum measurements.

With CTCSS enabled, TX sends 600 ms of tone-only lead-in instead of the usual
100 ms silent-audio lead-in. The opening Quindar cue remains 250 ms, followed
by 50 ms of tone-only audio before speech. On stop, CTCSS drops before the
existing 50 ms pre-cue interval, 250 ms stop cue and finite silent padding.
There is no reverse-phase burst. Repeated PTT resets the CTCSS oscillator and
voice-filter state. With CTCSS disabled the existing start timing is retained.

RX detects from the 48 kHz raw discriminator tap, before de-emphasis, volume,
mode trim and audio gating. Four low-pass poles at 300 Hz precede decimation
to 1 kHz, with DC rejection and a 400 ms quadrature correlation window updated
every 100 ms. Two qualified windows open the tone gate; two misses close it.
Acquisition is about 500 ms for a stable tone, with up to approximately 600 ms
release including window history. This favors adjacent-tone rejection over
fast acquisition. Qualification requires a tone-energy fraction of 0.70 to
open / 0.50 to remain open, plus 0.008 normalized peak amplitude after detector
filtering; sensitivity therefore varies with tone frequency and reception.

The effective audio gate requires the tone **and** every enabled noise/carrier
squelch detector. Disabling noise and carrier squelch does not bypass CTCSS.
Detection continues while audio is muted. A selected-tone notch (Q=8) and
300 Hz fourth-order voice high-pass remove the sub-tone from playback; a
5 ms gate ramp retains the existing click suppression. Playback filtering is
bypassed when RX CTCSS is off or the mode is not NBFM.

## Validation

Run the hardware-free checks:

```sh
python3 software/libcariboulite/tests/test_ctcss.py
python3 software/libcariboulite/tests/test_squelch.py
python3 software/libcariboulite/tests/test_tx_worker_parking.py
```

After building the Linux backend/controller, also run:

```sh
python3 software/libcariboulite/tests/test_ctcss_backend.py
```

The DSP checks cover all 50 standard tones, nearest adjacent-tone rejection,
±0.5 Hz source offset, speech-band interference, DC offset, noise, silence,
release/reset, invalid samples, playback tone suppression, and bypass. They
also exercise the actual modulator-to-demodulator raw tap at all eight RF
rates, tone deviation and the total modulation budget. DSP checks run under
strict and production math flags. RX worker checks cover tone-only squelch,
combined carrier/noise/tone gating, continued audio queue progress while muted,
and bypass in other modes. Mocked TX workers check retained starts and original
Quindar frame counts with CTCSS enabled. Backend tests cover launch options,
controller commands, JSON/monitor readback, mode bypass and running-state locks.

On 2026-11-22, the strict and production DSP checks passed locally and natively
on pi0b. The RX worker and mocked TX lifecycle checks passed in an isolated
pi0a build, along with 6 compressor, 19 TX-control, 37 controller and 28 backend
regression tests. The usual application paths on pi0a and pi0b were then rebuilt
with matching backend/controller hashes; all 3 CTCSS backend tests passed on
both hosts. Previous binaries are retained in `build/radio-backend/pre-ctcss/`.
Running radio sessions were not restarted.

Physical link validation was reported by the user on 2026-11-22 between the
two Pi radios through 90 dB attenuation: CTCSS operation succeeded with the
matching tone, and RX stayed muted with a wrong tone. This provides physical
end-to-end evidence for TX encoding and selective RX tone squelch on that link.
The report did not specify tone frequencies, RF frequency, sample rate, TX
power, test direction or duration; no tone-absent result was reported.

External radio/repeater interoperability, broader weak-signal/false-opening
performance, measured RF deviation/spectrum and end-to-end tail-muting timing
remain open. No automated test keys real RF hardware.

## SINAD comparison and tone-level diagnosis

An offline comparison on 2026-11-22 exercised the actual NBFM modulator and
demodulator at 1 MS/s, with a continuous 1 kHz test tone, 2500 Hz total deviation,
CTCSS off / 250 / 500 Hz deviation, and 88.5 / 254.1 Hz CTCSS frequencies.
Each case ran for three seconds; the existing streaming SINAD estimator supplied
the readings after settling. This used software loopback, with no RF hardware,
added channel noise, oscillator mismatch, ALSA or speaker stages.

The existing live SINAD metric uses the raw discriminator tap before the RX
CTCSS-removal filter. Its second-order 300 Hz high-pass does not eliminate
CTCSS, so the residual sub-tone can materially lower the reading without
establishing intermodulation. The diagnostic also applies an identical
selected-tone notch and fourth-order 300 Hz high-pass to every comparison case,
including CTCSS-off, before running a separate copy of the same SINAD estimator.
This isolates the sub-tone contribution; it is not a calibrated speaker-output
SINAD measurement and does not model the playback de-emphasis or PCM stage.

For 88.5 Hz CTCSS and 0.4 test-tone amplitude (nominal 1000 Hz voice deviation):

| CTCSS deviation | Existing raw-tap SINAD | SINAD after identical CTCSS-removal filtering |
| --- | ---: | ---: |
| Off | 68.6 dB | 68.6 dB |
| 250 Hz | 33.2 dB | 67.7 dB |
| 500 Hz | 27.1 dB | 67.7 dB |

At 254.1 Hz CTCSS, the corresponding raw readings were 68.6 / 16.7 / 11.0 dB,
while filtered readings remained 68.6 / 67.7 / 67.6 dB. Thus a large raw-meter
change alone is insufficient evidence of intermodulation.

At 0.95 test-tone amplitude (nominal 2375 Hz voice deviation), the tone exceeds
the available voice budgets with CTCSS enabled: 2250 Hz with a 250 Hz sub-tone,
or 2000 Hz with a 500 Hz sub-tone. Filtered SINAD was approximately 68.1 dB off,
40.3 dB at 250 Hz, and 27.4 dB at 500 Hz. The limiter clips the voice/test tone
before adding CTCSS, producing genuine harmonic distortion. The CTCSS sine is
added after that limiter. These results identify a software clipping mechanism;
they do not establish the cause of the user's perceived distortion on RF.

The regression covers both the meter bias at normal level and clipping at high
level under strict and production math:

```sh
python3 software/libcariboulite/tests/test_ctcss_sinad.py
```

For a physical comparison, use a 1 kHz test tone (`tx tone 1000 0.4`), fixed RF
level/attenuation and receiver settings, and compare CTCSS off / 250 / 500 Hz.
The normal synthesized tone defaults to 600 Hz, which is unsuitable for the
fixed-1-kHz SINAD notch. Measure consistently filtered recovered audio and
inspect the spectrum for harmonics and sidebands at 1000 ± CTCSS frequency.
The current live raw-tap SINAD display alone does not provide that corrected
comparison. Repeat at higher amplitude separately to expose limiter clipping.

### User-reported physical SINAD comparison

The user subsequently reported a two-radio comparison through 90 dB attenuation,
with 0 dBm TX power, a 1 kHz test tone at 0.4 amplitude, and 200 Hz CTCSS
deviation at 88.5 Hz. Reported SINAD was 19.0 dB with CTCSS off and 18.9 dB
with CTCSS on: nearly unchanged at the reported precision. Repeatability and
measurement uncertainty were not specified, so this does not establish a
quantitative upper bound on added distortion.
