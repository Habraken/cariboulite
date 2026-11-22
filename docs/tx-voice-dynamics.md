# TX microphone compressor and limiter

The backend has an optional broadband microphone compressor followed by a
lookahead peak limiter. It reduces speech level differences before AM, NBFM,
USB or LSB modulation. Processing is **on by default for microphone TX in all
four voice modes**. Explicit bypass preserves the existing audio samples and
introduces no delay. Synthesized test tones,
Quindar start/stop cues and stop-padding silence bypass the processor even
when it is enabled. RX is unaffected.

## Starting the backend

Build with the usual command:

```sh
make -C software/libcariboulite/tools/radio_backend
```

For a Jabra microphone/speaker, launch with the voice defaults (the explicit
`--tx-compressor on` below is optional):

```sh
./build/radio-backend/cariboulite_radio_backend \
  --rx-playback plughw:CARD=USB,DEV=0 \
  --tx-microphone plughw:CARD=USB,DEV=0 \
  --tx-source mic --tx-compressor on \
  --tx-comp-threshold -24 --tx-comp-ratio 3 \
  --tx-comp-attack 5 --tx-comp-release 200 \
  --tx-comp-makeup 6 --tx-comp-ceiling -1
```

For the Logitech H650e, use `plughw:CARD=H650e,DEV=0` in both device options.
The backend starts with TX stopped. Use the controller's normal TX start/stop
commands. These settings are initial tuning values, not verified intelligibility
improvements or an aviation/tactical-radio compliance profile.

## Parameters

| Launch option | Controller parameter | Default | Range | Meaning |
|---|---|---:|---:|---|
| `--tx-compressor on/off` | `on` / `off` | on | on/off | Enable or bypass compressor and limiter together |
| `--tx-comp-input` | `input` | 0 dB | −24…+24 dB | Fixed gain before the detector and compressor |
| `--tx-comp-threshold` | `threshold` | −24 dBFS | −60…0 dBFS | RMS detector level where hard-knee compression would begin |
| `--tx-comp-ratio` | `ratio` | 3:1 | 1…10 | Input/output level ratio above the knee; 1 bypasses gain reduction but retains makeup and limiting |
| `--tx-comp-knee` | `knee` | 6 dB | 0…24 dB | Width centered on the threshold; 0 selects a hard knee |
| `--tx-comp-attack` | `attack` | 5 ms | 0.1…100 ms | Gain-reduction attack time constant |
| `--tx-comp-release` | `release` | 200 ms | 10…2000 ms | Gain-reduction release time constant |
| `--tx-comp-makeup` | `makeup` | +6 dB | 0…24 dB | Fixed gain after compression, before limiting |
| `--tx-comp-ceiling` | `ceiling` | −1 dBFS | −12…0 dBFS | Peak ceiling at the processor's audio output |

Input gain cannot recover microphone ADC clipping. Set headset capture gain so
loud speech remains clean before increasing digital gain. Below the compressor
knee, input and makeup gain provide the quiet-speech lift. Above it, compression
reduces the loud-to-quiet range. Fixed makeup gain does not increase during
pauses, but it still amplifies microphone background noise; there is no noise
gate, speech detector, spectral shaping or adaptive noise suppression here.

## Changing settings from the controller

Stop TX and wait for its transition to finish, then enter, for example:

```text
tx compressor on
tx compressor threshold -28
tx compressor ratio 4
tx compressor knee 6
tx compressor attack 8
tx compressor release 200
tx compressor makeup 6
tx compressor ceiling -1
```

Each parameter edit preserves the other settings. Out-of-range or nonfinite
values are rejected. Settings cannot change while TX is running or transitioning;
accepted edits are applied when the next TX pipeline starts. Use
`tx compressor off` for a comparison with the original path. Settings persist
across TX starts and mode changes within the backend session; restart settings
come from the launch options, with microphone processing enabled by default.
An explicit off setting remains off across mode/source changes; selecting another
voice mode does not override that choice.

The passive monitor displays the saved parameters and whether the microphone
processor is enabled or bypassed. Its JSON configuration includes
`config.tx_compressor`. Existing v1/v2 reply layouts remain unchanged; parameter
readback is in the status JSON, not added to legacy command replies. The v2 wire
command is `tx_compressor PARAM VALUE`; enable uses `enabled 1` or `enabled 0`.

## DSP behavior and limits

The processor runs on normalized mono audio at 48 kHz, independently of the RF
sample rate. It uses a 10 ms exponential mean-square detector, a static soft-knee
compression curve and separate attack/release smoothing of gain reduction in dB.
The detector operates after input gain, with 0 dBFS referring to RMS amplitude 1
(a full-scale sine has RMS level approximately −3.01 dBFS). Timing values are
exponential time constants, rather than guaranteed times to reach final gain.

The limiter delays audio by 48 samples (1 ms). Its pending-sample peak detector
applies immediate gain reduction and a fixed 50 ms release while bounding the
processor's output peaks. State and delayed samples reset on every TX start
with the workers parked. It allocates no memory and takes no locks in the audio
processing loop. Invalid microphone samples become silence when enabled.

The ceiling applies **before the existing modulator filters and modulation**.
Later filtering, SSB envelope formation and RF hardware can change peaks. The
processor therefore does not certify final AM modulation depth, FM deviation,
SSB peak envelope power or adjacent-channel emissions. Existing mode-specific
settings remain relevant. Initial user listening feedback on 2026-11-22 was
positive; quantitative RF/spectral and intelligibility acceptance remain pending.

## Validation

The backend compiled successfully in an isolated directory on `pi0a-trixie`.
A synthetic 48 kHz benchmark processed 10 seconds of audio in 0.1905 CPU seconds
(about 1.9% of one Pi Zero 2 W core), including test-signal generation but excluding
RF modulation and transport. This is a DSP cost measurement, not a full-radio
real-time or intelligibility acceptance test. Installed radio binaries were not
replaced during validation.

The deterministic DSP test checks exact bypass, quiet-signal gain, compression
slope, limiting of abrupt peaks, lookahead delay, invalid input handling, reset,
release and invariance to audio block partitioning. It runs with strict math and
the production optimization flags:

```sh
python3 software/libcariboulite/tests/test_tx_dynamics.py
```

After building the backend on Linux, simulated backend tests exercise launch
options, controller edits, status readback, rejected input and TX-running locks:

```sh
python3 software/libcariboulite/tests/test_tx_dynamics_backend.py
```

The mocked TX worker lifecycle test also enables the processor, checks microphone
processing and reset, and retains the exact Quindar cue-count checks. None of
these automated checks transmits RF. Validation on the isolated Pi build passed
the backend-control tests, 19 existing TX-control tests, 37 controller
tests, 28 backend tests, and the eight-rate TX checks under strict and production
math. A subsequent controlled listening test
should compare bypass/processing on identical speech at quiet, normal and loud
levels, while measuring actual modulation peaks and occupied bandwidth.
