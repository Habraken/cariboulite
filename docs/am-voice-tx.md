# Narrow-voice AM transmission

AM TX uses a conventional carrier-present double-sideband envelope. It shares
the existing TX capture, clock adaptation, fractional native-rate packetizer,
SMI pacing, start/stop and half-duplex ownership logic with NBFM. All eight
native rates from 400 kS/s to 4 MS/s are supported.

## Operation

Launch with `--tx-mode am --tx-am-depth 0.8`, or use the controller while TX is
stopped:

```text
tx frequency 430.1
tx mode am
tx am-depth 0.8
tx source tone
tx tone 1000 0.5
tx power -12
tx start
```

Keep the controller connected while keyed. `tx stop`, controller disconnect,
or backend exit releases TX using the existing shutdown logic. Mode/depth
changes while keyed return `TX_RUNNING`. Changes rebuild retained TX resources
before the next start; they do not silently change an active waveform.

Use `tx source mic` for voice. Audio is clipped to ±1, then passes a second-order
300 Hz high-pass and fourth-order 3 kHz low-pass at 48 kHz. The filtered signal
is bounded again before modulation. There is no FM pre-emphasis in AM. Depth
is the modulation fraction at unit filtered audio, between 0 and 1 inclusive;
0 emits an unmodulated carrier. Default depth 0.8 reserves modulation margin.
Microphone gain and tone amplitude determine how much of this maximum is used.
A 0.5-amplitude, 1 kHz tone at depth 0.8 produces approximately 39.8% modulation
after voice filtering and linear interpolation to the RF sample clock.

Extra digital backoff can be set with `tx am-backoff DB` (0..18 dB), or
`--tx-am-backoff DB` at launch. It changes only AM and requires stopped TX.
The backend defaults to 6 dB after the attenuated-link linearity comparison.
This is a measured operating choice for this setup; more backoff did not
consistently improve distortion.

The IQ envelope is `out_scale * 10^(-backoff/20) / 2 * (1 + depth * filtered_audio)`, with constant
phase. It cannot become negative or exceed `out_scale`; peak bounding prevents
overmodulation, but excessive microphone audio can still sound distorted.
On audio underflow the modulator reports held ticks, falls back toward an
unmodulated carrier, and the real-time producer's existing underflow check fails
the stream. Missing audio is not treated as a successful voice frame.

## Power reference

`out_scale` is the maximum envelope at 100% modulation, not the carrier
amplitude. With the same 4000 IQ scale and RF power register setting, AM's
2000-IQ carrier before extra backoff is nominally 6 dB below the former constant-envelope NBFM
reference. At 100% modulation with no extra backoff the theoretical peak-envelope power
reaches that reference. With the default extra 6 dB backoff, the carrier is
nominally 12 dB and full-modulation peaks 6 dB below the NBFM reference. For smaller modulation depth the actual peaks are lower. Hardware
compression, calibration and frequency response can change these relationships.

The backend RF power value/estimate is not a calibrated AM carrier-power or
peak-envelope measurement. Use external RF instrumentation for those quantities.
The minimum 80 kHz analog/Fs÷8 digital TX frontend is shared with NBFM; voice
selectivity comes from the baseband filtering, not a dedicated analog AM filter.
Software tests do not certify radiated occupied bandwidth or transmitter purity.

## Controls and status

- `tx mode nbfm|am` selects saved TX modulation.
- `tx am-depth VALUE` selects saved AM modulation depth (0..1).
- `tx am-backoff DB` selects additional AM IQ backoff (0..18 dB).
- `--tx-mode`, `--tx-am-depth` and `--tx-am-backoff` set startup defaults.
- `config.tx_mode` and `config.tx_am_depth` expose settings to the passive monitor.
- v2 requests are `tx_mode am|nbfm` and `tx_am_depth VALUE`.

The v2 control reply now appends TX mode, AM depth and AM backoff. Update the
backend and controller together: older controllers reject the extended reply.
The updated controller accepts legacy replies but shows TX mode as unknown
instead of assuming NBFM. The passive monitor also reports the saved settings.
`rx mode` and the existing bare mode command continue to select RX only. Use
explicit `tx mode am` to select AM transmission. WBFM TX, USB and LSB remain
unsupported. NBFM deviation settings are retained but have no effect in AM.

## Validation and hardware connection

`tests/test_am_mod.py` uses a strict measurement oracle with strict/production
DSP builds. It verifies carrier/envelope, tone depth, voice filtering,
overmodulation bounds, all native sample clocks and real TX packet marker
packing, split/reset continuity, invalid input and carrier underflow fallback.
Existing NBFM reference waveforms, TX pacing/lifecycle, capture and ownership
checks remain in place. Integration tests check mode/depth saving and reject
changes during keyed TX. No device is opened by these software checks.

The operator confirmed the 90 dB attenuated direct RF24/HiF link before AM
hardware testing. pi4g-trixie is the transmitter and pi4f-trixie the receiver.
Tests use 430.1 MHz, beginning at the RF24 minimum power setting. AM reception
uses the calibrated RX playback stage; the NBFM-only SINAD estimator is not
used to assess AM.

Hardware validation at 430.100 MHz, the audible-artifact correction and remaining
linearity limits are documented in [the AM link report](am-voice-tx-validation-2026-11-15.md).

## Headset test profile after the TX wire-order correction

At 430.100 MHz on the 90 dB direct link, the operator reported substantially
less perceived quantization noise using TX power setting 6 dBm, AM backoff
0 dB and depth 0.8. This does not establish that remaining noise is eliminated.
These settings are explicit; an ordinary launch still uses power -12 dBm and
backoff 6 dB. Backoff changes digital RF amplitude, not nominal modulation depth.

For the Logitech H650e on pi4g, use `plughw:CARD=H650e,DEV=0` for capture.
The last measured hardware capture setting was 38 (-1 dB), AGC off: normal and
quiet speech were clear in direct playback and no samples clipped. Setting 44
(+5 dB) had clipped. Recheck current speech levels before increasing capture gain.

A fresh cued ten-second H650e capture at the same setting measured -2.77 dBFS
peak, -21.99 dBFS RMS and zero clipped samples; the two captured channels were
identical. Offline application of the actual AM voice-filter coefficients at
AM depth 0.8 yielded 62.24% peak and 4.15% RMS modulation. These are calculated
baseband values, not measured RF envelope values. They support investigating
speech compression/level control rather than a large fixed capture-gain increase.
The private recording remains in `/tmp/am-headset-current-level.wav` on pi4g.

To reproduce the improved link profile with this headset (start TX explicitly
from the controller only after confirming the attenuated connection):

```sh
./build/radio-backend/cariboulite_radio_backend \
  --rx-playback plughw:CARD=H650e,DEV=0 \
  --tx-microphone plughw:CARD=H650e,DEV=0 \
  --tx-source mic --tx-mode am --tx-am-depth 0.8 \
  --tx-am-backoff 0 --tx-power 6
```
