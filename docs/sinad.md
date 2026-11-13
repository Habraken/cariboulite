# NBFM SINAD estimate

The backend can estimate the quality of a 1 kHz FM test-tone link. Enable it
on the **receiving** CaribouLite by adding `--rx-sinad` to the usual backend
launch, for example from the repository root:

```sh
build/radio-backend/cariboulite_radio_backend \
  --channel RF24 --rx-frequency 430125000 --rx-mode nbfm \
  --rx-sample-rate 1000000 --rx-sinad --rx-start
```

The passive monitor shows `RX SINAD` alongside noise-squelch RMS. This flag is
optional and off by default. It requires live NBFM reception; WBFM, stopped RX,
dry runs, initial settling, silence and windows without a detectable 1 kHz tone
report an unavailable value. Selecting WBFM leaves the requested flag set but
does not produce measurements. The monitor is passive and does not enable it.

On the **transmitting** CaribouLite, launch its backend on the matching RF
frequency and use the controller with TX stopped:

```text
tx source tone
tx tone 1000 0.4
tx start
```

Stop with `tx stop`. The default TX test tone is 600 Hz, so explicitly set
1000 Hz. At the default 2500 Hz TX deviation, amplitude 0.4 requests 1000 Hz
peak deviation. Record amplitude, deviation, frequency and RF power when
comparing readings. The receiver's measurement does not depend on its PCM
volume, RX audio deviation normalization, de-emphasis or squelch settings.
Use a stable attenuated RF link and exclude the TX startup/tail tones from
measurements. After any signal change allow at least a complete new window.

## Measurement definition and limitations

The estimator observes the existing normalized 48 kHz raw discriminator tap
before de-emphasis, audio gain, PCM clipping and squelch muting. A separate
second-order Butterworth high-pass at 300 Hz and second-order Butterworth
low-pass at 3000 Hz define the measurement response. These are filter corner
frequencies, not brick-wall integration limits. There is no de-emphasis or
standard audio weighting in this branch.

A 1000 Hz notch with Q=30 removes the fundamental. Over each nonoverlapping
500 ms window the estimator sums total filtered power and notch-output power,
then reports:

```
SINAD dB = 10 log10(total power / residual power)
```

Harmonics within the filter response remain in the residual. The estimator
runs while squelch is closed. Initial filter settling is discarded for 200 ms,
so the first result arrives after 700 ms of processed audio. A new RX worker
or reset clears the measurement. Windows whose total mean square is at most
1e-12, or whose power ratio is below 1 dB, are unavailable. The latter is a
reporting criterion, not an independent carrier or tone-identification test.
Readings are capped at 80 dB, including an ideal synthetic sine wave.

This is a link-quality estimate, not a calibrated receiver sensitivity or
standards-compliant SINAD measurement. The notch also removes a little noise,
which biases readings upward. Conversely a tone offset from the notch leaves
fundamental energy in the residual: a pure 1001 Hz sine measures about 24.4 dB.
Independent TX/RX clocks and the receiver's audio rate correction can limit high
readings. A future frequency-tracking analyzer can improve that limit. Other
signals with energy near 1 kHz can produce a reading; use a known test tone.
The result includes transmitter noise/distortion and the receiver RF/audio
measurement response; it excludes playback clipping and speaker behavior.

Protocol-v1 snapshots add the optional `rx.sinad` object:

```json
{"enabled": true, "db": 20.125, "tone_hz": 1000, "low_hz": 300, "high_hz": 3000}
```

`db` is `null` when unavailable. Older snapshots without this object remain
readable by the monitor. No control-protocol fields or commands are changed.

## Validation

`make -C software/libcariboulite/tools/radio_backend test` includes synthetic
estimator tests under the production fast-math flags: known harmonic distortion,
gain invariance, pure tone, frequency offset, silence, noise-only input,
non-finite input reset and initial window timing. Backend tests check live
serialization with a fixture and unavailable measurements in dry-run mode.
These checks do not constitute a two-radio hardware calibration.
