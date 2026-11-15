# Narrow-voice AM reception

AM RX adds a conventional carrier-present, double-sideband envelope receiver.
The initial profile is **300–3000 Hz voice audio**, supporting all eight native
RF rates from 400 kS/s through 4 MS/s and 48 kHz mono playback. Broadcast AM
reception with a wider audio/channel profile is future work.

[AM voice TX](am-voice-tx.md) is now available alongside NBFM. USB and LSB
remain future work. The initial live receive check on pi4f-trixie at 119.050 MHz
produced intelligible speech; the operator reported good speech quality on
strong signals and noise on weaker ones. No physical AM TX validation has
been performed.

## Operation

From the repository root, an example receive configuration is:

```sh
./build/radio-backend/cariboulite_radio_backend \
  --channel RF24 \
  --rx-frequency 119050000 --rx-mode am --rx-sample-rate 1000000 \
  --rx-playback plughw:CARD=USB,DEV=0 --rx-am-agc on
```

This launch defaults to stopped reception. Once the receive antenna is
connected to RF24/HiF, use the controller:

```text
rx mode am
rx frequency 119.050
rx rate 4
rx start
```

Stop with `rx stop`. Mode and rate changes require stopped reception. Prefer
RX-prefixed commands for the airport listening test: these leave the saved
TX frequency and settings unchanged. The requested first live test is receive
only at 119.050 MHz on pi4f-trixie, with Jabra playback.

**The antenna connection and user confirmation are required before starting
that test.** Before any later physical TX test, the user must first confirm
that pi4g-trixie TX and pi4f-trixie RX are connected through the 90 dB attenuation
link. Do not start a TX test as part of antenna-connected reception.

The passive monitor reports `AM voice`, its voice-filter limits and whether
AM carrier normalization is enabled. Menu 14's `M` control cycles NBFM → mono
WBFM → AM voice → NBFM, with the existing stopped/loopback interlocks.

## Gain, filtering and squelch

Automatic carrier normalization is on by default. It estimates the mean
channel-filtered envelope using a 250 ms time constant. An initial 20 ms
peak acquisition and a guard for a new signal more than 2.5 times the current
reference prevent excessive startup gain. Normal modulation peaks do not
trigger that guard once the carrier estimate has settled. A minimum reference
of 128 IQ units caps gain at 32 times relative to the signed 13-bit IQ scale.

For manual audio gain, start with:

```sh
--rx-am-agc off --rx-gain 8000
```

Manual mode uses a fixed envelope reference of 4096 IQ units. The backend
now applies [calibrated RX playback](rx-audio-levels.md): use `rx volume DB`
for live volume, with `--rx-gain 8000` as unity legacy trim. This option controls software audio
normalization. AM disables the modem's fast RF AGC, whose continuous gain
updates would alter the AM envelope. RF gain stays fixed at 63 dB by default;
`--rx-am-rf-attenuation 12`, for example, selects 51 dB. This launch option
accepts 0–63 dB in 3 dB steps. Reduce RF gain if strong signals overload the
receiver; software normalization cannot undo RF clipping. The register settings
are written and read back before RX starts. FM retains its existing RF gain
policy. The frontend uses the minimum 160 kHz analog bandwidth.
A dedicated digital AM channel filter provides voice-channel selectivity.
The AM AGC launch option is retained when selecting another mode and returning
to AM; it is not a live controller command in this initial implementation.

The DSP preserves IQ amplitude, decimates to 50 kS/s, applies an additional
193-tap complex channel FIR at 3.9 kHz cutoff, takes the envelope, and converts
to 48 kHz audio. A second-order 300 Hz high-pass and fourth-order 3 kHz
low-pass provide the voice response. Stated audio limits are filter corner
frequencies, not brick-wall passbands. The first anti-alias frontend is shared
with the existing narrowband receiver; the limiter and FM discriminator are
not used.

FM de-emphasis is bypassed in AM; status reports effective de-emphasis zero.
The saved FM setting is retained for a later return to FM.

FM noise-RMS squelch and the NBFM SINAD estimator are not applicable to this
AM profile. Their readings remain unavailable, including when `--rx-sinad`
is requested. Attempting to enable FM noise squelch in AM returns `AM_NOISE`.
The saved NBFM noise-squelch preference is retained when switching modes.
Optional RSSI carrier squelch remains available, but the initial listening test
leaves it off to avoid hiding weak transmissions.

## Offset-tuned reception

The backend now defaults to offset-tuned AM reception (`--rx-am-offset on`).
The physical tuner is moved 12.5 kHz away from the requested channel, and an
exact native-clock complex mixer restores the wanted signal before the existing
channel filter. Receiver-fixed DC/image or leaked components are moved outside
the voice channel instead of being converted to audible beats by the envelope
detector. The selected RF band/conversion region is preserved at band edges.

Requested and reported RF channel frequencies retain their original meaning;
there is no need to add an offset to controller frequency commands. Register
readback reflects the physical tuner. `--rx-am-offset off` is available for
comparison/debugging. It is a launch setting, not a live controller command.
Legacy standalone DSP callers default to no shift and can opt in with
`audio_demod_set_am_offset` before processing. Pipeline callers opt in with
`am_offset_tuning_enabled`.

During the two-radio link test a separate component near 1.66 kHz was observed
in raw RX IQ; it fell substantially with the transmitter off. The wanted
carrier was only about 35 Hz from nominal. An earlier attempt to centre the
extra component by moving RX 1.7 kHz was therefore incorrect and created a slow
beat. Independent RSP2pro reception was clean; the offset-RX prototype removed
the audible artifact on the Jabra. The source of the TX-associated extra
component is not conclusively identified by that test.

## Software validation

`tests/test_am_demod.py` builds an independent measurement harness and tests
DSP under strict and production floating-point flags. It never opens a radio
or audio device. Coverage includes:

- All eight native rates, exact fractional-rate scheduling and 48 kHz counts.
- A 1 kHz, 60%-depth AM signal with positive/negative carrier offsets;
  automatic normalization across a fourfold carrier-amplitude range.
- Manual amplitude scaling, no FM de-emphasis, and unmodulated-carrier rejection.
- Voice filtering and rejection of a stronger adjacent modulated signal.
- Fragmented input/output, zero output capacity, partial-decimator reset,
  reset determinism, ±500 ppm output-rate correction and invalid inputs.
- A sudden strong transmission after silence without clipped PCM samples.

Existing FM waveform tests, mode/profile lifecycle checks, command protocol,
controller, backend and passive-monitor serialization checks also cover the
integration. NBFM and WBFM retain their existing DSP paths. The control protocol
keeps its v1/v2 wire layouts and represents mode as an enum internally.

These software tests do not establish RF reception quality, measured receiver
sensitivity, audio intelligibility or performance on live airport traffic.
The initial antenna-connected listening check has been performed as described
above. It does not establish calibrated sensitivity or weak-signal performance.

Hardware gain policy follows the [AT86RF215 datasheet, section 6.2.3](https://ww1.microchip.com/downloads/en/devicedoc/atmel-42415-wireless-at86rf215_datasheet.pdf).
