# CDCSS transmit encoding and receive code squelch

The standalone backend supports independent TX and RX Digital-Coded Squelch
(CDCSS, also known as DCS). It is **off by default** and applies only to **NBFM**.
AM, USB, LSB and WBFM RX bypass saved CDCSS selections. Within each direction,
CTCSS and CDCSS are mutually exclusive; TX and RX can use different systems,
codes or polarities.

## Launch and controller

Append, for example, to your normal backend launch:

```sh
--tx-cdcss 023N --tx-cdcss-deviation 250 --rx-cdcss 023N
```

The earlier `dcs` command and launch-option spellings remain accepted aliases.
Help, replies, monitor labels and `config.cdcss` use CDCSS.

A code has exactly three **octal** digits (0–7), optionally followed by `N`
(normal) or `I` (inverted). An omitted suffix means normal; suffixes are
case-insensitive. Use `off` to disable. `000N` is an enabled code, distinct from
`off`. All 512 nine-bit values are accepted; use the same manufacturer-supported
code on the other radio. Arbitrary values can have rotational aliases or less
favorable run lengths; accepting a value is not an interoperability guarantee.

The TX deviation setting is peak RF deviation in Hz, default **250 Hz**, range
50–1000 Hz. It must be strictly below the total `--tx-deviation` whenever TX
CDCSS is selected. With the usual 2500 Hz total, the default reserves 250 Hz for
CDCSS and limits voice to 2250 Hz before adding the sub-audio signal. This is an
initial narrowband setting, not a calibrated level for every receiver.

Stop the relevant direction and wait for transitions before editing:

```text
rx stop
rx ctcss off
rx cdcss 023N
rx start

tx stop
tx ctcss off
tx cdcss 023N
tx cdcss deviation 250
tx start
```

Use `rx cdcss off` / `tx cdcss off` before selecting CTCSS in that direction.
Attempts to enable both are rejected with `CONFLICT`, without changing the
accepted configuration. Launching with both selected is also rejected.
RX code changes require RX stopped; TX code/deviation changes require TX
stopped. Lifecycle transitions retain the existing configuration locks.
Accepted settings persist across PTT and mode changes within the session;
restart settings come from the launch arguments.

Polarity is part of the on-air bitstream. It is not an extra independent code
space: for example, **023N and 047I are rotationally equivalent** and can open
the same receiver. A wrong-code test must choose a genuinely different stream,
such as 023N versus 025N. No automatic polarity acceptance or code scanning is
performed; the receiver matches the configured code/polarity and its rotations.

## Backend protocol and monitor

New v2 requests:

```text
rx_cdcss 023N
tx_cdcss 023I
tx_cdcss_deviation 250
rx_cdcss off
```

Existing v1/v2 reply layouts are unchanged. The monitor and status JSON provide
readback through `config.cdcss`: `rx_code`, `tx_code`, `tx_deviation_hz`,
`rx_enabled`, `tx_enabled`, `valid`, `detected`, `turnoff` and `bit_errors`.
The enabled flags describe mode-dependent configuration. `detected` is null
until a complete receiver word is available, and `bit_errors` is null while
closed. When open, errors are the last accepted configured-word distance,
not an RF bit-error-rate estimate. `turnoff` indicates a qualified turn-off
signal and remains latched until a fresh code acquisition/reset.

RX gain profiles, volume and microphone compressor controls retain their
existing behavior. Code detection is from the raw discriminator tap, independent
of playback gain, de-emphasis, audio gating and RX deviation normalization.

## Wire format and DSP

TX repeats a 23-bit Golay codeword at **134.4 bits/s**. The nine code bits are
sent least-significant first, then fixed bits 0,0,1, then eleven parity bits.
The cyclic generator is x^11+x^9+x^7+x^6+x^5+x+1. The normal-polarity reference
word for 023 is `0x763813`, with integer bit zero sent first. Logical one means
positive RF deviation; inverted polarity complements the 23 transmitted bits.

The bit clock uses the exact rational increment 14/5000 at 48 kHz, independent
of the RF sample rate. Four cascaded 180 Hz poles smooth NRZ transitions without
overshoot. The shaped signal is added after voice conditioning, pre-emphasis
and voice headroom limiting; the microphone compressor never processes CDCSS.
Voice receives the same fourth-order 300 Hz high-pass used by CTCSS.

RF-route polarity is accounted for explicitly: CDCSS NBFM TX conjugates IQ on
the low HiF mixer route, and RX compensates the existing HiF I/Q ordering using
the receive-spectrum route helper. This avoids treating a two-Pi loopback's
possible polarity cancellations as proof of external-radio interoperability.
CTCSS and NBFM with CDCSS off retain their previous IQ convention.

RX uses four 300 Hz low-pass poles and a 2 Hz DC rejection stage. A bounded
clock/phase bank searches five rates (nominal and ±0.5/±1%) and ten sampling
phases per rate. Each 23-bit window is compared with all rotations of the
configured on-air word. Up to three differing bits are accepted, with three
independently spaced whole-word matches required to open. Typical acquisition
is roughly 0.5–0.8 seconds; this conservative qualification differs from
single-word commercial decoders. A 350 ms hold tolerates a missed word. The
filtered signal must also exceed 0.006 normalized RMS; weak-signal behavior
requires physical validation.

The effective gate requires CDCSS **and** every enabled noise/carrier detector.
Detection continues while closed or playback-muted. Playback passes through a
fourth-order 300 Hz high-pass when RX CDCSS is selected. This attenuates sub-audio
energy; it does not remove every possible high-frequency harmonic of a poorly
shaped external CDCSS signal. Existing 5 ms audio gate ramps remain in use.

All DSP storage is bounded, with no allocation, file I/O or locks in the CDCSS
sample processing. Nonfinite input resets the receiver closed, including under
production fast-math flags.

## PTT and turn-off sequence

CDCSS adds an **800 ms sub-audio-only acquisition lead-in**, followed by the
existing 250 ms opening Quindar cue and 50 ms post-cue interval before voice.
Every PTT start resets the bit clock, codeword, shaping and voice-filter state.

CDCSS continues through the existing 50 ms pre-stop interval and 250 ms closing
Quindar cue. At the start of final padding, it sends **134.4 Hz sine-wave
turn-off signaling for 180 ms**, then silence. Eighteen extra padding frames
retain the entire original silent FIFO/DMA drain after the marker. RX detects
the turn-off sinusoid through an eight-cycle correlation window and closes
after two qualified estimates. Signal level/noise can prevent fast turn-off;
the ordinary hold timer remains the fallback. Fault aborts skip normal cues
and turn-off signaling, as with the existing TX lifecycle.

## Validation

Hardware-free checks:

```sh
python3 software/libcariboulite/tests/test_cdcss.py
python3 software/libcariboulite/tests/test_squelch.py
python3 software/libcariboulite/tests/test_tx_worker_parking.py
```

After building the Linux backend/controller:

```sh
python3 software/libcariboulite/tests/test_cdcss_backend.py
```

CDCSS DSP tests run under strict and production math. They pin wire-format
vectors, check all 512 codewords, three-bit configured-word correction,
normal/inverted polarity and the 023N/047I alias, independent reference timing,
clock offsets, wrong codes, noise, silence/reset, turn-off and playback
sub-audio rejection. Actual modulation/demodulation tests cover all eight RF
rates, total deviation bounds with full-level voice, and bit-exact disabled /
non-NBFM bypass.

RX-worker fixtures exercise CDCSS-only gating, combined squelch, wrong polarity,
polarity compensation, turn-off, queue progress while muted and non-NBFM
bypass at 1 MS/s. Existing CTCSS/noise/carrier cases remain at 1/2/4 MS/s.
Mocked TX lifecycle tests run once with CTCSS and once with CDCSS, retaining cue
counts, parked-worker, restart, fault/cancellation and diagnostics checks.
Backend/controller tests cover code syntax, saved readback, launch options,
mode bypass, conflicting settings, deviation budget and running-state locks.

Software checks do not establish interoperability with a commercial CDCSS radio,
physical code polarity on every RF route, RF spectral acceptance, weak-signal
false-open rates or end-to-end timing. These remain physical acceptance tasks.
No automated test above keys real RF hardware.

Implementation validation (22 November 2026): the backend and controller build
without warnings on both pi0a and pi0b, with identical binary hashes between
boards. The five CDCSS backend/controller integration tests pass on each board, including
the CDCSS launch options and legacy DCS aliases.
The native RX-worker and repeated-PTT suites pass, as do the existing CTCSS,
compressor and radio-control regressions. The final DSP suite passes under
both strict and production math locally and natively on pi0b. Previous application binaries are
retained in `build/radio-backend/pre-dcs/` on each Pi. Both radio applications
remain stopped after validation.

Physical acceptance (22 November 2026): the operator confirmed that CDCSS works
over the radio link and verified rejection of both a wrong code and no code.
RX opens with the matching code and remains muted in those rejection cases.
This validates the tested link configuration; the broader interoperability
and RF acceptance tasks above remain separate.

## References

- [RF Technology T50 manual, DCS section](https://www.rftechnology.com.au/downloads/Manual%20T50.pdf): repeated NRZ Golay signaling and bit rate.
- [Communications Specialists DCS-23 specifications](https://www.com-spec.com/dcs23.htm): polarity selection, turn-off signaling and playback high-pass filtering.
- [GopherTrunk author's wire-format reference](https://github.com/MattCheramie/GopherTrunk/blob/v1.3.6/internal/scanner/conventional/tonecode/tonecode.go): bit order, Golay generator, reference word and inverted-code aliases. Our encoder uses polynomial division rather than a copied lookup table.
