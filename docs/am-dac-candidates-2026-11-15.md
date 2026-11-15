# AM DAC correction candidates — 15 November 2026

Both calibrated frequency-shifted IQ and envelope noise shaping restore weak AM
modulation. In the cued comparison the operator reported that both improved
speech similarly, with no distinctive quantization-like roughness noticed.
The integrated noise-shaping option also passed a subsequent listening check.

Noise shaping is now available as **`--tx-am-dac-shaping on`**, default off.
It retains I-only AM, Q=0 and the requested carrier tuning; no speech processing
or manual IQ calibration is involved. Calibrated shifted IQ measured somewhat
cleaner but depends on calibration and showed LO leakage variation across starts.
It remains a diagnostic, not a production mode. Aviation-inspired processing
and SSB remain deferred; this work addresses the underlying AM conversion path.

Implementation commits are `368dd5d` (preserve the low I data bit) and `83b2f6d`
(opt-in AM DAC shaping). Both hosts were synchronized and all CMake applications
rebuilt. No permanent FPGA or kernel-driver change was needed.

## Controlled comparison

pi4g transmitted, pi4f received through the operator-confirmed 70 dB wired RF24
link. Wanted RF was 430.100 MHz. The main matrix used TX power setting 6 dBm,
AM depth 0.8 and digital backoff 0 dB. RX used manual gain 63 dB, AM offset
reception and loopback capture; measurements were taken from raw IQ before
AM AGC, playback calibration, ALSA conversion or the speaker. Source amplitudes
were 0.01, 0.03, 0.1 and 0.5 at TX rates 400 kS/s, 1 MS/s and 4 MS/s, with RX
held at 1 MS/s. Every comparison case used the same hardware TX calibration
register values, including the baseline and noise-shaped case.

At 1 MS/s and input amplitude 0.03:

| Waveform | Recovered fundamental modulation | 2nd/3rd harmonic ratio | In-band tone/residual |
| --- | ---: | ---: | ---: |
| Unshaped AM | 2.23% | 11.81% | 17.7 dB |
| Calibrated shifted IQ | 2.32% | 1.08% | 25.2 dB |
| Noise-shaped AM prototype | 2.27% | 1.12% | 23.7 dB |

At input amplitude 0.01, the unshaped fundamental was severely suppressed, while
both candidates recovered approximately the expected 0.8% modulation. Ratios
for nearly absent fundamentals are noise dominated and are not useful THD
measurements. At input amplitude 0.5, the three methods measured approximately
0.9%, 0.9% and 1.1% second/third harmonic ratio respectively in the reference
1 MS/s matrix. Thus improvement is concentrated at low levels, not every metric
at every amplitude.

These are relative end-to-end estimates, not standardized SINAD, complete THD,
calibrated carrier/PEP measurements or transmitter certification. The independent
envelope calculation selects ±6 kHz around the actual received carrier, then
measures its 300–3300 Hz envelope spectrum and fits the 1 kHz tone/harmonics.

## Candidate A: calibrated shifted IQ

A 25.2 kHz rotating carrier uses both I and Q; TX tuning is compensated so the
wanted carrier stays at 430.100 MHz. Audio filters and depth are unchanged.
Calibration was performed while keyed, avoiding repeated starts between each
adjustment. Coordinate searches adjusted hardware TXCI/TXCQ, digital DC offsets,
and Q-path gain/phase, using separate LO and image objectives.

For this TX unit/frequency/power, the selected diagnostic values were:

- TXCI=35, TXCQ=27;
- Q gain 1.016 and phase correction -0.009 radians;
- digital I/Q offsets -4 and -12 counts.

They are measured fixture values, not universal constants or new SDK defaults.
The existing SDK already performs a median startup calibration; it was not
replaced or attributed to a particular silicon erratum by this investigation.

The initial shifted waveform had approximately -25.6 dBc LO leakage and
-42 dBc image power in the measurement bands. Final stationary calibration
captures measured roughly -54 to -57 dBc LO and -60 to -62 dBc image. Across
the later start/rate/amplitude matrix, LO values ranged approximately -47 to
-61 dBc and image values -56 to -61 dBc. This substantial improvement does not
establish stable suppression at every temperature, frequency or power.

A deployable shifted-IQ solution would need calibration/profile management,
robust frequency/sideband handling on both RF paths and further spectral
qualification. Those changes are not silently enabled by the DAC-shaping flag.

## Candidate B: envelope noise shaping

The selected approach quantizes the envelope to a 64-code grid with second-order
error feedback at 40 kHz, holding each output between updates. It preserves the
average envelope through a sequence of neighbouring realizable amplitudes rather
than amplifying quiet speech. Q remains zero; the radio stays on its requested
tuning. Fast first/second-order native-clock experiments gave little benefit;
the lower update rate was selected from measured 20–160 kHz trials.

An interleaved-reference staircase directly checks whether this merely hides
distortion. It alternates I=2001 with each odd I code 1891..2101 for 2 ms per
level while continuously keyed. Two raw captures per waveform were independently
aligned, then each target was normalized to its neighbouring reference levels.

For example, the raw transmitter stayed essentially at 1.000 while I changed
from 1961 to 2015. With shaping, the corresponding output changed from about
0.981 to 1.007, close to the intended 2.7% change. The large amplitude plateaus
were removed in these captures. This demonstrates improved transfer fidelity,
not just a subjective masking effect. It does not certify perfect linearity
across every DAC code, band or RF operating point.

### Fractional native-clock correction

The integrated implementation uses the exact 4 MHz/divider timing. A stricter
production-compiler test exposed a small DC bias at 500 kS/s: 40 kHz updates
alternate 12- and 13-sample holds, so unweighted error feedback did not preserve
time-averaged envelope area. The reproduced bias at input amplitude 0.001 was
0.000444 of carrier (about 0.89 IQ code).

Feedback now stores error in IQ-code × sample-duration units, using the next
hold's actual duration. The test remained strict; it was not relaxed to accept
the bias. Strict and production builds pass all eight native rates, checking
zero/small/full/overdriven inputs, output bounds, tight carrier/fundamental
measurements, exact block partitioning and reset, and raw output when disabled.
The final weighted implementation was retested in 28 RF captures, including all
eight TX rates and additional fractional-rate/default-backoff cases.

## Spectral and silence tradeoffs

Noise shaping moves error out of the voice band; it does not create physical
DAC bits or make the transmitted spectrum identical to raw AM. In the 37.5 kHz
RX-offset check, its measured integrated 3.5–60 kHz out-of-voice energy was
roughly 2 dB above the raw/shifted references. The wider 4 MS/s RX/500 kHz analog
profile measured an approximately 1 dB increase over a ±200 kHz window.

The wide check also inspected ±100.8, ±151.2 and ±201.6 kHz bands so the shifted
candidate's higher-order products were not hidden by the initial ±60 kHz window.
The ±100.8 kHz bands were around -62 to -64 dBc in these captures. All such
values include receiver noise/spurs and its frequency response; they are not
a calibrated emission mask or proof of arbitrary out-of-band suppression.
Reduced-RX-gain and changed-RX-offset comparisons help bound that limitation.

With no modulation, measured 300–3300 Hz envelope-noise power was:

| Digital backoff | Shaping off | Shaping on |
| --- | ---: | ---: |
| 0 dB | -61.0 dBc | -61.1 dBc |
| 6 dB | -55.7 dBc | -53.9 dBc |

The backoff-0 result is effectively unchanged in this measurement; default
backoff 6 showed about 1.8 dB more in-band noise. These are separate captures,
not a traceable noise-floor specification. This tradeoff and the limited RF
qualification are why the new option is explicit and defaults off.

## Integrated build validation and use

The initial integration completed 53 raw-IQ captures: all eight TX rates with
four amplitudes, plus additional backoffs 3/6/9/18 dB and powers 0/12/14 dBm.
After duration weighting, the final build completed 28 additional captures,
plus four shaping-on/off silent-carrier checks. Every captured window had zero
measured stream-counter increments and no ADC rail samples. Very weak signals
at large backoff were noise limited; those captures establish operation, not
precise distortion performance. The stronger high-power cases retain analog
nonlinearity; shaping is not a cure for PA compression.

The full backend `make test` target passed after the fractional-duration fix,
including the new strict/production DSP oracle and 108 backend/controller
Python integration tests. All CMake applications rebuilt on both Pis.

The three prototype speech passes used the approved private H650e recording,
30 seconds each, with a Jabra beep before each. The operator reported both
candidates improved similarly, then explicitly reported no distinctive
roughness. The same assessment was reported for a final 30-second pass through
the rebuilt production backend with shaping enabled. Listening here used Jabra;
no new RSP2pro listening result is claimed.

For the tested live-headset profile on pi4g:

```sh
./build/radio-backend/cariboulite_radio_backend \
  --rx-playback plughw:CARD=H650e,DEV=0 \
  --tx-microphone plughw:CARD=H650e,DEV=0 \
  --tx-frequency 430100000 --tx-mode am --tx-source mic \
  --tx-am-dac-shaping on --tx-am-backoff 0 --tx-power 6
```

AM depth remains 0.8 by default. TX still starts explicitly through the controller.
The passive monitor reports `DAC shaping=on`. This flag is a startup selection;
there is no new live controller command or control-reply layout change.

[Baseline artifacts](baselines/20261115-am-dac-candidates/README.md) retain the
case data, prototype patches, calibration, regression evidence and final release
checks. Private voice and raw recordings remain outside Git. Both radios were
released idle after the work.
