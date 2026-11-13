# Native RX sample rates and playback buffering

RX now supports all eight native modem rates down to 400 kS/s in NBFM and
WBFM. The controller's `rx rate` menu uses hexadecimal divider keys; explicit
`rx rate 1` still means 1 MS/s. A one-shot selection is `rx divider A`.

| Menu key | Exact rate (MS/s) | Integer Hz command label |
| --- | ---: | ---: |
| 1 | 4 | 4000000 |
| 2 | 2 | 2000000 |
| 3 | 4/3 | 1333333 |
| 4 | 1 | 1000000 |
| 5 | 0.8 | 800000 |
| 6 | 2/3 | 666667 |
| 8 | 0.5 | 500000 |
| A | 0.4 | 400000 |

These are the native 4 MHz/divider encodings in the
[AT86RF215 datasheet](https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/Atmel-42415-WIRELESS-AT86RF215_Datasheet.pdf).
Labels are rounded to the nearest Hz; DSP timing uses exact divider ratios.
666666 Hz is also accepted as an alias. Dividers 7 and 9 are unsupported.

Integer decimation preserves the existing DSP paths where possible. New
polyphase FIR frontends handle the fractional ratios before the existing
NBFM 50 kS/s discriminator and WBFM 250 kS/s discriminator. Both produce
48 kHz audio. WBFM selects a modem cutoff of at least 100 kHz at lower rates.

## A second, separate audio fault

During the first RX-rate hardware validation, the user heard distortion
resembling intermodulation in the last sample (800 kS/s). This case failed
validation: it had 485 playback underruns in 20 seconds despite approximately
21 dB SINAD and zero IQ alignment errors.

The kernel publishes each 131072-byte DMA quarter as a burst of 32768 IQ
samples. At 800 kS/s, bursts arrive every 40.96 ms; at 400 kS/s, every 81.92 ms.
The previous 50 ms ALSA buffer, with 40 ms startup prefill, cannot reliably
bridge those bursts. This differs from the byte-alignment fault fixed in
commit 8618ddd, which damaged pre-playback signal measurements.

A controlled comparison held the RF settings constant at 800 kS/s and changed
only requested ALSA buffering through a diagnostic interposer:

| Buffer | Measurement | Underruns | Mean SINAD |
| ---: | ---: | ---: | ---: |
| 50 ms | 30 s | 725 | 21.61 dB |
| 200 ms | 30 s | 0 | 21.40 dB |

Playback call timing showed bursts separated by approximately 40 ms in both
cases. The audio adapter's recovery path adds repeated interruptions when the
short buffer runs dry. Such interruptions can produce modulation sidebands;
a single-tone test does not establish actual RF intermodulation.

The production fix sizes playback buffering for two DMA burst intervals plus
30 ms, with a minimum of the original 50 ms. The ALSA start threshold remains
one audio period below the negotiated buffer size. This increases receive
audio latency at lower rates, reaching about 194 ms requested buffering at
400 kS/s. Actual negotiated period and buffer sizes are logged at startup.

## Software validation

The complete backend test target passes. Added DSP tests exercise all eight
rates in NBFM and WBFM, including full 75 kHz synthetic WBFM deviation,
waveform amplitude, sample counts, fractional phase timing, arbitrary input
chunks and reset reproducibility. They run with strict and production
floating-point flags. Controller and backend tests cover rate labels, divider
menus and the distinction between explicit MS/s values and menu keys.

Playback tests verify buffer negotiation, startup prefill, parameter rejection
and existing write/recovery behavior. Pipeline lifecycle tests pass with the
new buffering API.

## Hardware validation

pi4g-trixie transmitted at 4 MS/s through the user's wired 90 dB attenuation
into pi4f-trixie at 430.1 MHz. The Jabra was the playback device. Each RX case
started a fresh backend. NBFM cases measured 20 seconds with TX power request
0 dBm and a 1000 Hz tone of amplitude 0.4 (1000 Hz nominal peak deviation).
WBFM cases settled for six seconds, captured one million decoded IQ samples,
and measured another four seconds. The TX tone amplitude was 1.0 and power
request +6 dBm for those cases; TX remained NBFM with 2500 Hz deviation.

All 16 mode/rate cases passed: zero recorded alignment failures, invalid
frames, RX read errors, IQ/audio queue drops, playback underruns and TX driver
underruns. Measured IQ and PCM delivery matched expected rates within the
short-window tolerances (3% and 4%). This validates these runs, not unlimited
uptime or every hardware startup condition.

| Divider | NBFM mean SINAD (dB) | WBFM captured tone (Hz) | WBFM raw amplitude |
| --- | ---: | ---: | ---: |

| 1 | 21.86 | 1000.004 | 0.03296 |
| 2 | 21.45 | 1000.092 | 0.03290 |
| 3 | 21.06 | 1000.000 | 0.03287 |
| 4 | 20.97 | 1000.000 | 0.03287 |
| 5 | 20.90 | 1000.000 | 0.03285 |
| 6 | 21.04 | 1000.000 | 0.03279 |
| 8 | 20.99 | 1000.008 | 0.03276 |
| A | 20.95 | 1000.015 | 0.03277 |

Replaying the WBFM IQ through the production DSP recovered 1000 Hz at every
rate, with amplitude approximately 0.033, consistent with 2500/75000
normalization. Tone-fit SNR was 18–20 dB for this narrow-deviation source;
these are waveform checks, not WBFM sensitivity or SINAD measurements.
WBFM monitor SINAD is intentionally unavailable.

[Structured results, capture analysis and reproduction tools](baselines/20261113-rx-native-rates/)
are committed. Full logs and bounded IQ captures are retained under
`build/radio-backend/implementation-20261113/cariboulite-rx-buffered-rates-20261113T223229Z/`.

## Playback waveform check

A separate 12-second-per-rate capture recorded PCM accepted by ALSA at 4,
4/3, 0.8, 2/3, 0.5 and 0.4 MS/s. Every run had zero recorded underruns.
Analysis of the trailing four seconds found no clipped samples, tone
amplitudes 2827–2865 PCM units, frequency 999.51–999.56 Hz and free-frequency
tone-fit SNR 21.56–22.19 dB. At 800 kS/s the fit SNR was 21.89 dB, comparable
to the 4 MS/s reference at 22.19 dB. The strongest measured second-through-fifth
harmonic across these captures was −35.8 dBc at 400 kS/s.

These checks found no large additional distortion specific to the repeated
800 kS/s case. They capture digital samples before the speaker, so they do
not measure the Jabra's analog/acoustic output. The reported intermodulation-like
sound was not established as RF intermodulation. Raw playback captures remain
in the ignored build artifact directory under
`cariboulite-pcm-probe-20261113T223735Z/`; compact analysis and reproduction
sources are committed alongside the hardware results.
