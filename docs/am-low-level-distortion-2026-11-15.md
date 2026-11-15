# Quiet-speech AM distortion investigation — 15 November 2026

The quiet-speech symptom is reproducible with a generated 1 kHz tone, without
using the headset microphone. It is an amplitude-dependent distortion within
the voice passband, rather than the normal sound of a 300–3000 Hz voice filter.
The TX chain is the leading suspect given the operator's similar observations
on CaribouLite and RSP2pro; the exact failing hardware/data stage is not yet
identified. No production modulation or RF defaults were changed in this run.

## Controlled RF checks

Production source: `c695de8`. pi4g-trixie transmitted and pi4f-trixie received
through the existing 90 dB wired RF24/HiF link at 430.100 MHz. Both streams ran
at 1 MS/s; AM depth was 0.8, RX offset tuning was enabled, and tone recordings
used RX volume -12 dB. A matrix of source amplitudes 0.03, 0.1, 0.25 and 0.5
and digital backoffs 0, 3, 6 and 9 dB used power setting 14 dBm. Additional
power checks used settings 6 and 12 dBm at backoff 6 dB. Power settings are
not calibrated AM carrier power or PEP.

At the default 6 dB backoff and power setting 14 dBm:

| Source amplitude | Baseline 2nd/3rd harmonic ratio | Diagnostic dither | Software-only DSP |
| --- | ---: | ---: | ---: |
| 0.03 | 99.3% | 4.69% | 0.095% |
| 0.1 | 28.8% | 1.51% | 0.033% |
| 0.25 | 23.2% | not tested | 0.038% |
| 0.5 | 0.84% | 2.30% | 0.029% |

The software-only check passes the generated audio through the actual AM
modulator and demodulator, including signed-16-bit IQ conversion. Its IQ carrier
is approximately 1002 counts. It excludes hardware, ALSA, queues and clock
reconciliation; its absolute playback levels and residual noise are not directly
comparable with RF measurements.

The RF measurements fit the actual recovered tone frequency in the middle two
seconds of each three-second 48 kHz recording. Harmonic ratio includes only the
second and third harmonics, not all distortion. Tone-to-residual results in the
CSV are not standardized SINAD. In cases where the fundamental almost vanishes,
a harmonic percentage is noise dominated and should not be treated as a precise
THD measurement.

The response contains plateaus and abrupt changes as carrier IQ level and
modulation amplitude cross particular operating points. A model truncating
roughly eight low IQ bits reproduces several of these features, including
suppression of small tones at some backoffs. This is a diagnostic hypothesis,
not a verified modem specification or identification of missing physical bits.
The intended interface carries 13-bit signed I/Q samples; see Microchip's
[AT86RF215 datasheet, sections 4.5.4 and 6.1.1](https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/Atmel-42415-WIRELESS-AT86RF215_Datasheet.pdf).

## Experiments that did not establish a production fix

- Changing RF power did not remove the low-level distortion.
- A temporary 100 kHz digital TX offset, with RF tuning compensated to retain
  the same wanted carrier, improved some cases but left significant distortion.
- Constant carrier phases of 90, 180 and 270 degrees changed the result but
  did not produce consistently clean weak modulation.
- Slower SMI write timing (setup/strobe/hold 1/8/4 instead of 1/4/2) produced
  essentially the same result: 99.5%, 28.9% and 0.76% at amplitudes 0.03, 0.1
  and 0.5 respectively. Original timing was restored by pipeline cleanup.
- A temporary 16-step, zero-mean IQ amplitude dither, spanning -120 to +120
  counts, greatly improved quiet tones and restored a much more proportional
  amplitude response. This supports coarse amplitude quantization somewhere
  after the floating-point modulator. It slightly worsened the strong-tone
  result and its RF spectral effects have not been measured. It is **not a
  production fix** and was not installed as the backend.

All 46 RF diagnostic cases completed with zero measured stream-counter
increments and no PCM clipping. These checks include RX queue loss,
read/alignment/playback errors, TX writer errors and hardware underruns. This
rules out reported stream interruptions during these captures; it does not
verify the transmitted sample values at the modem input.

## Headset comparison and gain correction

The H650e exposed automatic gain control as off and microphone capture gain at
its maximum setting 44 (+5 dB). An uncued initial recording was discarded after
the operator clarified they were waiting for an audible cue.

A cued 20-second stereo recording then caught normal and quiet speech. Its two
channels were identical, ruling out cancellation when converted to mono. Each
channel contained 244 full-scale samples. The operator judged the quiet part
clear when played directly through the Jabra without RF.

Capture gain was reduced by 6 dB to setting **38 (-1 dB)**. A second cued
20-second recording had **zero clipped samples**, with peak **-3.42 dBFS**.
The headset was left at this reduced gain; automatic gain control stayed off.
This fixes an independently observed source-clipping problem, but cannot explain
or fix the generated-tone RF distortion. Raw microphone recordings remain in
`/tmp` and are not committed.

## Next diagnostic boundary

Inspect the values received at the modem's digital TX input with known changing
low-bit patterns, before attributing the quantization to a DAC or RF stage.
The modem provides an external digital I/Q loopback intended for interface tests
(datasheet section 4.5.7). Integrating that with the FPGA RX FIFO and the shared,
half-duplex SMI bus needs a dedicated test: it must not enable loopback during
normal RF TX. If the digital words are intact, the next boundary is the modem
DAC/analog TX path and converter. No bandwidth widening, speech compression or
permanent dither is justified by the present measurements.

Exact measurements, source identities and temporary diagnostic patches are in
[the baseline directory](baselines/20261115-am-low-level/README.md).

## Subsequent digital finding

The next investigation identified and corrected an SDK wire-byte-order error.
All 78 hardware IQ patterns now reach the modem exactly. See the
[byte-order correction report](smi-tx-byte-order-fix-2026-11-15.md). The earlier
quantizer-like response was partly explained by exchanged low I/Q byte fields;
it was not proof of missing FPGA/DAC input bits. Residual RF AM distortion
remains after correcting the digital transport.
