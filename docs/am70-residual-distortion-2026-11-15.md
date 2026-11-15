# Residual AM distortion on the 70 dB link — 15 November 2026

The residual quiet-audio artifact is real and reproducible after the TX wire-order
fix. It is not explained by reported stream interruptions, microphone clipping,
or the RX audio-level stage. Compression and other aviation-inspired processing
remain deferred. No production DSP, RF defaults or FPGA firmware were changed
in this investigation.

## Setup and scope

Both hosts used commit `81a0f08`, with pi4g transmitting and pi4f receiving on
RF24 through the operator-confirmed 70 dB wired link. Wanted RF was 430.100 MHz,
TX/RX rates were 1 MS/s, AM depth 0.8, RX offset tuning enabled, and captured
PCM volume -12 dB. The headset and speaker were excluded from objective tone
measurements. RF power settings are register requests, not calibrated carrier
power or PEP. RX gain was fixed manually: AGCC=0, AGCS=21 (63 dB request), or
42 dB in the reduced-gain comparison.

## Digital interface and software envelope

A temporary FPGA checker observed changing IQ words at the SMI collector and
modem external digital-loopback return. The sequence steps I by 73 modulo 8192;
Q is a rotated/XOR transformation of I. A full cycle exercises all 8192 codes
on both channels. Each checker verified 8191 consecutive transitions after
acquisition. Both completed without error. A second run deliberately flipped a
Q bit: both checkers detected the corruption. Simulation additionally verified
skipped-word detection and the distinct duplicate policies.

The collector requires every sequence transition. The modem-return checker
allows held/repeated words because the native sample-gap mechanism repeats
values on the 4 MS/s interface. It verifies changing-word order and contents,
not exact repeat counts, timing jitter or indefinite-duration reliability.
This test begins at the SDK sample input; it does not include microphone capture
or the software modulator. No RF TX command was issued, embedded TX control
was disabled, and the normal FPGA image was restored after the diagnostic.

A separate offline probe used the actual production AM modulator and frame
packetizer for 12 amplitude/backoff combinations, checking one million settled
IQ samples per combination. At input amplitude 0.01, backoff 0, the packetized
waveform retained approximately 0.797% modulation with a second/third harmonic
ratio of 0.51%. At amplitude 0.03 it retained 2.387% modulation with 0.089%
harmonic ratio. The weak signal is therefore present in software.

The packetizer still forces the low I bit to one as a legacy TX tag; in the
current serializer that is an actual I data bit. This introduces a bounded
zero-or-one-code change and restricts I to odd codes. It is a separate software
precision limitation, visible in the offline results, but cannot explain the
much larger RF loss below. It has not been changed in this diagnostic run.

## RF and receiver comparisons

The main matrix has 16 cases: power/backoff pairs (6,0), (6,6), (12,6), (0,0),
with amplitudes 0.01, 0.03, 0.1 and 0.5. Eight additional cases repeat selected
settings with RX gain reduced by 21 dB. These all completed without measured
stream-error increments or PCM clipping.

At power 6/backoff 0:

| Input amplitude | Software 2nd/3rd harmonic ratio | Recovered PCM ratio | PCM tone/residual |
| --- | ---: | ---: | ---: |
| 0.01 | 0.51% | fundamental too weak for a useful ratio | -2.9 dB |
| 0.03 | 0.089% | 11.8% | 17.4 dB |
| 0.1 | 0.050% | 2.1% | 31.4 dB |
| 0.5 | 0.0042% | 0.9% | 40.5 dB |

Changing RF gain alone did not restore the weak signal. Reducing RX gain
preserved approximately the same 11.7% and 2.1% harmonic ratios at amplitudes
0.03 and 0.1; its lower SNR worsened residual measurements. At backoff 6,
amplitude 0.03 measured about 41.8% at both power settings 6 and 12. Static
carrier rotations of 45 and 90 degrees did not provide a consistent improvement.

Four raw-IQ captures bypassed the AM audio processing. With normal TX and
amplitude 0.01, an independent narrowband envelope calculation measured only
0.152% fundamental modulation, versus the software's 0.797%. At amplitude 0.03,
raw-envelope second/third harmonic ratio was about 12.5%. None of the raw
captures reached ADC rails. This places the observed distortion before the RX
AM audio gain, audio filters, ALSA playback and speaker. The link remains an
end-to-end measurement; it does not independently isolate every TX/RX analog
stage. Previous operator comparisons on RSP2pro also implicated TX.

## Frequency-shifted diagnostic: improvement with an emission penalty

A temporary TX modulator rotates IQ at 25.2 kHz and compensates tuning to retain
the wanted carrier at 430.100 MHz. It uses the same audio filtering, depth and
RF gain; no speech compression, boost or amplitude dither was introduced.

At power 6/backoff 0, amplitude 0.01 improved from -2.9 to 16.5 dB PCM
tone/residual. At amplitude 0.03 the second/third harmonic ratio fell from
11.8% to 2.89%. The independent raw-envelope estimate at amplitude 0.01 restored
0.816% fundamental modulation, close to the software expectation.

However, raw-IQ spectra also showed a component at wanted carrier +25.2 kHz
around -25.2 dBc and one at +50.4 kHz around -41 dBc. These are consistent with
TX LO feedthrough and an image, but are relative end-to-end receiver estimates,
not calibrated transmitter emission measurements. The normal waveform measured
roughly -66 to -69 dBc in the same narrow offset bands. The diagnostic is **not
a production fix** and was not installed as the normal backend.

## Interpretation and next boundary

The evidence supports operating-point-dependent amplitude nonlinearity after
the digital interface, with the modem DAC/analog TX path a leading candidate.
It does not establish a specific count of discarded DAC bits or prove that a
particular internal modem block is responsible. A 13-bit digital interface does
not guarantee 13 effective analog bits. The modem's seven-bit DAC-override
registers alone are insufficient proof of the normal IQ path's effective
resolution.

A slow fine carrier-level sweep was confounded by roughly 0.51 dB change when
returning to its initial setting; it cannot establish precise amplitude steps.
A continuously keyed follow-up alternated reference I=2001 with each odd I code
from 1891 to 2101, holding each level for 2 ms and Q=0. This avoids TX/RX restart
between level changes. Independent analysis of two raw-IQ captures normalized
each test plateau to its adjacent reference plateaus. It found repeatable
plateaus and transitions about 64 input codes apart:

| Example I codes | RF amplitude relative to reference, approximately |
| --- | ---: |
| 1901–1951 | 0.968 |
| 1961–2015 | 1.000 |
| 2021–2071 | 1.030 |
| 2091–2101 | 1.058 |

The next transition regions appear near 1953, 2017 and 2081. Within-plateau
variation across repeated cycles was typically below 0.1% in these captures.
Ideal linear response would change by about 2.7% between I=1961 and I=2015;
the measured response stayed essentially unchanged. This is direct evidence
of coarse amplitude quantization in this fixed-phase, Q=0 operating condition,
consistent with approximately six low input bits not affecting steady RF
amplitude except around transition regions. It is not a complete specification
of effective converter resolution for arbitrary dynamic IQ, frequency or gain.
The bits themselves survived the digital-loopback check; the exact internal
quantizer/DAC implementation remains unobserved.

Before speech processing, address the legacy low-I-bit tag independently and
qualify any TX waveform change against both low-level fidelity and unwanted
emissions. A subcarrier approach would also need image/LO suppression, frequency
handling on both RF paths, native-rate tests, and start/stop validation. Improving
voice loudness alone would not resolve the measured transfer error.

[Baseline artifacts](baselines/20261115-am70-residual/README.md) contain settings,
analysis, temporary diagnostic source and FPGA restoration receipts. Private
speech recordings and raw RF/audio captures remain outside the repository.

## Subsequent listening comparison

Two cued 45-second Jabra passes used the same approved H650e normal/quiet
recording: A, unchanged production TX; B, temporary 25.2 kHz rotating IQ with
compensated tuning. Both used power setting 6 dBm, backoff 0 dB, depth 0.8,
1 MS/s TX/RX and RX volume -6 dB on the 70 dB link. The operator reported
**less noise/roughness in B**. This confirms an audible improvement, not complete
elimination. It does not remove the measured LO/image emission penalty or
qualify the diagnostic for deployment. Both passes stopped normally.
