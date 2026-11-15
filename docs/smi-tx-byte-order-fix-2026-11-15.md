# SMI TX byte-order correction — 15 November 2026

The low-level AM distortion investigation found a software serialization error,
rather than missing IQ precision in the FPGA or modem digital interface.

The FPGA SMI collector expects four bytes in this order:

| Byte | Contents |
| --- | --- |
| 0 | SOF/control flags, I[12:8] |
| 1 | zero marker bit, I[7:1] |
| 2 | zero marker bit, I[0], Q[12:7] |
| 3 | zero marker bit, Q[6:0] |

The SDK constructed a 32-bit integer in that visual order and stored it natively.
On the little-endian Pi, the SMI bus emitted its bytes in reverse order. The
collector resynchronized to the final SOF byte and then combined it with the
next sample's reversed remaining bytes. For constant samples, this exchanges
the low I and Q byte fields; for changing samples, adjacent values can also mix.

For example, the hardware diagnostic observed:

| Sent I, Q | Collector and modem-loopback I, Q before correction |
| --- | --- |
| 2, 0 | 0, 1 |
| 0, 1 | 2, 0 |
| 1002, 0 | 768, 117 |

The last example explains the apparent coarse amplitude steps: low changes in
an I-only AM envelope were partly moved into Q while the high I bits remained.
This affected the shared TX transport, not just AM.

The SDK now writes the four protocol bytes explicitly in memory order, without
a host-endian integer store. The driver quiet-padding word follows the same
SOF-first wire contract. No permanent FPGA firmware change is required.

## Validation

- A regression against the real SDK serializer checks the actual output bytes
  for all 8,192 signed codes on both IQ channels. It failed before the fix and
  passes afterward.
- The driver quiet-frame RTL test now models actual Pi memory/SMI byte order;
  it also failed before its correction and passes afterward. Driver admission,
  transitions, restart, ownership and installer regression checks passed.
- Backend regression tests and Soapy/SMI timed-transfer checks passed, including
  fragmented TX byte-prefix retries.
- A temporary FPGA observer exposed both the SMI-collected word and the modem
  digital-loopback word over SPI. With the original serializer, 61 of 78 test
  patterns failed at both observation points. With the corrected SDK and the
  same original loaded driver, all 78 passed, including every individual bit,
  signed/mixed patterns and values 992 through 1039. This isolates the SDK fix.
- The temporary image met 64 MHz LVDS and 62.5 MHz system timing. Its unused SMI
  RX FIFO was omitted to fit the diagnostic logic, so it was used only for the
  digital test. No RF TX command was issued; EEC was disabled. Normal FPGA
  firmware was restored after every attempt.

The initial fixture attempts lacked explicit interface setup, correct bus
switching or production-style bounded buffering. Their results are excluded;
a fixed-pattern check and a nonzero SMI positive control were required in the
qualified test. Programming was done inside the initialized owning process to
ensure the intended temporary image remained active. The qualified comparison
runs are `/tmp/am-bittrace-20261115-200953` (before) and
`/tmp/am-bittrace-20261115-201633` (after). The normal FPGA image SHA-256 is
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.

RF level/backoff/native-rate and listening qualification follows separately;
this digital result does not by itself establish RF distortion or noise limits.

## RF regression and remaining limits

The normal production FPGA image and corrected SDK/driver completed all eight
TX and eight RX native-rate cases in both AM and NBFM. The AM run had 25 cases
including power/depth checks; the NBFM run had 16 native-rate cases. All had zero
stream-counter increments and no PCM clipping. All CMake applications were
rebuilt on both Pis, and both installed/loaded drivers report source version
`1AC65CD6AE81A1E2ED63F14`.

The corrected 20-case level/backoff sweep confirms that digital correctness does
not remove every RF distortion mechanism. At power setting 14 dBm and backoff
6 dB, the 0.1-amplitude tone's second/third harmonic ratio fell from 28.8% to
6.8%, and the 0.25-amplitude case fell from 23.2% to 2.95%. At amplitude 0.5,
the ratio changed from 0.84% to 2.11%; the corrupted waveform had not provided
a valid RF calibration reference. At zero backoff, weak 0.1-amplitude modulation
measured 0.17%, while the stronger 0.5-amplitude case measured 10.36%. There is
no single backoff setting at that RF power that fixes the complete level range,
so the default remains 6 dB.

A fresh temporary 25.2 kHz TX-offset experiment with corrected wire order
improved some weak cases but was not consistently distortion free. It is not
deployed. The digital checks locate the byte-order fault conclusively; they
do not yet locate the remaining RF nonlinearity within the modem's digital
frontend, DAC, analog TX path or external converter. Further RF transfer/spectral
measurements are needed before changing modulation architecture or adding dither.

Native AM tone-to-residual ratio was 29.95–30.82 dB, with 2.07–2.37% second/third
harmonic ratio. Native NBFM tone-to-residual ratio was 23.61–24.32 dB, with
0.63–0.96% second/third harmonic ratio. These are fitted end-to-end PCM metrics,
not calibrated SINAD or RF spectral purity measurements. Listening at power
setting 14 dBm/backoff 6 dB confirmed quiet speech still distorted on both the
CaribouLite/Jabra and independent RSP2pro.

An additional eight-case gain-staging test used zero digital backoff and power
settings 0 and 6 dBm. At power setting 6, amplitudes 0.1 and 0.5 measured
2.05% and 0.87% second/third harmonic ratio respectively. The 0.03-amplitude
case remained much weaker and measured 11.71%. In the repeated cued listening comparison, the operator confirmed normal and
quiet speech clear on both CaribouLite/Jabra and RSP2pro at power setting 6 dBm,
backoff 0 dB and depth 0.8. This qualifies that profile at 430.100 MHz, not every
power/frequency/modulation level. The existing defaults are retained.

[Baseline artifacts](baselines/20261115-smi-tx-wire-fix/README.md) retain the
before/after bit traces, exact settings, regression CSVs and diagnostic observer.
Both radios were released idle after the tests.
