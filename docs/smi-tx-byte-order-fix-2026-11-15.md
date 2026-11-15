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
