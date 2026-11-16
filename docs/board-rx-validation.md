# Receiver identity and per-board qualification

On 2026-11-16 the user confirmed that all preceding pi4f-trixie RX RSSI/SINAD
results belong to board **#1**, UUID **`d8810044-c444-46f4-a548-703412cbf971`**.
The device-tree identity was read before the proposed swap and matches the
[recorded EEPROM initialization](flashing/README.md).

This retrospective association is explicitly user-supplied; it does not turn
historical files into contemporaneous EEPROM observations, change measured
values or identify the transmitter. It includes the receiver held at 4 MS/s
while TX sample rate was varied. Machine-readable attribution and the primary
report list are in [associations.json](baselines/board-rx-identities/associations.json).

## Board #2 — initialized, scoped RX qualification complete

The user labels the previous receiver #1 and its replacement #2. Board #2 is
installed on pi4f with the nominal 90 dB link to the original TX reference on
pi4g (UUID `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`). The TX reference is not #1.

Board #2 has UUID **`36b67ac4-b313-4369-99bd-4b3e09caabfa`**. Two initial 4096-byte
reads matched and were entirely FF. The checked blank-only writer programmed
and verified each page, then two complete readbacks matched the prepared image.
After reboot, HAT identity/product/variant matched and `/dev/smi` appeared with
the expected driver. [EEPROM and boot evidence](baselines/20261116-board2-rx/)
records these results. Full local backups remain under `build/board2-20261116/`
on pi4f and in the archived local evidence on pi4g.

The user reports that #2 still has the incorrect U25 oscillator. Its frequency
has not been measured on #2; do not transfer #1's scope measurement to #2.
Board #2 stays RX-only; successful identity programming is not RF acceptance.
The guarded collector requires #2's boot UUID on pi4f and confines TX commands
to pi4g. All eight native RX rates initialized and the 120-point RX-rate/power sweep
completed with zero recorded transport errors. All 2,878 retained snapshots
reconcile to the summaries. No TX trials were run on #2; both radios were
released afterward. See the [#2 report and #1 comparison](board2-rx-qualification-2026-11-16.md).
The measured SINAD increase is an uncalibrated link observation, not isolated
receiver-quality or oscillator acceptance.

## Board #3 — initialized, scoped RX qualification complete

The user installed #3 on pi4f and confirmed the nominal 90 dB link to the same
original TX reference on pi4g. UUID **`f23c0015-fd1b-47b7-8314-f5c9e7de0ea7`**
was generated for its confirmed blank EEPROM. Page checks and two full final
readbacks matched the prepared image; after reboot, HAT identity and `/dev/smi`
were verified. [EEPROM and boot evidence](baselines/20261116-board3-rx/) is
separate from #1/#2. All eight native RX rates completed a 120-point sweep,
with zero recorded transport errors and valid SINAD in every transmitting
measurement window. All 2,885 retained snapshots reconcile with the summaries;
no TX state/start command was recorded on #3. Both radios were released.
See the [#3 report and comparisons with #1/#2](board3-rx-qualification-2026-11-16.md).
The inherited RX-only restriction remains; no oscillator-frequency measurement
or TX qualification is inferred for #3.

## Qualification procedure (repeat for subsequent boards)


1. Read and preserve two complete EEPROM backups, check agreement and inspect
   any existing identity. A boot-time device-tree identity alone does not verify
   EEPROM contents. Preserve an existing valid unique identity; assign a fresh
   UUID to a confirmed blank board. Do not copy the preceding board's UUID.
2. Prepare the correct variant's image with the agreed identity fields and
   overlay. For a blank device, use the checked page writer with header-last
   ordering, per-page verification and complete repeated readback. Retain the
   image, backups, hashes and programming receipt under the new UUID.
3. Verify boot-time recognition after reboot, matched driver/FPGA setup and RX
   initialization. Do not run TX trials on the replacement board. Record startup
   failures or clock/transport limits as results rather than bypassing them.
4. If RX is viable, repeat the RX-rate axis of the NBFM sweep: eight native rates,
   pi4g TX fixed at 4 MS/s, 1 kHz tone/amplitude 0.4/deviation setting 2500 Hz,
   430.100 MHz RF24 and the restored nominal 90 dB link. Each rate has TX off
   and fourteen power requests from -12 to +14 dBm, with two seconds settling
   and five seconds measurement. This gives 120 points. Additional fixed-RX /
   varying-pi4g-TX comparisons remain possible without transmitting on pi4f.
5. Capture UUID/variant, source/binary/firmware/driver versions, requested and
   observed rates, RF setup, raw snapshots and error counters in a separate
   record. Compare matching settings with the preceding receiver; identify all
   failed or unsupported conditions and retain the original data unchanged.

RSSI and the fixed-notch SINAD estimate are uncalibrated link measurements.
The wrong oscillator may prevent initialization or reliable acquisition at some
rates; successful identity programming alone is not RX qualification. A full
RX/TX hardware acceptance requires the correct oscillator and separate tests.

The first board's EEPROM scripts and receipts remain under
`build/eeprom-init-20261106/` on pi4f. Their blank-only writer refuses nonblank
EEPROMs; recovery of a nonblank or partially programmed device needs inspection
of its saved contents before choosing a restore procedure.
