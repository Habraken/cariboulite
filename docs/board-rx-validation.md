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

## Board #4 — EEPROM verified, RX qualification failed

Device #4 on pi4f has UUID **`c2440a8d-3852-4c44-b96f-1b64a962e7dc`**.
Two matching blank reads preceded programming; page verification, repeated full
readback and post-reboot HAT/SMI recognition passed. RX startup failed twice at
4 MS/s and once at 2 MS/s with SMI synchronization errors. Six lower rates were
measured across 90 points. Invalid SMI frames appeared at 800 kS/s and 400 kS/s;
15 retained windows had nonzero cumulative invalid-frame counters. Other
retained transport counters were zero. The failed runs remain archived, and the
continuation retains failures explicitly. Both radios were released. No TX test
or oscillator measurement was performed on #4. The nominal 90 dB link was
inherited from the preceding setup, without independent reconfirmation.
See the [#4 report](board4-rx-qualification-2026-11-16.md) and
[evidence](baselines/20261116-board4-rx/README.md).

### Subsequent #4 retest and reboot check

After the user's manual test-app/LED/RX session, the [fresh full retest](board4-rx-retest-2026-11-16.md)
passed all 120 points with zero recorded transport counters. A requested pi4f
reboot was followed by startup failures at4and2MS/s and invalid frames in lower-rate
measurements. The [post-reboot run](board4-rx-reboot-2026-11-16.md) was stopped
after33completed points at the user's request to prioritize #5. Both radios are
released; diagnosis is deferred. Earlier failures remain retained.

### #4 controlled startup diagnosis

A later [two-boot investigation](board4-startup-diagnosis-2026-11-16.md) reproduced
4MS/s RX startup failure before a test-app LED-toggle session and clean short
RX probes afterward. Soft reset, reprogramming and an initial test-app-only
session did not clear the first failure. The cause remains unconfirmed; no
permanent workaround was applied. Both radios were released.

### #4 backend-only LED isolation

The [new backend/controller LED commands](board4-backend-led-control-2026-11-16.md)
reproduced recovery without the test app and isolated a state dependency: #4
started4MS/s RX with LED1=0 and failed with LED1=1, regardless of LED0. The
original pi4g reference started RX in either state. Cause remains unconfirmed;
no automatic LED initialization workaround was added. Both radios were released.

### #4 full RX sweep with LED1=0 after reboot

The user-requested [post-reboot LED1-low sweep](board4-led1-low-rx-qualification-2026-11-16.md)
passed all120points across eight native RX rates and the full established TX-power
request grid. All2,879retained snapshots reconcile; every transmitting window had
valid SINAD and all transport counters were zero. LED0 stayed1 and explicit
LED1=0 setup was independently verified before all eight rate starts. Both radios
were released. This is conditional RX success with LED1low; the LED1high failure
and its unresolved cause remain separate findings.

### #4 additional 1240.1 MHz qualification

On 17 November the same eight-rate RX/power sweep at 1240.1 MHz passed all
120 points with LED1=0 and LED0 preserved. All 2,877 snapshots reconcile, all
112 transmitting points had fully valid SINAD, and transport counters were zero.
The original pi4g board transmitted through the user-confirmed nominal 90 dB
coax link; both radios were released. The existing
[#4 qualification report](board4-led1-low-rx-qualification-2026-11-16.md) now
contains separate 430.1 and 1240.1 MHz results and plots. Cross-frequency
comparisons are uncalibrated end-to-end link differences.

### #4 additional 144.1 MHz qualification

The unchanged 90 dB coax setup was used on 17 November for a further full
120-point sweep at 144.1 MHz. LED1=0 was verified before all eight RX-rate
starts. All 2,877 retained snapshots reconcile, all 112 transmitting points had
fully valid SINAD, and transport counters were zero. Both radios were released.
The [qualification report](board4-led1-low-rx-qualification-2026-11-16.md) retains
separate 144.1, 430.1 and 1240.1 MHz results, plots and uncalibrated comparisons.

### #4 frequency response at fixed 0 dBm and 1 MS/s

On 18 November the [full frequency sweep](board4-frequency-sweep-2026-11-18.md)
measured 172 frequencies from 30 to 5,999.999 MHz, using both radios at 1 MS/s,
0 dBm requested TX and the existing nominal 90 dB coax link. LED1 remained low.
The plan covers every listed amateur/ISM range with at least three points.
Twenty-one frequencies were repeated with squelches disabled; the estimator is
pre-gate, and 16 frequencies still have no reportable SINAD. The final selection
has 156 fully numeric SINAD points and 4,287 paired snapshots. Three isolated
startup invalid-frame events were observed across the two runs, with no new
transport errors during retained measurement windows. Both radios were released.
These remain uncalibrated link measurements, not receiver-only sensitivity.

### #1 repeat of the full frequency sweep

After the user swapped board #1 onto pi4f on 18 November, the identical
172-frequency plan and 21 open-squelch follow-up points were repeated with
0 dBm requested TX, both radios at 1 MS/s and LED1 low. Plan bytes, software
hashes and backend arguments apart from socket paths match board #4's run.
All 4,289 selected paired snapshots reconcile. Board #1 has 155 fully numeric
SINAD points and 17 unreported values; 16 unreported frequencies are shared
with board #4, with 2250 MHz additional on #1. The 2650 MHz RSSI peak is present
on both boards, while their bypass-region RSSI levels differ substantially.
Board #1 recorded zero transport counters and no invalid-frame events across
both runs. Both radios were released. See the
[board #1 report and overlaid comparison](board1-frequency-sweep-2026-11-18.md).

### #1 repeat with the laptop distant

The user moved the laptop away, then authorized the same 172-frequency plan
and 21 open-squelch follow-up points on board #1. Both remain at 1 MS/s, TX
request 0 dBm, LED1 low and the nominal 90 dB coax setup. All 4,289 selected
paired snapshots reconcile. There are 157 fully numeric SINAD points, one
partial point and 14 unreported values. The 2650 MHz peak persists at
−58.36 dBm versus −58.24 dBm nearby, with no reported SINAD in either run;
the bypass-region lack of SINAD also persists. Three startup invalid-frame
events were observed, with no new transport errors in retained measurement
windows. Both radios were released. The
[laptop-distant report](board1-laptop-distant-frequency-sweep-2026-11-18.md)
retains the comparison and placement/causality limitations.

## Board #5 — EEPROM verified, all RX startup attempts failed

Device #5 UUID: **`b43713d5-7ebf-45ff-b327-aa2e0962c9e2`**. The user confirmed
installation on pi4f and the90dB link to the original pi4g transmitter. Two matching
blank EEPROM reads preceded programming. Page verification, repeated full
readbacks, image validation and post-reboot HAT/SMI recognition passed.
All eight native RX rates then failed startup with SMI synchronization errors;
no power-sweep/RSSI/SINAD points were collected. No manual test-app initialization
was performed after reboot. Both radios were released. No production software
change, TX test on #5 or oscillator measurement was performed; cause remains open.
See the [#5 report](board5-rx-qualification-2026-11-16.md) and
[evidence](baselines/20261116-board5-rx/README.md).

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
