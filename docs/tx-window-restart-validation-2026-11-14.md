# TX finite-window restart fault

Native TX-rate validation exposed another source of short audio disturbances.
At 400 kS/s, a 60-second test showed recurring SINAD dips down to 15.13 dB,
roughly every 10.5 seconds. At 500 kS/s, dips occurred roughly every 8.4 seconds.
The 4 MS/s reference remained between 21.11 and 22.11 dB in the sustained test.
Recorded DMA underrun, queue, writer, RX framing and playback counters stayed
zero, so those counters alone did not establish continuity.

The old driver limited each SMI transfer window to `0x00ffffff` bytes. That is
4194303.75 IQ pairs: 10.48576 seconds at 400 kS/s and 8.38861 seconds at
500 kS/s. Restart code wrote `SMICS_CLEAR`, discarding prefetched TX FIFO
bytes. The length could also end within a four-byte IQ word. Separate timer
and watchdog contexts could attempt the same restart without a shared lock.

At 400 kS/s, observed low-SINAD update times in one 60-second verified window
were 7.43, 17.86, 28.30, 38.94 and 49.37 seconds after the measurement began.
This follows the finite-transfer cadence, allowing for the 500 ms estimator
window and 200 ms status publication. Offline replay of the captured RX IQ
also reproduced a dip to 18.48 dB, with a brief discriminator peak near 0.71
against a normal tone amplitude near 0.4. The fault is visible before ALSA.

## Fix and checks

Finite TX windows now end on complete DMA-period boundaries (`0x00fe0000`
bytes with the current period size). Restarts preserve the SMI FIFO and use
the register transaction lock around the idle check and START write. Initial
stream setup and teardown still clear the FIFO as before. RX window setup
and its refresh path retain their prior behavior.

The new test compiles the actual restart code against a prefetched-FIFO
model, checks word/period alignment and preservation of unrelated control
bits, and runs 500 rounds with four competing restart threads. It also checks
that active, idle, RX and stopped streams are handled correctly. Existing
exclusive-open, serialized-transition, TX demand/admission and installer tests
pass. The module builds against the transmitter's running kernel.

The unchanged FPGA quiet-framing simulation was not run because `iverilog`
is absent; this fix changes only the Linux driver.

## Hardware comparison

The same wired setup, unchanged backend/DSP, 0 dBm requested TX power,
1000 Hz tone of amplitude 0.4, and RX fixed at 4 MS/s were tested before and
after a trial driver reload. Each source rate measured 60 seconds after a
three-second settling period. Both comparisons used the same bounded IQ and
playback capture interposers.

| TX rate | Old-driver SINAD range | Fixed-driver SINAD range |
| --- | ---: | ---: |
| 4 MS/s | 21.107–22.109 dB | 21.106–21.923 dB |
| 400 kS/s | 15.127–22.008 dB | 21.082–21.979 dB |

All recorded transport error counters remained zero in both comparisons.
The user also reported that the tones sounded clean after the change.
This is a repeated boundary test, not a guarantee covering unlimited uptime.

The final module preserves the prior RX setup behavior. It was built for
`6.18.60+rpt-rpi-v8` and loaded on both radios with existing parameters
`fifo_mtu_multiplier=6 addr_dir_offset=2 addr_ch_offset=3`. The prior installed
module was retained for rollback. Module hashes, compact measurement time
series, offline replay metrics and reproduction scripts are under
[validation data](baselines/20261114-tx-window-restart/).

Full before-fix IQ captures and playback recordings are retained in the ignored
build artifact directory under `cariboulite-tx-stability-20261113T225540Z/`.
The after-fix raw captures remain on pi4f-trixie under
`/tmp/cariboulite-tx-stability-20261113T230127Z/`; compact records and playback
captures were also copied into the local build artifacts.


The final driver also passed the complete repeated eight-rate TX matrix.
It is installed for the running kernel on both radios, and their generated
embedded module headers were refreshed. Previous installed modules are retained
under `build/driver-tx-window/installed-before-window-HOST.ko.xz` on each host.
No backend startup service was added.
