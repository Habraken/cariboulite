# U25 and TX write-timing diagnosis — 2026-11-06

Slowing the host SMI write timing restored approximately 100 application
frames/s and a consistently observed TX modem state on one new Full board at
1 MS/s. Returning to the original timing reproduced approximately 446 frames/s
and TXPREP/TX alternation. This supports a TX capture-timing problem; it does
not itself measure U25's frequency or qualify RF signal quality. The later
[divided-clock measurement](u25-clock-measurement-2026-11-06.md#measured-result)
established an approximately 37.125 MHz U25 clock instead of 125 MHz.

## Board, software and observations

- Board UUID: `d8810044-c444-46f4-a548-703412cbf971`; HAT vendor
  `CaribouLite Reproduction 2026Q4 PD9JH`, product `CaribouLite RPI Hat`.
- The user reports U25 is stamped `37.1`, rather than the expected `125`, and
  reports successful WBFM reception of a known station. A later measured
  divide-by-32 output of 1.1602 MHz establishes approximately 37.125 MHz at U25.
- Host: Pi 4, kernel `6.18.60+rpt-rpi-v8`; host SMI source clock readback
  `125000003 Hz`. This is the Pi peripheral clock, not a measurement of U25.
- Tested RF profile: RF24/HiF, requested `430100000 Hz`, requested `-12 dBm`,
  `1000000 S/s`, NBFM with the internal `600 Hz` tone. The modem TX profile
  read back 1 MS/s; FPGA sample gap `3` was observed through the diagnostic
  preload during the trials.
- The user reported picking up the test tone during this sequence. The exact
  trial, received pitch and signal quality were not independently identified.

Git HEAD was `a4e56b6fadd7f8834d75f74987724baebbb6857d`, with pre-existing
documentation changes. `provenance.json` records all source and binary hashes.
The library's 32,220-byte embedded FPGA image matches `firmware/top.bin`
(`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`).
Post-reboot initialization loaded it; these trials retained the operational FPGA.

## Controlled write-timing comparison

Only the TX write setup/strobe/hold tuple changed; write pace stayed `1`.
Rates below use stable snapshot endpoints after a one-second warm-up, within
one backend session and one driver session generation. One application frame
contains 10,000 IQ samples at 1 MS/s, so nominal demand is 100 frames/s.
DMA rates are byte-transport IQ equivalents, not measured RF sample rates.

| Trial | Write setup/strobe/hold | Stable seconds | Queue puts/gets, frames/s | DMA IQ-equivalents/s | Sampled modem STATE |
| --- | --- | ---: | ---: | ---: | --- |
| Original | `1/4/2` | 4.953 | 446.37 / 446.37 | 4,465,370 | 84 TXPREP, 15 TX |
| Intermediate | `1/8/4` | 4.950 | 179.00 / 178.59 | 1,787,415 | 99 TX |
| Slower | `2/12/6` | 4.952 | 99.97 / 99.97 | 999,277 | 99 TX |
| Original repeated | `1/4/2` | 4.952 | 446.48 / 446.48 | 4,466,464 | 84 TXPREP, 15 TX |
| Slower repeated | `2/12/6` | 14.967 | 100.09 / 100.09 | 1,000,558 | 297 TX |

TXPREP is `0x03`; TX is `0x04`. These are polled state observations, not
continuous measurements. Every trial had zero reported SPI failures, queue
drops, writer errors and driver underruns. All returned to idle with modem
TRXOFF (`0x02`), restored write timing `1/5/1` with pace `1`, and exited with
status zero and no cleanup errors. The producer and writer used their assigned
CPUs with locked memory; real-time scheduling was denied (`rt_error=1`) and
both used the recorded `SCHED_OTHER` fallback throughout the comparison.
The final direct SMI check found streaming state `0` and flags `0`, with read
and write timing both `1/5/1/1`; no backend process remained. Recorded hashes
of the original backend, library, FPGA image and driver/RTL inputs were
unchanged after the trials.

## Interpretation and limits

The repeat of the original failure after the timing changes strengthens the
link to SMI write timing. In the checked-in design, U25 is divided by two for
the FPGA system clock. RX captures LVDS with the modem's independent clock,
while TX synchronizes host write strobes and data through the system clock
domain. A slower-than-designed U25 could therefore leave RX usable while
causing missed or misaligned TX captures. Lost captures could keep the FPGA
FIFO requesting host data without producing matching valid modem samples.
This is a plausible mechanism, not a measured identification of the failing
edge. The subsequent divided-clock test confirms a lower-than-designed U25
frequency; it does not independently identify the precise capture failure.

The `2/12/6` result establishes a temporary 1 MS/s transport workaround on this
board. It does not qualify 2/4 MS/s, microphone operation, long-run stability,
frequency accuracy, modulation quality, spurious emissions or the other four
boards. The subsequent U25 measurement confirms the frequency mismatch on this
board; exact component identity and any replacement remain separate work.

## Temporary launch and evidence

The process-local `LD_PRELOAD` probe overrides only the hardware-paced TX
request for `1/4/2`, updates the expected tuple for normal readback verification,
and logs timing/sample-gap reads. Initialization and restore pass through.
No production source, installed library or FPGA image was replaced; the usual
backend command still selects its original TX timing.

With the diagnostic artifacts present, launch from the repository root:

```sh
CARIBOULITE_TX_WRITE_TIMING=2,12,6 \
LD_PRELOAD="$PWD/build/u25-tx-diagnosis-20261106/tx_timing_probe.so" \
./build/radio-backend/cariboulite_radio_backend \
  --channel RF24 --tx-frequency 430100000 --tx-power -12 \
  --tx-sample-rate 1000000 --tx-source tone \
  --rx-playback plughw:CARD=USB,DEV=0 \
  --tx-microphone plughw:CARD=USB,DEV=0
```

The backend starts idle; start and stop TX through the usual controller.
This command applies the diagnostic only to that process and installs nothing.

Evidence is under `build/u25-tx-diagnosis-20261106/`: `provenance.json`, SMI
before/after JSON, preload source/library, runner, and each trial's commands,
backend log, status JSONL and summary JSON. These ignored build paths are local,
ephemeral artifacts; preserve separately to retain the full measurement record.

The [five-trial summary](baselines/20261106-u25/tx-timing-results.json) is
versioned with the measurement milestone. The
[diagnostic helpers](../firmware/diagnostics/README.md) retain the timing-probe
source and its build command, so the temporary workaround can be reconstructed
after clearing the local build directory.
