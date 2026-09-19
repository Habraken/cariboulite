# RX noise and carrier squelch

RX pipelines now enable noise squelch by default and leave carrier squelch off.
Menu 14 provides **N** to toggle noise squelch and **C** to toggle carrier squelch.
The display shows each enable state and the resulting audio gate. These choices
survive TX/RX switching and rate changes within that monitor session; a fresh
session restores defaults. Both enabled detectors must permit audio. Turning
both off gives the original PCM output without squelch processing.

## Signal path

Noise detection uses 48 kHz demodulated discriminator audio before DC removal,
de-emphasis, the voice low-pass filter and PCM gain. Measuring playback PCM would
make the threshold depend on volume and would discard much of the noise being
measured. `nbfm_demod_process_with_raw` supplies this optional parallel float
output with the same sample count as PCM. The original process API remains a
wrapper with no tap, and its PCM output is unchanged.

`noise_squelch` has two cascaded 6 kHz high-pass biquads followed by a 20 ms
exponential power average. The tap is normalized to approximately +/-1 for
+/-2.5 kHz deviation, not to PCM full scale. It opens below RMS **0.12** after
30 ms of qualifying audio and closes above RMS **0.18** after 120 ms. Between
thresholds it retains its state. It starts closed and resets closed on non-finite
samples. The filter history and noise measurement continue while audio is muted.
These are initial engineering thresholds, awaiting physical acceptance; they are
not calibrated sensitivity specifications. With all-zero IQ there is no detected
noise, so noise squelch alone can open: it is not a stream-validity detector.

`carrier_squelch` uses the selected AT86RF215 channel's RSSI. It opens at
**-97 dBm or stronger** for three consecutive 10 ms RF blocks and closes below
**-102 dBm** for fifteen blocks. Values in between retain the state. Invalid or
failed reads immediately close the detector. The chip reports signed RSSI,
with 127 indicating invalid, as specified in the
[AT86RF215 datasheet, section 6.2.5.5](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42415-WIRELESS-AT86RF215_Datasheet.pdf).
These thresholds refer to the modem measurement, not calibrated power at the
external connector or an SDRPlay reading; front-end losses and bandwidth matter.
AGC configuration is unchanged.

The RX reader performs one checked register read per captured 10 ms block only
when carrier squelch is enabled. The register follows the selected RF09/RF24
radio. It carries the value and validity with the IQ block through the RF FIFO,
so delayed DSP processing does not use a newer measurement from another block.
This is an end-of-block measurement, not sample-synchronous RSSI. It uses the
shared hardware lock with cancellation disabled until the lock is released.
This adds a short SPI transaction to the reader when enabled. It deliberately
checks the read result rather than using the existing cached-RSSI helper, which
can misinterpret a negative SPI error as a valid signed RSSI value.

Both detectors run in `demod_worker`, outside the standalone NBFM DSP. A combined
gate applies a 5 ms linear fade to PCM before the audio FIFO. Muted PCM still
contains 480 samples per block: clocks, queue feedback and playback keep running.
Existing buffering contributes additional audible latency. There is no new thread
or allocation in processing. Detector resets accompany DSP reset; changing the
enable flags re-qualifies the detectors. Controls and gate status use C atomics;
only the demod worker owns detector/filter state. Ordinary pipeline stop/join
and cleanup still own the hardware and buffers.

## Interfaces and scope

- [noise_squelch.h](../software/libcariboulite/src/noise_squelch.h): caller-owned
  state, `noise_squelch_reset` and one-sample `noise_squelch_process` at 48 kHz.
- [carrier_squelch.h](../software/libcariboulite/src/carrier_squelch.h): caller-owned
  state, `carrier_squelch_reset` and `carrier_squelch_process` once per 10 ms block.
  Both modules are device-independent and allocation-free; callers provide valid
  state pointers and serialize access.
- `rx_params_t.noise_squelch_disabled` defaults false;
  `carrier_squelch_enabled` defaults false. Zero-initialized RX parameters
  therefore select the requested defaults, including the normal RX menu and
  automated baseline runner.
- `rx_pipeline_set_squelch(p, noise_enabled, carrier_enabled)` changes enable flags
  during an initialized pipeline's lifetime. The control owner serializes this
  with init/destroy. `rx_pipeline_squelch_open` reports the effective decision,
  not the completion of the fade or the state of already queued playback.
- Thresholds are named constants in the two module headers. Runtime level
  adjustment is deferred; no tuning UI or persistence is introduced here.

Option 13 and the memory demo remain unsquelched diagnostic compositions.
Their controls do not create a production RX pipeline. Existing standalone DSP
and worker comparisons with both squelches disabled retain exact PCM equality.

## Validation and physical acceptance

Run `python3 software/libcariboulite/tests/test_squelch.py` for independent
hysteresis/timing checks and actual demod-worker tests at 2 and 4 MS/s. Tests
include speech-band audio, high-frequency audio noise, synthetic broadband RF
noise, clean FM, strong/weak/invalid RSSI, combined gating, live bypass and
continued PCM production while muted. A parallel-tap test verifies independence
from playback gain and de-emphasis. Lifecycle tests additionally mock RSSI reads
for both channels, disabled sampling, invalid registers and failed SPI access.
No RF hardware is accessed by these software tests.

Software validation passed: all twelve relevant suites (source, sink, tone,
modulator, demodulator, rate, memory adapters, squelch, RX lifecycle, TX stop,
monitor loopback and baseline-runner reporting), plus the application and
memory-demo builds. The disabled-squelch worker comparison remains sample-exact
against the frozen pre-refactoring implementation at both RF rates.

Jan accepted the adjusted implementation after physical testing; see the feedback
below. The following procedure remains available for regression testing in
menu 14 at each of 2 and 4 MS/s; acceptance does not imply that every listed
combination was separately reported:

1. With default noise ON/carrier OFF, start RX without a signal and check that
   noise is muted. Apply the known NBFM signal and check clean opening and speech;
   remove it and check closure without chatter. **N** should restore unsquelched
   noise when disabled.
2. Disable noise squelch with **N**, enable carrier squelch with **C**, and vary
   the received signal level across the opening/closing region. Compare the
   displayed modem RSSI, not the external receiver's power reading. Confirm
   independent opening and closure; then enable both and verify combined action.
3. Check RX stop/start and a stopped rate change, with the selected enable flags
   retained. Confirm option 13 still plays normally.

No interactive retuning test is required. Report if the initial noise or carrier
thresholds are unsuitable for the real RF noise floor; software tests cannot
establish these levels for the physical receive chain.

### Physical feedback

Jan reports that noise squelch works and carrier squelch opens, but initially
would not close. The test signal is approximately -90 dBm and the idle RF24 RSSI
is mostly -102 to -105 dBm. That explains why the original strictly-below
-105 dBm closing threshold did not qualify.

At Jan's suggestion, both thresholds were raised by 3 dB: open at -97 dBm,
close strictly below -102 dBm, retaining 5 dB hysteresis and the existing timing.
Jan confirmed satisfaction with the adjusted result and requested a commit.
Noise and carrier squelch are accepted for the reported setup. A reading of exactly
-102 dBm still resets the 150 ms closing qualification; frequent excursions to
that level may require further tuning. Tested RF rates were not specified.

The earlier apparent echo was confirmed to be simultaneous RSP2pro and
CaribouLite playback, not duplicated CaribouLite audio.
