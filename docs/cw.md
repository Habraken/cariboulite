# Manual CW and CW audio over NBFM

The standalone radio backend, controller and monitor support manual CW input.
No text sender, message memories, Morse decoder, automatic character/word
spacing or iambic-B trailing element is included. The interactive test
application has no CW controls.

## Key input and timing

Connect the USB keyboard-emulating interface directly to the Pi. Its firmware
holds `]` for D10 and `[` for D9 until the respective contact is released.
The backend reads Linux evdev key states, independent of keyboard layout and
terminal focus. The selected device is exclusively grabbed only when explicitly
armed. Prefer a stable `/dev/input/by-id/...-event-kbd` path. The backend needs
read/grab permission for that input device as well as its normal radio/audio
permissions.

Default key type is straight, speed 12 WPM PARIS, configurable from 5–30 WPM.
At 12 WPM a dit is 100 ms, a dah 300 ms and the generated element gap 100 ms.
The operator supplies character gaps (three dits) and word gaps (seven dits).

| Key type | Behaviour |
|---|---|
| `straight` | Either contact directly holds the key down |
| `cootie` | Either contact directly holds the key down; no generated elements |
| `iambic` | Held contacts generate dits/dahs; squeezing alternates them |
| `ultimatic` | During a squeeze the last-pressed paddle repeats; releasing it resumes the remaining paddle |

Mapping is configurable by swapping the two inputs. By default `[` is dit and
`]` is dah. An atomic report pressing both together retains previous ultimatic
priority, or defaults to dit when none exists. A completed element is never
cut short by a priority change. Brief opposite-paddle taps are not queued after
release. Releasing an iambic squeeze completes the current element without a
trailing opposite element. Settings cannot change while contacts, generated
elements, envelope release or an RF transmission are still active.

Input loss or `SYN_DROPPED` releases the key, disarms the input and stops RF.
Reconnecting does not re-arm it. A fault may require the ordinary stop/recovery
command before re-arming. Arming rejects an already-held contact. Normal held
straight/cootie contacts have no transmission timeout; hardware/stream faults
and process shutdown still stop RF.

## Modulation, listening and sidetone

The selected RX radio mode determines the automatic TX path while keying is
armed. It must be `nbfm` or `cw`; the saved TX mode is synchronized when arming
and at the start of a keyed transmission. Frequency, sample rate and TX power
continue to use the existing saved TX controls, including an explicitly set
separate TX frequency.

* **NBFM:** generate the existing TX tone, initially 600 Hz and amplitude 0.4,
  shaped by the key envelope. The RF carrier remains on in inter-element gaps.
* **CW:** generate a shaped carrier at the requested RF frequency using the
  existing full-scale IQ reference and configured TX power. The NBFM audio
  amplitude does not reduce native-CW carrier amplitude.

Both paths enter TX automatically on key-down and return to RX after the final
envelope release plus a configurable 250 ms hang. A new element within that
interval keeps TX active. The usual Quindar opening/closing cues are bypassed.
The hang is generated on the RF-source timeline; actual antenna switching also
includes bounded driver queues and lifecycle work and still needs measurement.
The first element is buffered across RX-to-TX startup rather than being lost
if its contact has already been released. Excessive backlog disarms with an
explicit buffer error instead of dropping key transitions.

Local sidetone is generated independently of RF startup. A continuously owned
playback mixer combines RX audio and local keying, requesting 2.5 ms ALSA
periods and a 10 ms device buffer; actual negotiated values are logged. USB
firmware debounce, USB polling, scheduling and the audio device add latency;
“immediate” means independent of radio startup, not a zero-latency guarantee.
No hardware sidetone-latency result is claimed yet. The NBFM sidetone uses the
TX tone pitch/amplitude. Native-CW sidetone uses the CW listening pitch with
fixed 0.4 source amplitude. Existing RX volume/mute controls apply to sidetone.
Practice mode sounds the same local keying and inhibits automatic RF TX.
Manual PTT and independent TX source/mode changes are rejected while the CW input is armed, including in practice mode.

Native CW reception selects a complex channel around the displayed RF carrier,
then translates it to an audible beat note. Channel widths are 250, 500 and
1,000 Hz (full -3 dB width), default 500 Hz. Listening pitch defaults to 600 Hz,
configurable from 100–2,000 Hz independently of width. Offset tuning and HiF
spectral inversion follow the existing RF path policy. FM noise squelch and
SINAD estimation do not apply to native CW. The `matched` profile uses a
+35.5 dB CW trim, measured with pi4f transmitting to pi4g on the 430.1 MHz
bench link on 2026-11-28. It targets −24 dBFS during steady key-down at 0 dB
volume; silent gaps are excluded. The carrier must be centred in the narrow
filter. This is a fixed bench reference, not automatic level control for
arbitrary received signals. Existing volume and `--rx-mode-gain cw=DB`
controls remain available for the wanted listening level. See
[the receive measurements](baselines/20261128-cw-receive/README.md).
Zero IQ in key-up intervals does not establish zero RF leakage; this requires
an RF measurement.

## Start and control

Build with the usual backend Makefile. For example, on the Pi:

```sh
make -C software/libcariboulite/tools/radio_backend
build/radio-backend/cariboulite_radio_backend \
  --cw-device /dev/input/by-id/REPLACE_WITH_KEY_INTERFACE-event-kbd \
  --rx-playback plughw:CARD=H650e,DEV=0 --rx-mode cw --tx-power -12
```

In the controller, with RX stopped:

```text
cw setup iambic 12 0 1 250 5
cw rx 500 600
cw enable on
```

The six setup arguments are key type, integer WPM, swap (0/1), practice (0/1),
hang milliseconds (0–5000) and attack/release ramp milliseconds (1–20). The
example enables practice. After releasing the key and allowing its envelope
to finish, use `cw setup iambic 12 0 0 250 5` for RF-enabled keying. Hang values
shorter than the hardware buffering require particular bench attention;
250 ms is the agreed default, not a measured switching guarantee.

Use `cw enable off` to release the exclusive input grab. `rx volume mute` or
the existing RX volume command changes local output. `tx tone 600 0.4` controls
the NBFM tone. `cw rx 250 700` changes native-CW filtering/pitch while RX is
stopped. The monitor shows arm/practice state, key type, WPM, hang and errors;
full snapshots include contact and receive-filter settings in `cw`.

The wire commands are protocol-v2 `cw_enable 0|1`,
`cw_setup TYPE WPM SWAP PRACTICE HANG_MS RAMP_MS` and
`cw_rx BANDWIDTH_HZ PITCH_HZ`. `cw_test 0..3` supplies synthetic raw contacts
only in `--dry-run`; live backends reject it. Physical USB ownership does not
transfer to a controller connection, so closing the controller does not
release a held physical key. Disarm explicitly or stop the backend.

## Verification

```sh
python3 software/libcariboulite/tests/test_cw.py
python3 software/libcariboulite/tests/test_cw_session.py
python3 software/libcariboulite/tests/test_cw_backend.py
```

The first test is portable and checks all native RF rates, key timing and
complex-channel filtering under strict and production floating-point flags.
Session/backend integration tests run on Linux using simulated input and no
radio or audio device. Physical interface event capture, actual sidetone
latency, RF key-up leakage, key-click spectrum, first-element RF timing and
antenna TX/RX hang timing remain bench-validation items. No FPGA/driver
replacement is required for this implementation.

Software verification on 27 November 2026 passed the strict/production CW
DSP/keyer checks locally and the simulated CW session/backend tests on pi4f.
Existing backend, RX control, controller, TX control and FPGA-control groups
passed, together with mocked RX lifecycle and TX worker-parking checks. The
existing transport, AM/SSB, native-rate, signalling and dynamics groups were
also exercised during the regression run. Tests used an isolated development
build and did not install binaries or access live RF.

On 28 November, physical testing on pi4f confirmed the USB paddles and prompt
sidetone in practice mode, native CW TX, and keyed audio through NBFM. The
250 ms TX/RX hang felt natural at 12 WPM in both transmit paths. Native CW RX
on pi4g was then centred and gain-matched to approximately −24 dBFS; Jan
confirmed comfortable pitch and volume. These results establish a working
baseline with the selected defaults, while the quantitative RF/timing checks
above remain open. See [practice/TX results](baselines/20261128-cw-practice/README.md)
and [receive results](baselines/20261128-cw-receive/README.md).

For a permanent build in a normal Pi checkout, use
`make -C software/libcariboulite/tools/radio_backend`; the backend, controller
and monitor are under `build/radio-backend/`. Input arming and keyer/filter
settings are still selected through the controller for each session.
