# Backend LED controls and #4 RX isolation — 16 November 2026

Added and deployed explicit LED commands in the backend and controller on both
pi4f and pi4g. A backend-only experiment reproduced the earlier test-app recovery
and narrowed it to **LED1's programmed state on #4**. No automatic startup
workaround or FPGA firmware change was made.

## Commands

With RX and TX stopped, use the interactive controller:

```text
fpga leds
fpga leds 1 0
fpga leds 1 1
```

The first command reads LED0/LED1 plus BTN/CFG. Writes set both LED bits and
perform one checked readback. Same-state writes are sent; writes are not retried.
SPI errors report HARDWARE; mismatched LED readback reports VERIFY. An explicit
LED reply carries an optional `fpga_io N` tail in protocolv2. Ordinary get/status
replies and polling do not add FPGA I/O. Active RX/TX, transitions and an FPGA
requiring programming reject LED operations. A stopped RX fault allows diagnostic
LED access without automatically clearing the fault.

The existing digital getter now propagates SPI read failures instead of reporting
success with zero-valued outputs. Tests cover that behavior, protocol validation,
controller parsing/readback, repeated writes, simulation defaults, active-radio
rejection and reset defaults. All13FPGA-control tests, protocol tests, the SPI
mock tests and control-server tests pass.

## Hardware experiment

Board #4 UUID `c2440a8d-3852-4c44-b96f-1b64a962e7dc` remained on pi4f with the
user-confirmed90dB link. Pi4f was rebooted; boot ID
`07693751-414f-44ed-aa14-d754fd13d82e`. The experiment used one live backend and
the built controller for every operation. No test app was run. No transmitter
was started on either board; these are sample-synchronization checks at4MS/s,
not RF/SINAD qualification.

Initial readback was LED0=1, LED1=1, BTN=1, CFG=15, consistent with the requested
firmware marker. Baseline RX failed. Read-only access, writing1/1 again and toggling
only LED0 did not recover RX. The exact prior test-app write pattern
1/0 → 1/1 → 0/1 → 0/0 recovered RX. A subsequent state matrix gave:

| Programmed LED0 | Programmed LED1 | #4 RX start |
| ---: | ---: | --- |
| 0 | 0 | Success |
| 1 | 0 | Success |
| 0 | 1 | Failure |
| 1 | 1 | Failure |

Alternating0/0 → 1/1 → 0/0 repeated success → failure → success. Successful
three-second observation windows emitted roughly13million samples. Alignment
counters are cumulative across failed starts; nonzero inherited counters must
not be described as clean full-run transport acceptance. One invalid frame was
recorded during a transition to the failing state.

The original pi4g reference UUID `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` started RX
successfully with1/1 → 1/0 → 1/1, with no alignment failures or invalid frames.
An initial comparison accidentally selected pi4f's audio-card name USB; it failed
before acquisition and is excluded. The valid comparison used pi4g's actual
H650e playback device. Both attempts are retained.

## Finding and limits

LED1 state predicts the observed RX-start behavior on #4 under these settings;
LED0 alone does not. The original board did not show the dependency. This makes
LED1's pin/net, output loading, physical coupling and the applicable FPGA pin
mapping useful next inspection targets. It does not prove a solder bridge,
clock fault or RTL defect. The board's oscillator frequency was not measured.
No permanent initialization write or relaxed acquisition check was added.

#4 was left stopped with LED0=0, LED1=0. The reference was left stopped with1/1.
Both radios released their SMI descriptors. Full logs and status windows are
archived; [source hashes, summary and evidence](baselines/20261116-board4-backend-leds/README.md)
record the experiment. The prior test-app investigation remains separate evidence.

## Subsequent register comparison — 17 November 2026

The [before/after comparison](board4-register-comparison-2026-11-17.md) found
only the intended LED1 bit change in the ten exposed readable registers across
three stopped low/high cycles. RX still succeeded with LED1low, failed with
LED1high and recovered low. No unintended exposed-control change was observed;
unexposed or transient state and electrical causes remain unresolved.
