# Uniform USB HID PTT — 28 November 2026

The agreed behaviour is the same for every supported device: green/call
toggles TX; disconnect only turns TX off and is a no-op when TX is already
off. Both stop paths restore the RX state saved before transmission, so an
idle radio returns to idle and a receiving radio returns to RX. Microphone
mute and volume are outside the PTT mapping.

The controller now discovers scalar HID Telephony Hook Switch, Flash and
Drop controls from report descriptors without a vendor allowlist. It handles
report IDs, scalar bitfields, padding, usage ranges and Global Push/Pop.
Unknown or unsupported layouts fail closed; opaque vendor protocols and
array-based call controls need an adapter. Auto selection requires exactly
one compatible interface. See [controller usage](../../radio-control.md#usb-hid-call-button-ptt).

Two measured device adapters are included:

- H650e `046d:0a46`: its active-low call pulse maps to the shared toggle.
- Jabra SPEAK 510 `0b0e:0420`: its input fields marked Constant are accepted
  for the measured report layout. The green button uses Flash, including
  while already in a call and after disconnect. The red button sends an
  intermediate Hook=1 report before clearing Hook. This intermediate state
  is ignored rather than relying on the packet pair arriving promptly;
  delayed and repeated red presses therefore cannot start TX.

The Jabra's Off-Hook indicator is enabled for the connected control session
so call controls remain available in RX and TX. The existing output report
is preserved and restored on normal controller exit. Startup feedback is
primed without issuing TX requests. H650e needs no such handshake. The
radio state remains visible in the passive monitor.

## Validation

The descriptor tests passed normally and with UBSan on both Pis, and with
AddressSanitizer/UBSan on the local Mac. Cases include captured device
reports, an unlisted generic device, numbered/unnumbered reports, usage
ranges, bit offsets, global state stacks, startup state suppression,
feedback-bit preservation, explicit-disconnect precedence, delayed Jabra
red reports, malformed descriptors and 5,000 randomized descriptor inputs.
GCC AddressSanitizer could not reserve its required address range on the Pi
kernels, so ASan ran on the supported local host instead.

The controller integration test used the real dry-run backend and verified
the shared mapping from both RX and idle, no TX start from repeated red,
green toggling TX off, early stop during startup, disconnect/RX restoration,
and existing CW ownership/practice/native-CW interlocks. All 37 existing
controller tests passed on each Pi. Both permanent controller binaries have
the same SHA-256, recorded with source hashes in the host summaries.

Live tests used NBFM, 430.100 MHz, 1 MS/s and requested −12 dBm TX power.
Pi4f uses the Jabra microphone/playback; pi4g uses the H650e. Both use matched
RX gain with 0 dB volume. Jan confirmed:

- Jabra green toggles TX and voice is audible through the H650e.
- Jabra red stops TX; a second red press leaves TX off. The controlled
  red-test phase added exactly one TX-on request (green) and one TX-off
  request (red), with no additional start from repeated red.
- H650e call toggling and voice continue to work with the shared decoder.

The complete controller logs counted five TX-on/off pairs on pi4f and one
pair on pi4g. During the bounded passive status recordings, neither host
reported capture/read/write errors, capture overruns, driver underrun
periods, playback underruns, clipped RX samples or TX faults. Both radios
finished in RX. Their normal controllers remain running with
`--headset-ptt auto`; the pi4f CW key input remains disarmed for this voice
test. No microphone audio was saved.

`f-summary.json` and `g-summary.json` contain test-log excerpts, source and
binary hashes, operator feedback and final states. Their compressed status
files preserve the recorded counters. `jabra-controlled-sequence.json`
records the RF-disabled Flash/red/green-after-red sequence and restoration
of its temporary indicator. The running controller's indicator then remains
active as part of the qualified Jabra call-control session.

Further hardware models, physical unplug during TX, and repeated operational
qualification remain future validation. Descriptor support does not imply
that every vendor's firmware follows the advertised semantics.
