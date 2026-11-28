# Jabra SPEAK 510 USB telephony controls — 28 November 2026

The Jabra on pi4f-trixie identifies as USB `0b0e:0420`. Its HID descriptor
includes the standard Telephony usage page, including Hook Switch and Phone
Mute, alongside vendor-defined reports. The relevant generic semantics are
defined in the [USB HID Usage Tables, Telephony page](https://www.usb.org/sites/default/files/hut1_3_0.pdf).
The report layout differs from the tested H650e. Several telephony input
fields are also marked Constant in this descriptor, so simply mapping Linux
key events or accepting only descriptor fields marked Data is insufficient.

Separate operator-confirmed button tests gave these results:

| Test state | Button | Observed input reports |
| --- | --- | --- |
| Idle | Green/connect | `03 01 00`, then `03 00 00` about 2 s later |
| Idle | Red/disconnect | None captured |
| Idle | Mute, twice | None captured |
| Host in-call indicator active | Red/disconnect | `03 03 00`, then `03 02 00` |
| Host in-call indicator active | Mute | `03 12 00`, then `03 02 00` |

The temporary output report `03 01 00` sets the standard Off-Hook indicator.
It enabled the call-control reports observed above. Writing the output also
produced an initial vendor report `04 02 00` and a `03 03 00` state report;
these must not be mistaken for physical button presses. The red-button pair
corresponds to the Hook Switch bit clearing, while the mute pair pulses the
Phone Mute bit. One complete mute pulse was captured before the bounded
in-call interval ended; the second physical mute action is not independently
confirmed in that interval. ALSA's capture switch remained on; this does not
establish the microphone's internal audio mute state.

The indicator automatically restored to idle with output `03 00 00`, with a
successful three-byte write recorded in `button-summary.json`. The Jabra
button capture was stopped. Pi4f remained in RX and its buttons were never
mapped to RF. The H650e call-button PTT implementation on pi4g was retained.

These observations support a common call-control action layer with device
adapters for report decoding and host call-state feedback. The later agreed uniform mapping is green/call → TX toggle and
disconnect → TX off only, preserving the previous RX state when TX stops.
Mute stays outside PTT. See the subsequent
[uniform implementation and validation](../20261128-usb-hid-ptt/README.md);
this inspection alone did not qualify Jabra RF operation or physical unplug
handling.

`device.txt`, `report-descriptor.hex`, `events.jsonl.gz` and
`button-summary.json` preserve the device identity, descriptor, input-event
capture, phase boundaries and indicator restoration. No audio was recorded.
