# H650e call-button PTT — 28 November 2026

The Logitech Stereo H650e on pi4g-trixie identifies as USB `046d:0a46`.
An initial mixed-button capture was superseded after the operator reported
an incorrect sequence. Separate, operator-confirmed five-second holds then
established these input report pairs:

| Button | Pulse | Reset |
| --- | --- | --- |
| Mute | `08 02` | `08 03` |
| Call/phone | `08 01` | `08 03` |

Each pair was about 1 ms apart despite the five-second hold. This interface
therefore supplies clicks rather than a sustained held state. Linux also
emits synthetic `KEY_MICMUTE` events, including duplicates on call reports;
the controller uses the distinct raw HID call pulse instead of that key.
See `controlled-button-capture.json` for the corrected capture.

The C radio controller adds `--headset-ptt PATH` for click-to-toggle TX/RX and
`--headset-observe PATH` for button detection without PTT commands. It checks
the tested USB ID, ignores mute/volume/reset/unknown reports, and debounces
call pulses for 250 ms. It uses the existing persistent controller connection
and TX ownership. Unplugging the headset or ending the controller releases
owned TX; reconnection requires starting a new controller session. CW
practice/ownership and native-CW manual-TX interlocks remain intact.

The isolated headset-PTT test passed against the real dry-run backend,
covering observe/mute isolation, duplicate pulses, toggling and stopping
during startup, disconnect/RX resume, and CW interlocks. All 37 existing
controller tests also passed on pi4g. A physical observe-mode test detected
one call-button click while microphone audio remained present: median
100 ms RMS was −23.75 dBFS before and −22.87 dBFS after, with no all-zero
windows. The ALSA capture switch stayed on. Only level statistics were
saved, not microphone audio; see `microphone-check.json`.

The selected live test uses NBFM at 430.100 MHz, 1 MS/s, requested −12 dBm
TX power: pi4g transmits microphone audio from the H650e to pi4f, which plays
received audio on the Jabra SPEAK 510. Both use matched RX gains at 0 dB
volume. The pi4f paddle input is disarmed for this voice test, retaining its
Iambic 12 WPM swapped-paddle configuration for subsequent use. The prior CW
sessions are saved under `build/headset-ptt-20261128/before-nbfm-session.json`
on each Pi. The pi4g call-button controller runs from the normal
`build/radio-backend/` path; passive live status is captured separately from
the audio stream.

Jan confirmed: **“Voice and TX/RX switching both work.”** The live capture
contains four accepted TX clicks and four accepted RX clicks on pi4g. No
microphone capture errors/overruns, RF writer errors, driver underrun periods,
RX reader errors, playback underruns or clipped receive samples were reported
during the recorded test. Both radios finished in RX and the pi4g call-button
controller remains connected and ready. See `live-f-summary.json`,
`live-g-summary.json` and their compressed passive status recordings.
Physical unplug-during-TX and broader operational testing remain future
checks; disconnect cleanup was exercised against the dry-run backend.
