# First physical USB CW practice test — 28 November 2026

Pi4f-trixie, 192.168.6.163, reconstructed/repaired CaribouLite #2 (UUID 36b67ac4-b313-4369-99bd-4b3e09caabfa). Seeed XIAO M0 USB key interface (2886:802f), serial E44256EE5154315432202020FF0B1C0B, evdev event4; Logitech H650e headset.

The isolated CW backend was started in real-device practice mode, with RX stopped and RF transmission disabled. The Seeed keyboard interface was exclusively grabbed. Settings: Iambic, 12 WPM PARIS, normal mapping ([ dit, ] dah), 600 Hz sidetone, 250 ms hang and 5 ms ramps. ALSA accepted mono S16_LE at 48 kHz, period 120 frames (2.5 ms), buffer 480 frames (10 ms).

The operator tried each paddle and a squeeze, and reported correct elements and prompt audio feedback. This is subjective functional acceptance, not an instrumented latency measurement. The passive status journal and final snapshot record practice enabled, no requested/running TX and no CW errors. RF envelope, actual switching timing, RF leakage and over-the-link CW/NBFM reception were not tested.

Backend PID 22305 was left running in practice mode. Control socket: /tmp/cariboulite-cw-practice-20261128/control.sock; root access is required. The bounded passive recorder was stopped after capturing the test. Production application paths were not replaced.

## Native CW transmit check

The operator subsequently confirmed that native CW TX works on reconstructed
board #2 / pi4f at 430.100 MHz. The session was explicitly switched out of
practice, retaining Iambic 12 WPM, swapped paddles, 5 ms ramps and 250 ms hang.
Requested TX power was -12 dBm. This is operator-reported functional RF
acceptance; power, envelope, spectral purity and latency were not independently
measured. See native-cw-user-confirmation.json for the passive status snapshot.
The running session remains RF-enabled, superseding the practice-only state
recorded earlier above.

The operator reports that TX/RX switching feels natural at 12 WPM with the
configured 250 ms hang. Retain 250 ms as the default. This is subjective
operating feedback at the tested speed, not a measured antenna switching
latency or acceptance across the full 5–30 WPM range.

## Keyed NBFM audio transmit check

The operator confirmed that CW tone transmission through NBFM also works on
pi4f / reconstructed board #2 at 430.100 MHz, and that TX/RX switching feels
natural at 12 WPM. Settings were Iambic 12 WPM, swapped paddles, 600 Hz tone,
amplitude 0.4, deviation setting 2500 Hz, requested power -12 dBm, 5 ms ramps
and 250 ms hang. See nbfm-user-confirmation.json for the passive snapshot.

Both native CW and keyed NBFM audio therefore have operator-reported functional
TX acceptance with the USB interface and H650e headset. Switching feedback is
scoped to 12 WPM; no instrumented latency, spectral or full-speed-range claim
is made. The running session remains RF-enabled in NBFM keying mode,
superseding the preceding native-CW state.
