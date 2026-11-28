# Native CW receive test and level reference — 28 November 2026

## Calibrated listening setup

Pi4f transmits at 430.100000 MHz; pi4g receives at **430.100812 MHz** to
compensate the approximately +812 Hz relative carrier offset measured in
decoded IQ. This is a correction for this pair during this test, not an
absolute frequency calibration of either board. The nominal 90 dB attenuator,
additional 2 m RG179 cable and approximately 2 m physical separation remain
the bench setup. Requested TX power remains −12 dBm (reported PA step −11).

Pi4g uses a 500 Hz CW filter, 600 Hz listening pitch, **matched CW trim
+35.5 dB**, base gain 8000 and **0 dB RX volume**, feeding the H650e headset.
Pi4f retains the Jabra for local sidetone. Its keyer is restored from the
temporary straight-key calibration setting to **Iambic, 12 WPM, swapped
paddles, 250 ms hang and 5 ms ramps**.

The complete 120-million-sample, 1 MS/s raw IQ recording was replayed offline
through the production-flag CW demodulator with a digital −812 Hz shift to
emulate centring the receiver. The initial 100 ms was excluded. Complete
100 ms key-down windows were selected above −74 dBFS neutral RMS with a crest
factor below 3.8 dB, excluding noise, silent gaps and partial elements.
At +35.5 dB trim and 0 dB volume:

| Result | Value |
| --- | ---: |
| Selected steady windows | 152 |
| Median keyed PCM RMS | −24.022 dBFS |
| 5th–95th percentile | −24.339 to −23.980 dBFS |
| Highest selected window peak | −20.570 dBFS |

These are **recorded-IQ replay measurements**, not a fresh live key-down
measurement after the receiver restart. They establish the fixed gain for
this link direction and signal level; weaker/stronger signals still produce
weaker/stronger audio. The reverse direction and other frequencies/rates
have not been calibrated. No change was made to the other matched-mode trims.

[`recorded-level.json`](recorded-level.json) includes the raw-recording
SHA-256 and each selected window. [`measure_cw.py`](measure_cw.py) reproduces
the replay with numpy and a shared library compiled from `nbfm_demod.c`,
`nbfm_demod_dsp.c`, `wbfm_demod.c` and `am_demod.c` using `-O3 -ffast-math`.
The full raw file remains on pi4g at
`build/cw-link-20261128/cw-level-iq.bin` and is not added to Git.
The temporary bounded IQ-recording preload is removed from the final receiver
session. `rx-calibrated-setup.json` records the resulting startup and status.
The calibrated receiver restarted successfully without the preload, with
zero reader errors, zero playback underruns and zero clipped samples in the
initial four-second check (idle channel, not a keyed level validation).
The isolated build, 28 backend integration tests including CW matched-gain
mode switching, and the CW backend integration scenario passed. Final keyer
status is recorded in `tx-restored-keyer.json`.

## Live listening confirmation

After the calibrated receiver restart, Jan confirmed: “Yes, pitch and volume
are comfortable now.” The passive live meter recording also supports the
level match: 120 distinct RMS/peak pairs selected above −40 dBFS with crest
below 4.5 dB had a median RMS of **−24.115 dBFS** (range −25.273 to
−24.038 dBFS). No clipped samples were reported during the recording.
Key-down was inferred from level/crest, not synchronized paddle contacts;
the samples were deduplicated by RMS/peak pair and are not an exact count of
independent meter windows. See `live-listening-summary.json` and
`live-listening-levels.jsonl.gz` for the summary and full passive recording.
The current 500 Hz filter, 600 Hz pitch and matched gain were retained.

## Initial setup and diagnosis

Pi4f (192.168.6.163), reconstructed board #2, retains the Seeed USB paddle
interface and transmits native CW at 430.100 MHz, requested -12 dBm. Settings:
Iambic 12 WPM, swapped paddles, 250 ms hang and 5 ms ramps. A Jabra SPEAK 510
USB speaker (ALSA CARD=USB) supplies local sidetone after the H650e was moved.

Pi4g (192.168.6.77), original reference board, runs a newly built isolated
backend from build/cw-development-20261128/source/build/radio-backend. It is
receiving native CW at 430.100 MHz, 1 MS/s, with a 500 Hz channel and 600 Hz
listening pitch, using the H650e headset. Its simulated CW backend test passed.
Installed application paths were not replaced.

At neutral receive gain, passive measurements showed approximately -94.7 to
-85.8 dBFS audio RMS while RF RSSI ranged from -127 to -102 dBm. The receiver
was therefore restarted with cw=30 dB mode trim and +12 dB volume to make the
listening test audible. This is a test-specific gain adjustment, not a
calibration. Initial reader, alignment and playback counters showed no errors
or underruns. The operator subsequently confirmed audible received CW, with pitch described as a little high.

Control/status sockets on each Pi are under /tmp/cariboulite-cw-link-20261128.
Pi4f paddle-controlled RF TX is armed.

The receive-pitch setting on pi4g was lowered from 600 to 400 Hz following
that feedback, retaining the 500 Hz channel width, RF frequency and listening
gain. Actual audible pitch also includes the carrier offset; it was not
independently measured. Pi4f local sidetone was left unchanged.

The subsequent raw IQ recording located the wanted carrier near −11.69 kHz
instead of the expected −12.50 kHz offset-tuning position. Thus the wanted
carrier was roughly 812 Hz off centre, outside the 500 Hz full-width receive
filter. Increasing gain on that rejected signal also increased noise, while
the audible beat note remained high. Receiver-only fine tuning and restoring
600 Hz pitch resolved the filter/pitch setup before gain matching. The
`centered-level.json` sample contains no keyed windows and was not used as a
level reference.
