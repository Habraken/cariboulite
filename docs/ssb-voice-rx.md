# USB and LSB voice reception

Use the standalone backend and controller together. Select USB or LSB at
startup with `--rx-mode usb` or `--rx-mode lsb`, or use `rx mode usb` and
`rx mode lsb` while RX is stopped. RX and TX have separate mode settings.

For the attenuated HiF link, start the backend on the receiver with the
desired playback device. The example uses the Jabra USB speakerphone on
pi4f-trixie.

```bash
build/radio-backend/cariboulite_radio_backend \
  --channel RF24 --rx-mode usb --rx-frequency 430100000 \
  --rx-sample-rate 1000000 --rx-playback plughw:CARD=USB,DEV=0
```

Connect the updated controller from another terminal and start RX.

```text
rx start
```

On pi4f-trixie, the matching backend and controller are available under
`build/radio-backend`. Restart both programs if they were running when the
binaries were updated.

SSB pitch depends on the difference between the local receive reference and
the transmitter's carrier reference. With a known 1 kHz tone, adjust the
frequency until the recovered audio is near 1 kHz. For example, to lower the
receive reference by 580 Hz from 430.100 MHz, enter the following commands.
The required adjustment depends on the two radios.

```text
rx stop
rx frequency 430.099420
rx start
```

The receiver selects the requested sideband in complex baseband and filters
audio for 300–3000 Hz voice. It offsets the hardware tuner by 12.5 kHz and
shifts the IQ back digitally to keep the hardware DC spur out of the voice
passband. RF sideband sense accounts for HiF spectral inversion. SSB shares
the existing volume control and carrier squelch. Its audio uses fixed IQ
scaling and the modem's RF AGC. FM noise squelch, de-emphasis and the NBFM
SINAD estimator are unavailable in SSB.

For weak signals, try `rx volume 12` for up to 12 dB of live playback gain.
If more gain is needed on the 90 dB bench link, restart the backend with
`--rx-gain 80000` for 20 dB of additional gain over the default of 8000.
Check the meter for clipping when moving to stronger signals. Tune against
a known tone on the current receiver, since a correction measured on the
RSP2pro does not transfer to pi4f-trixie.

## Validation

Synthetic IQ tests exercised all eight native sample rates with both RX modes,
including inverted IQ, offset tuning, an opposite-sideband tone and tuner DC.
The minimum measured opposite-tone rejection in PCM was 60.5 dB. Tests also
checked reset, bounded-output processing and identical samples across block
splits. This is a DSP measurement.

### Attenuated RF link

The original pi4g-trixie board transmitted into pi4f-trixie's HiF port
through 90 dB attenuation on 2026-11-19. Both radios used 1 MS/s and nominal
430.100 MHz. TX used the −12 dBm setting with a 1 kHz, 0.5-amplitude tone.
Three seconds of 48 kHz PCM were captured per mode combination using ALSA
Loopback with calibrated playback at 0 dB.

| TX | Matching RX peak | Peak in opposite RX mode | Difference |
| --- | ---: | ---: | ---: |
| USB | 1107.9 Hz at −56.5 dBFS | 898.0 Hz at −96.2 dBFS | 39.6 dB |
| LSB | 906.9 Hz at −55.0 dBFS | 1096.2 Hz at −96.4 dBFS | 41.4 dB |

TX-off audio measured about −80 dBFS RMS for both modes. The opposed-mode
comparison includes the transmitting and receiving radios together. The
capture does not isolate TX image leakage from RX rejection. No RX queue
drops, read errors or clipped audio samples were reported. Logs and captures
are in `/tmp/ssb-rx-link-20261119T223327Z`.

In the follow-up, an RX request of 430.100102 MHz brought the recovered tones
to 967.9 Hz in USB and 1034.4 Hz in LSB. Receiver volume was +12 dB and
matching-mode audio was about −45 dBFS RMS. The correction is specific to this
radio pair and session. Follow-up logs and captures are in
`/tmp/ssb-rx-fine-tune-20261119T223500Z`. Both backends were stopped after
the captures.

The operator subsequently confirmed USB reception on pi4f-trixie after the
receiver tuning and gain guidance above. No new calibrated audio level was
reported for that listening check.
