# USB and LSB voice transmission

The standalone backend accepts `--tx-mode usb` and `--tx-mode lsb`, or
`tx mode usb` and `tx mode lsb` in the controller while TX is stopped.
Both modes use the same tone and microphone sources, PTT lifecycle and eight
native sample rates as AM and NBFM. The controller and passive monitor show the
selected TX mode. Update the backend and controller together so the controller
can decode the USB and LSB mode names in v2 replies.

The modulator filters 48 kHz audio for 300–3000 Hz voice and uses a Hilbert FIR
with 4 ms of group delay to generate quadrature audio. The complex envelope
uses the existing IQ scale as its peak limit and interpolates I and Q to the
native TX clock. AM depth, AM backoff, DAC shaping and FM deviation apply to
their respective modes. SSB uses suppressed carrier, so silence produces zero
digital IQ after filter settling. The low HiF route uses the difference
`LO − IF`; TX conjugates SSB IQ for that route to compensate for the reversal
of frequency offsets.

## Attenuated bench check

Use the original CaribouLite on pi4g-trixie as TX and connect its HiF port to
the RSP2pro through the confirmed 90 dB attenuation. Use pi4f-trixie in the
receiver role. At the established 430.100 MHz bench frequency, start at
−12 dBm and 1 MS/s. Enter these commands in the controller attached to the
backend on pi4g-trixie.

```text
tx frequency 430.100
tx rate 1
tx power -12
tx mode usb
tx source tone
tx tone 1000 0.5
tx start
```

Keep that controller connected while keyed. On the RSP2pro spectrum, look for
the tone approximately 1 kHz above the actual TX center, allowing for receiver
and transmitter frequency error. Check the center and opposite side for
residual carrier and image. Run `tx stop`, select `tx mode lsb`, and repeat.
The tone should move approximately 1 kHz below the actual center. After the
tone check, use `tx source mic` and listen in the matching mode for speech
intelligibility and distortion. The existing start/stop cue tones are
transmitted in the selected sideband.

## Validation

Strict and production software builds were tested at 300 Hz, 1 kHz and 3 kHz
for both modes and all eight native TX rates. The minimum opposite-sideband
rejection in generated IQ was 60.7 dB. The software tests also cover carrier
suppression, mixer-route packet conjugation, fractional packet continuity,
delay-line reset, envelope bounds and audio underflow. The RSP2pro bench
observations below cover sideband placement, residual carrier and a microphone
pitch check.

On 2026-11-19, the original board on pi4g-trixie streamed a 1 kHz,
0.5-amplitude tone in both USB and LSB at 430.100 MHz, 1 MS/s and the −12 dBm
power setting. Each session completed 316 TX frames including startup and
shutdown padding, returned to IDLE and reported zero queue drops, worker
errors, writer errors and driver underrun periods. The transmitter was
unkeyed at the end. The raw snapshots and backend log are in
`/tmp/ssb-tx-20261119T214805Z/tx` on pi4g-trixie.

The coordinated SDRconnect repeat used the same settings for 20 seconds per
sideband and also returned to IDLE with zero queue drops, writer errors and
driver underrun periods. The RSP2pro is attached to a separate Pi 5 running
SDRconnect. The run on pi4g-trixie is in `/tmp/ssb-rsp-20261119T215244Z/tx`.

In a dedicated 30-second USB run at the same settings, the operator saw the
1 kHz tone peak above 430.100 MHz in SDRconnect, confirming USB placement on
the low HiF route. TX returned to IDLE with zero writer errors and driver
underrun periods. The run log is in `/tmp/ssb-usb-20261119T215712Z/tx`.

The operator heard intermittent audio during the same run and suspected
SDRconnect. Across 151 TX status snapshots spanning 30.07 seconds, the DMA
completion counter advanced at every interval, with zero queue drops, queue
timeouts, writer errors or driver underruns. This supports continuous host
streaming. The audio interruption remains unlocalized without a receiver-side
recording correlated with the TX log.

On the USB repeat, the operator set SDRconnect to 1 MS/s and heard a clear
tone estimated at 500 Hz. The spectrum showed peaks on both sides of the
430.100 MHz receiver setting. The transmitter sent a 1 kHz test tone.
Peak frequencies and relative levels were not recorded, so RF image
suppression remains undetermined. TX returned to IDLE with zero writer
errors and driver underrun periods. The run log is in
`/tmp/ssb-usb-20261119T215946Z/tx`.

With the microphone muted and SDRconnect tuned to 430.100 MHz in USB, the
operator observed a steady peak at 430.099420 MHz, 580 Hz below the receiver
tuning. If it is TX carrier leakage, a 1 kHz test tone should appear at
430.100420 MHz in USB or 430.098420 MHz in LSB. The TX-off check below confirmed that the peak follows TX keying.

The operator then tuned SDRconnect in USB to the muted peak at 430.099420 MHz
and reported that microphone speech returned to the correct pitch. That
supports using the peak as the carrier reference for this run. The residual
carrier level relative to the wanted sideband was not measured.

With a 1 kHz USB test tone at 0.4 audio amplitude, the operator measured
the residual carrier 10 dB below the wanted tone on the RSP2pro (−10 dBc
relative to this tone). This is a receiver spectrum reading at the stated
drive level. Opposite-sideband rejection on RF remains unmeasured.

With receiver gain fixed, the operator reported the carrier at −118 dBm
both with and without modulation. The 1 kHz tone read −108 on the same
spectrum, 10 dB above the carrier. The operator confirmed that the carrier
peak disappeared when TX was unkeyed.

The operator repeated the fixed-gain test in LSB with the same 1 kHz,
0.4-amplitude tone. The residual carrier again read −118 dBm and the wanted
tone read −109 on the same spectrum, giving −9 dBc relative to the LSB tone.
The tone was 1 dB lower than in USB. The opposite-sideband image level was
not measured.

The operator visually confirmed RF energy in the selected sideband in
SDRconnect for USB and LSB. This completes the visual sideband check on the
430.100 MHz attenuated bench link. RF opposite-sideband rejection remains
unquantified. The residual carrier measured −10 dBc in USB and −9 dBc in
LSB relative to the 1 kHz, 0.4-amplitude tones.
