# AM voice link validation — 15 November 2026

AM voice TX and offset-tuned AM RX were tested at 430.100 MHz through the
operator-confirmed 90 dB direct link, with pi4g-trixie transmitting and
pi4f-trixie receiving. The production implementation is commit `94ead44`.
The default AM depth is 0.8 and additional digital TX backoff is 6 dB.

## Audible artifact and correction

The operator heard a wobble or extra beat in received AM audio. An independent
RSP2pro receiving the same transmission sounded clean. Playing a generated tone
and recorded received AM through the Jabra isolated the artifact to the received
recording: only recorded AM contained it.

Raw received IQ showed the wanted carrier approximately 35 Hz from nominal,
plus a separate component near 1.66 kHz, approximately 30 dB below the carrier.
Moving the RX center moved the wanted carrier while that component remained
near its original baseband frequency. It was approximately 39 dB weaker with
TX stopped. This supports a TX-associated fixed-baseband interference mechanism;
the physical coupling source has not been established. The earlier interpretation
of a 1.7 kHz carrier frequency error was incorrect.

The backend now tunes AM RX 12.5 kHz away from the requested RF frequency and
translates the wanted signal digitally before channel filtering. The requested
and displayed channel stay unchanged. Band boundaries and IQ polarity determine
the direction; existing FM processing is retained. The operator confirmed:
“Clean now; artifact gone.” A diagnostic toggle is available as
`--rx-am-offset off`; the default is `on`.

## TX linearity and audio checks

A digital backoff comparison motivated the 6 dB default. With a 1 kHz tone,
0.5 source amplitude, depth 0.8 and power setting 14 dBm, the earlier diagnostic
receiver configuration measured approximately 11.6% second/third harmonic ratio
at zero backoff and 0.64% at 6 dB. These are end-to-end audio measurements,
not calibrated RF spectral or modulation-depth measurements.

The final production check uses the normal requested frequency, default offset
RX, and default backoff. It exercises five power settings, all eight native TX
rates with RX at 1 MS/s, all eight RX rates with TX at 1 MS/s, and four AM depths.
Each case captures three seconds of 48 kHz mono PCM. Measurements use the middle
two seconds, fit the actual tone frequency, and report tone-to-residual ratio,
second/third harmonic ratio, clipping, and short low-level windows. This is
not the NBFM SINAD estimator or a standardized SINAD instrument.

All 25 cases completed with zero stream-counter increments. Native-rate tone
frequency was 999.50–999.62 Hz, tone-to-residual ratio 32.73–33.86 dB, and
second/third harmonic ratio 0.73–0.84%. Native-rate PCM level varied by 0.02 dB.
No native-rate low-level 10 ms windows or PCM clipping were detected. A separate
ten-second speech capture through the microphone path also had zero stream
errors and no PCM clipping. Both radios were released idle after testing.

Results and exact counter observation windows are stored in
[the baseline directory](baselines/20261115-am-tx-link/README.md).

## Qualification limits

Small modulation depths still show substantial end-to-end harmonic distortion
in this RF setup, despite clean synthetic DSP tests. This remains an open
linearity investigation; the offset correction addresses the extra beat rather
than proving RF modulation accuracy. The depth sweep records this limitation.

The 2.45 GHz exploratory test had insufficient received signal even at the
highest tested power setting, so it does not qualify operation in that band.
Hardware conclusions here apply to 430.100 MHz. Software tests cover the tuning
policy at supported band/conversion boundaries.

TX power settings are not calibrated AM carrier power or peak envelope power.
RF occupied bandwidth, calibrated AM depth and RF distortion remain unmeasured.
Speech was injected through the backend microphone/capture path and recovered
through AM RX; subjective speech assessment is recorded separately when available.

## Software checks

The complete backend/controller suite passed with 105 Python tests and its C
DSP, stream, lifecycle and pipeline checks. The monitor suite passed 64 tests.
Strict and production DSP builds passed all eight native clock tests, including
both AM RX offset signs, split/reset continuity and injected interference.
CMake application builds passed. Historical software summaries are retained in
this baseline directory. Temporary diagnostic IQ/IF builds were not deployed as
production binaries.
