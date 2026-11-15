# AM DAC correction comparison and implementation baseline

See [the report](../../am-dac-candidates-2026-11-15.md). The first independent
change is `368dd5d`, preserving the low I bit. The opt-in, duration-weighted AM
DAC shaper is `83b2f6d`. It is selected with `--tx-am-dac-shaping on`, defaults
off, and is reported by the passive monitor. It does not implement compression,
aviation-specific processing, SSB, frequency shifting or IQ calibration.

## Recorded experiments

Every CSV contains settings, actual TX calibration registers, metrics, measured
stream-counter deltas and a SHA-256 for its raw-IQ file. `*-metadata.json` retains
host, commit, executable hash, kernel/driver and launch flags. Prototype globals
are recorded in `parameters_json`; they are not the production startup flag.

| CSV | Captures | Purpose |
| --- | ---: | --- |
| `initial.csv` | 20 | Baseline and fast/slow shaping variants |
| `continuation.csv` | 8 | Complete selected second-order and uncalibrated shifted cases |
| `calibration.csv` | 94 | Keyed hardware/DC/gain/phase calibration search and repeats |
| `shaper-clock.csv` | 18 | 20–160 kHz shaping-clock comparison |
| `comparison.csv` | 36 | Raw, calibrated shifted and shaped AM; three TX rates/four amplitudes |
| `meter-gain42.csv` | 6 | Reduced RX gain |
| `meter-if37p5.csv` | 6 | Different RX offset |
| `wide.csv` | 6 | 4 MS/s RX, 500 kHz analog profile and wider spectral observation |
| `stair.csv` | 4 | Two raw and two shaped interleaved-reference captures |
| `production-unweighted.csv` | 53 | Initial integrated build, eight rates and backoff/power checks |
| `production-weighted.csv` | 28 | Final duration-weighted build, eight rates/fractional/default-backoff checks |
| `silence-off.csv`, `silence-on.csv` | 2 each | Unmodulated-carrier AF noise with production shaping off/on |

`manifest.json` maps names to original timestamped directories. Its listening
entry retains the operator's answers; `final-listening.json` records the final
integrated-build pass. `calibration.json` stores the selected experimental values
and initial/final repeats. These values are specific to the measured TX unit,
frequency and power; they are not installed as defaults.

All retained captured windows have zero measured stream-error deltas and no
ADC rail samples. Calibration trials intentionally include poor RF settings;
those are search observations, not waveform-quality acceptance cases. No claim
of indefinite-duration reliability follows from the short capture windows.

Raw RF/audio remains in `/tmp/<run>` on pi4f, with selected working copies on
pi4g. The approved private speech recording is not included. No new RSP2pro
recording/listening result is part of this baseline.

## Measurement definitions and limits

`metrics.py` operates on actual signed-16-bit IQ captures with the declared
native RX rate. It finds the strongest carrier near the selected IF, rather
than assuming the nominal oscillator frequency. RF offset-band values integrate
±100 Hz around each offset and reference the ±100 Hz carrier power. Broader
integrals cover the stated 3.5–60 or 3.5–200 kHz regions, with/without a narrow
RX-DC exclusion. The receiver's response, noise, images and nonlinearities remain
in these observations; no complete calibrated emission mask is established.

Envelope analysis uses a ±6 kHz FFT band around the measured carrier, takes
magnitude, normalizes by the mean, and selects 300–3300 Hz. After trimming 10 ms
at each end, it fits 1/2/3 kHz components. Harmonic ratio includes the second and
third only. `inband_tone_residual_db` is not standardized SINAD. When a fundamental
is nearly absent or near the noise floor, harmonic percentages are not useful
precision measurements. `inband_envelope_power_dbc` is the normalized envelope's
in-band power and is useful for the silent-carrier comparison.

`stair-analysis.csv/json` and `analyze_stair.py` retain an independent transfer
analysis. Each 2 ms target is normalized to adjacent reference levels, using
only the middle 1 ms. Correlation aligns the known repeating waveform. The raw
plateaus and shaped, nearly linear response are therefore measured independently
of the audio AGC and speaker. The duration-weighted production correction does
not change the intended constant-interval 1 MS/s transfer principle.

## Prototype source and reproduction

The diagnostic executables were built against objects from commit `368dd5d`,
before the production option was added. Use an isolated checkout/build of that
revision; retain the current working radio installation. The `candidate-*.patch`
files and `diag.h` describe the final prototype sources. Their base filenames
are temporary copies:

| Temporary name | Base file at `368dd5d` |
| --- | --- |
| `mod.c` | `software/libcariboulite/src/nbfm_mod.c` |
| `main.c` | `software/libcariboulite/tools/radio_backend/main.c` |
| `rx.c` | `software/libcariboulite/src/rx_pipeline.c` |
| `rx37500-am.c` | `software/libcariboulite/src/am_demod.c` |
| `rx40-profile.c`, `rx-wide-profile.c` | `software/libcariboulite/src/radio_profile.c` |

Extract those files from Git into `/tmp/am-candidate`, apply the corresponding
patches with `patch -p1` in that directory, and copy `diag.h` there. The historical
`rx40-profile.c` name was retained, but its qualified offset is 37.5 kHz. Copy
patched `mod.c`/`main.c` to `stair-mod.c`/`stair-main.c` before applying the
staircase patches. The staircase executable adds diagnostic modes only.

Build the modulator with the production DSP flags (`-O3 -ffast-math
-fno-math-errno -funroll-loops`), and main with the normal backend flags. Link
against the base backend objects, replacing `nbfm_mod.o`/`main.o` for TX and
`rx_pipeline.o` for bounded IQ capture. The alternate RX offset also replaces
`radio_profile.o` and `am_demod.o`. Exclude stale duplicate modem-snapshot objects.
The usual SDK, ALSA, pthread and libm dependencies apply. These temporary
executables do not overwrite production binaries.

`helper.py` and `run_*.py` retain the exact fixture logic, role checks, case
settings and shutdown. They use fixed `/tmp` paths and explicit pi4g TX/pi4f RX
hostnames; adjust paths in an isolated reproduction. `production_helper.py`
selects the normal backend and passes the shaping startup flag. Its old prototype
`diag` operation is not used with the production backend.

The prototype main polls a trusted, atomically replaced parameter file while
keyed, applies checked/read-back TXCI/TXCQ values under the hardware lock, and
acknowledges each epoch atomically. Modulator parameters are copied between
frames; the fixture allows settling before measurement. This is a diagnostic
interface, not a new public control protocol. The RX fixture accepts a bounded
capture request and writes at most two million IQ pairs to tmpfs. Its blocking
writes completed without observed stream errors; it is not a production capture
implementation.

### Excluded attempts and corrected defects

- The first variant sweep was interrupted by an empty acknowledgment-file race.
  The fixture was fixed to publish acknowledgments atomically and tolerate an
  empty read. The retained initial CSV contains the 20 complete baseline/variant
  cases; the selected families were completed in the continuation run.
- A first 40 kHz RX-offset diagnostic was rejected before capture because the
  normal demodulator accepts only 0/±12.5 kHz. The qualified diagnostic extends
  the exact lookup-table shift to ±37.5 kHz. No production RX behavior changed.
- The first strict carrier-mean check exposed the 500 kS/s unequal-hold bias.
  `duration-reproduction.txt` retains it. The final shaper weights feedback by
  cell duration instead of relaxing the test. `weighted-shaper-tests.txt` and
  the final full test log retain the passing strict/production checks.

## Production validation

`software-validation.json` records the implementation and checks. The source
oracle is `software/libcariboulite/tests/test_am_dac_shaping.c`, driven by its
adjacent Python builder and included in the backend's `make test` target.
It checks all eight rates, tiny through overdriven input, grid/nonnegative output
bounds, carrier mean, fundamental, exact partitioning/reset, invalid setup, and
raw equivalence when disabled. Existing raw AM/NBFM and lifecycle regressions
remain part of that full target.

The current implementation uses exact integer 4 MHz/divider timing and stores
feedback in IQ-code × sample-duration units. It bounds output at 0..4032 and
clears feedback at saturation. This addresses coarse amplitude conversion; it
does not certify RF modulation depth, analog linearity or PA compression.

Both standalone backends and all CMake applications were rebuilt on both hosts.
`release.json` records the final idle/closed-device checks. The normal FPGA and
installed/loaded driver remain unchanged by this candidate comparison.
