# AM link baseline — 15 November 2026

See [the validation report](../../am-voice-tx-validation-2026-11-15.md) for
findings, audible comparisons, the fixed-baseband interference correction and
remaining qualification limits.

Production under test: `94ead44`, AM RX offset enabled, AM TX backoff 6 dB,
RF24/HiF 430.100 MHz, operator-confirmed 90 dB wired attenuation. pi4g-trixie
is always TX and pi4f-trixie RX. The native rate keys are divisors of 4 MHz:
1, 2, 3, 4, 5, 6, 8, A (A denotes hexadecimal 10).

## Files and measurement scope

- `audio-analysis.csv` and `.json`: all final tone cases. Harmonic ratio includes
  second and third harmonics only. Tone-to-residual includes noise/distortion
  after fitting a free-frequency sinusoid; it is not calibrated SINAD.
- `hardware-validation.json`: counter checks, native-rate summary and speech
  capture result. Snapshot observation windows cover complete recordings.
- `metadata.json`: exact backend hashes, source commit, kernel, driver and flags.
- `software-validation.json`: initial AM TX software checks.
- `offset-rx-software-validation.json`: final offset RX software checks.
- `radio_helper.py`: owns one backend, records status/control, and stops/unkeys
  in cleanup. It refuses to replace a running backend.
- `run_link_test.py`: five powers, eight TX rates, eight RX rates, four depths.
  Optional `--bypass-check` is exploratory; 2.45 GHz was not hardware-qualified.
- `run_linearity_test.py`: optional additional-backoff diagnostic using the
  current nominal frequency. Historical backoff results preceded the offset RX
  correction and are not directly interchangeable with this runner.
- `make_voice_reference.py` and `run_voice_test.py`: human speech from the Open Speech Repository
  through the TX microphone/capture path; create the reference on pi4g first.
- `analyze.py`: NumPy PCM analysis; no NumPy dependency in the backend itself.

All captures are 48 kHz mono signed 16-bit PCM via ALSA Loopback, with RX volume
−12 dB for tones and −6 dB for speech. Stream counters include RX IQ/audio queue
loss, read/alignment errors, playback underruns/write errors, TX writer errors
and driver underrun periods. The harness drains queued status and requires a
recent timestamp before comparing counters. Earlier diagnostic captures used
shorter counter observation windows and are excluded from the final validation.

## Reproduction

Stop controllers/backends first and confirm the attenuated RF link. Copy
`radio_helper.py` to `/tmp/am-tx-link-helper.py` on both hosts and the runner to
pi4f. Run it on pi4f with passwordless SSH to pi4g and the documented SSH known
host file available. Results are placed in `/tmp/am-tx-link-<timestamp>/` on
pi4f (RX WAVs/status and combined results) and pi4g (TX status). The helper uses
the production build at `build/radio-backend/cariboulite_radio_backend`.

Copy receiver results to a local folder and run `analyze.py FOLDER`. Raw
recordings/status remain in the timestamped `/tmp` directories; they are not
permanent repository artifacts. Repository CSV/JSON retain the final metrics.

Final raw tone run: `/tmp/am-tx-link-20261115-171134`; final speech run:
`/tmp/am-tx-voice-20261115-171433`. Native-rate tone cases had zero stream
errors, clipping or low-level windows. Low depth cases measured 25.44% and
16.61% second/third harmonic ratio at depth settings 0.25 and 0.5 respectively;
they are retained as a qualification limitation, not marked as linearity passes.
The 6 dB backoff comparison and raw IQ diagnostic described in the report
preceded the final correction; final metrics do not reuse their counter checks.

Speech listening failed after the original tone validation: source-versus-AM
comparison found distortion in recovered speech, and the independent RSP2pro
also sounded bad. See `speech-followup.json`. Historical speech results above
used synthesized speech and are transport checks only. `make_voice_reference.py`
now prepares a human-read Harvard-sentence sample, credits Open Speech Repository,
and verifies the original file hash. This replacement source must be auditioned
before a new RF speech qualification; no replacement RF pass is claimed here.

Final subjective speech result: **PASS** with the operator-supplied recording,
in separate CaribouLite/Jabra and RSP2pro passes at 430.100 MHz and 1 MS/s.
See `speech-followup.json` for exact settings and run identifiers. Earlier
source/audition failures remain historical results; low-modulation RF linearity
is still a qualification limitation.
