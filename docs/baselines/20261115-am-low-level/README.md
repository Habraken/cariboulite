# Low-level AM diagnostics

See [the investigation report](../../am-low-level-distortion-2026-11-15.md).

The eight CSV files contain 46 RF cases and four software-only cases. Hardware
metadata identify source commit, backend hash, kernel, driver and launch flags.
Diagnostic binaries are isolated `/tmp` builds; metadata commit identifies their
source base, not a claim that their binary equals the production backend.
`*-diagnostic.patch` records their only source modifications. No diagnostic
patch was applied to the tracked production source or installed binary.

`run_low_level.py` runs the 20-case production baseline on pi4f. It uses
`/tmp/am-tx-link-helper.py` on both hosts, as documented in the preceding
[AM link baseline](../20261115-am-tx-link/README.md). It refuses an existing
backend owner, uses pi4g only as TX, and stops/unkeys both backends on exit.
Run only with the known attenuated wired link in place.

Other temporary runners used the same capture/counter logic with the diagnostic
binary selected via the helper's optional binary argument. Offset testing tuned
TX to 430.200 MHz to compensate the positive 100 kHz digital carrier shift on
the HiF conversion path. Phase diagnostics used `AM_DIAGNOSTIC_PHASE` set to
90, 180 or 270. Dither used a 16-step -120..120-count ramp at 1 MS/s; its RF
spectral effects are unqualified. Slower-timing tests changed only the TX write
setup/strobe/hold request and restored saved timing on stop.

`software_speech_loop.c` is the software-only modulator/demodulator harness:
mono float32 48 kHz audio on stdin, signed-16-bit 48 kHz audio on stdout. It uses
480-frame blocks and 1 MS/s IQ. Build against `nbfm_mod.c`, `nbfm_demod.c`,
`nbfm_demod_dsp.c`, `wbfm_demod.c`, and `am_demod.c`, with the SDK source include
path and `-lm`. The four software measurements used 3-second 1 kHz inputs at
the listed amplitudes. The first 0.5 second and last 0.5 second were excluded
from fitting, as in the RF analysis. The resampler's initial one-sample latency
was padded with zero only at the excluded end of the WAV.

Raw runs on pi4f:

- Baseline: `/tmp/am-low-level-20261115-191041`
- TX offset: `/tmp/am-if100-low-level-20261115-191319`
- Phases: `/tmp/am-phase90-20261115-191516`,
  `/tmp/am-phase180-20261115-191548`, `/tmp/am-phase270-20261115-191621`
- Slow SMI: `/tmp/am-slow-smi-20261115-191943`
- Dither: `/tmp/am-amplitude-dither-20261115-192429`

Repository CSV/JSON retain the measurements; `/tmp` WAVs and status streams are
not permanent artifacts. Personal source and microphone recordings are excluded.
