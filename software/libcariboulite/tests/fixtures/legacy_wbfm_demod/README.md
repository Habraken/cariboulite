# Frozen FM DSP reference for step 1

`nbfm_demod.c` and `.h` are unchanged copies from commit
`e246267f93d249902d3153ff69576bf3936436c8`. Keep them frozen; they are test
inputs, never application build inputs. The combined implementation preserves
WBFM and its shared NBFM operations before the mode/file extraction.
`oracle.c` renames public symbols and compiles in a separate translation unit.

`test_fm_reference.py` compares exact PCM, raw taps and per-call progress at
both RF rates, with varied chunks/capacities, resets, clock corrections and
audio-control changes. Existing NBFM worker and signal-quality tests remain
independent acceptance checks. No hardware or ALSA is required.
