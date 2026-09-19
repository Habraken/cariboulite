# Numerical reference for step 5

These two files are unchanged copies of `src/nbfm_demod.c` and `.h` from
commit `b3da533` (accepted step 4). Keep them frozen: they are the independent
before-extraction oracle for `test_nbfm_demod.py`, not application build inputs.
The test replaces FIFO I/O with deterministic inputs/depth observations and
captures PCM outputs. Symbol renames allow both old and new workers to run in
one binary. Time is fixed in that test to exclude nondeterministic heartbeat
reads from the clock-correction comparison. No hardware or ALSA library is used.
