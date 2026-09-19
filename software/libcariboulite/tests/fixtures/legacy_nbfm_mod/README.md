# Numerical reference for step 6

The C source and header are unchanged copies from accepted step-5 commit
`185086f`. Keep them frozen as the independent waveform oracle for
`test_nbfm_mod.py`; they are not application build inputs. The test renames the
old symbols to compare both implementations in one binary, at both RF rates,
with interpolation enabled/disabled, pre-emphasis, clipping, silence and underrun.
