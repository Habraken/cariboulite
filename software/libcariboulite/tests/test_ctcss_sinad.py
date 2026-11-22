#!/usr/bin/env python3
"""Offline 1 kHz + CTCSS SINAD regression. No radio or ALSA devices.

Compare the existing raw-tap SINAD estimate with identical tone-removal
filtering for every case. The 0.4 amplitude avoids clipping; 0.95 exercises
voice-headroom clipping. IQ/data artifacts exist only in temporary storage.
"""
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
src = here.parent / "src"
modules = ("nbfm_mod.c", "nbfm_demod.c", "nbfm_demod_dsp.c", "wbfm_demod.c", "am_demod.c")
with tempfile.TemporaryDirectory(prefix="ctcss-sinad-") as directory:
    for policy, flags in (("strict", []), ("production", ["-ffast-math", "-fno-math-errno", "-funroll-loops"])):
        artifacts = Path(directory) / policy
        artifacts.mkdir()
        executable = artifacts / "test"
        subprocess.run(["cc", "-std=c11", "-D_GNU_SOURCE", "-O3", "-Wall", "-Wextra", "-Werror", *flags,
                        "-I" + str(src), str(here / "test_ctcss_sinad.c"),
                        *[str(src / module) for module in modules], "-lm", "-o", str(executable)], check=True)
        print("CTCSS SINAD " + policy, flush=True)
        subprocess.run([str(executable), str(artifacts)], check=True, timeout=180)
