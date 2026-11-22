#!/usr/bin/env python3
"""Offline CTCSS DSP tests under strict and production math; no hardware."""
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="ctcss-") as directory:
    for policy, flags in [("strict", []), ("production", ["-ffast-math", "-fno-math-errno", "-funroll-loops"])]:
        exe = str(Path(directory) / policy)
        subprocess.run(["cc", "-std=c11", "-D_GNU_SOURCE", "-O3", "-Wall", "-Wextra", "-Werror", *flags,
                        "-I" + str(here.parent / "src"), str(here / "test_ctcss.c"),
                        *[str(here.parent / "src" / name) for name in ("nbfm_mod.c", "nbfm_demod.c", "nbfm_demod_dsp.c", "wbfm_demod.c", "am_demod.c")], "-lm", "-o", exe], check=True)
        print("CTCSS " + policy, flush=True)
        subprocess.run([exe], check=True, timeout=90)
