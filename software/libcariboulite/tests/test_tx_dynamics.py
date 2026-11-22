#!/usr/bin/env python3
"""Exercise the microphone compressor/limiter under strict and production math."""
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="tx-dynamics-") as directory:
    for policy, flags in [("strict", []), ("production", ["-ffast-math", "-fno-math-errno", "-funroll-loops"])]:
        exe = str(Path(directory) / policy)
        subprocess.run(["cc", "-std=c11", "-O3", "-Wall", "-Wextra", "-Werror", *flags,
                        "-I" + str(here.parent / "src"), str(here / "test_tx_dynamics.c"),
                        "-lm", "-o", exe], check=True)
        print("TX dynamics " + policy, flush=True)
        subprocess.run([exe], check=True, timeout=30)
