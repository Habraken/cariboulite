#!/usr/bin/env python3
"""Measure SSB IQ with a strict oracle for both DSP build policies."""

from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
CFLAGS = [
    "cc", "-std=c11", "-D_GNU_SOURCE", "-O3", "-Wall", "-Wextra", "-Werror",
    "-Wno-unused-parameter", "-ffunction-sections", "-fdata-sections",
    f"-I{SRC}", f"-I{SRC / 'at86rf215'}", f"-I{SRC / 'rffc507x'}",
]

with tempfile.TemporaryDirectory(prefix="ssb-tx-") as directory:
    build = Path(directory)
    oracle = build / "oracle.o"
    subprocess.run([*CFLAGS, "-c", HERE / "test_ssb_mod.c", "-o", oracle], check=True)
    for policy, flags in [
        ("strict", []),
        ("production", ["-ffast-math", "-fno-math-errno", "-funroll-loops"]),
    ]:
        objects = [oracle]
        for source in ("nbfm_mod", "mod_worker"):
            obj = build / f"{policy}-{source}.o"
            subprocess.run(
                [*CFLAGS, *flags, "-c", SRC / f"{source}.c", "-o", obj], check=True
            )
            objects.append(obj)
        executable = build / policy
        subprocess.run(
            ["cc", *objects, "-Wl,--gc-sections", "-lm", "-pthread", "-o", executable],
            check=True,
        )
        print(f"Testing SSB TX {policy}", flush=True)
        subprocess.run([executable], check=True, timeout=120)
