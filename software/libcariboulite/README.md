# CaribouLite C/C++ and SoapySDR library

The build definition is [CMakeLists.txt](CMakeLists.txt). It requires C/C++
compilers, CMake 3.15 or newer, threads, ncurses and ALSA development files.
SoapySDR is detected optionally; without its development files the Soapy module
is skipped. The IIR source dependency must be present.

From the repository root, a userspace build is:

```sh
git submodule update --init --recursive
cmake -S software/libcariboulite -B build
cmake --build build --parallel 2
```

On Debian-family systems the dependency package names include `build-essential`,
`cmake`, `pkg-config`, `libncurses-dev`, `libasound2-dev`, and optionally
`libsoapysdr-dev`. A fresh OS installation has not been exercised by the
September 2026 documentation audit; see [DOC-01](../../ROADMAP.md#documentation-validation-backlog).

The build creates `libcariboulite.so`, `libcariboulite_static.a`, the test
application, utilities, and (when detected) the Soapy module. It does not build
or install the kernel driver; see [driver/README.md](../../driver/README.md).

Installation is a separate operation:

```sh
sudo cmake --install build
sudo ldconfig
```

Inspect installation paths before using a custom prefix: some subdirectories
have absolute install destinations. The test application has no active install
rule; run `./build/cariboulite_test_app` from the repository root so its
`firmware/top.bin` programming path resolves correctly.

Software regression tests live in [tests](tests/); hardware diagnostics also
exist under [test](test/). Building successfully does not verify radio operation.

## Hardware-free NBFM example

From the repository root with the build configured:

```sh
cmake --build build --target nbfm_memory_demo -j2
./build/nbfm_memory_demo
python3 software/libcariboulite/tests/test_memory_audio.py
```

The demo connects memory-backed audio to the standalone modulator and demodulator
at 2 and 4 MS/s, without opening audio or radio devices. The
[extension guide](../../docs/audio-dsp-extension-guide.md) also provides a direct
C11/libm build that does not require the application's ALSA dependencies.

# License
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.
