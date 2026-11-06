# Temporary U25 and TX timing diagnostics

These helpers preserve the 2026-11-06 clock investigation. They require the
repository's built `build/libcariboulite.so` and matching hardware permissions.
Compilation, simulation and `--check-only` do not access the board. Programming
occurs only when `program_image.py` is explicitly invoked without that option.

## Build the host helpers

From the repository root:

```sh
mkdir -p build/u25-clock-probe-20261106 build/u25-tx-diagnosis-20261106
cc -std=gnu11 -O2 -Wall -Wextra -Werror -Wno-missing-braces \
  -Isoftware/libcariboulite/src firmware/diagnostics/load_checked.c \
  -Lbuild -Wl,-rpath,"$PWD/build" -lcariboulite -lpthread -lm \
  -o build/u25-clock-probe-20261106/load_checked
cc -shared -fPIC -O2 -Wall -Wextra -Werror -Idriver \
  firmware/diagnostics/tx_timing_probe.c -ldl \
  -o build/u25-tx-diagnosis-20261106/tx_timing_probe.so
```

The preload is scoped to this Linux/aarch64 backend. It forwards the backend's
ioctl arguments, logs FPGA sample-gap reads, and optionally replaces only the
TX `1/4/2` write tuple through `CARIBOULITE_TX_WRITE_TIMING=setup,strobe,hold`.
The caller's expected tuple is updated, retaining actual readback verification;
normal initialization and timing restoration pass through. See the
[TX timing results](../../docs/u25-tx-timing-diagnosis-2026-11-06.md) for the
tested 1 MS/s workaround and its limits.

## Check, load and restore the clock image

First inspect inputs without hardware access:

```sh
python3 firmware/diagnostics/program_image.py \
  firmware/diagnostics/u25-clock-probe.bin \
  --sha256 b17ab3901c0c89dc09fd34d7140662c35e977b7e0fe0908d0e7fbff19a448cbb \
  --check-only
```

For the physical test, stop radio applications and disconnect other PMOD
circuitry, then explicitly run the same command **without `--check-only`**.
The loader claims `/dev/smi` before GPIO/SPI, holds the modem/mixer in reset
during configuration, and checks CDONE, SPI identity and frontend low-power
mode. It loads the supplied file even when an FPGA image is already running.
Ordinary programming/verification failure triggers restoration of the verified
`firmware/top.bin`; a loader timeout leaves hardware state unresolved in the
receipt and requires inspection before further use.

The output is U25 divided by 32 on **J6 pin 7**, with scope ground on **J6 pin 9
or 10**. Expect **3.90625 MHz** for 125 MHz U25, or **1.16015625 MHz** for nominal
37.125 MHz. Use a ×10 probe and high-impedance scope input; establish connector
orientation from the board's pin-1 mark. Full connections and the measured
result are in the [measurement procedure](../../docs/u25-clock-measurement-2026-11-06.md).

Restore the verified original image before radio operation:

```sh
python3 firmware/diagnostics/program_image.py --restore
```

Normal backend startup can retain the diagnostic image, so explicitly restore
it. The wrapper checks image hashes and writes new timestamped receipts/logs
under `build/u25-clock-probe-20261106/`; `--receipt PATH.json` selects a new
receipt in an existing directory. Existing receipt and log files are rejected
before hardware access. It does not modify EEPROM, packaged firmware or
installed applications. A successful explicit restore has `candidate_programmed`
true with the original image hash; `rollback_restored` records automatic
fallback only.

## Stage and simulate the source patch

The patch targets an isolated copy of the current production RTL. Keep the
versioned bitstream for the reference-board comparison instead of rebuilding
it. To check the source separately, from the repository root:

```sh
mkdir -p build/u25-clock-source-check
cp firmware/*.v firmware/io.pcf build/u25-clock-source-check/
patch -d build/u25-clock-source-check -p1 < firmware/diagnostics/u25-clock-probe.patch
iverilog -g2012 -s tb_clock_probe -I build/u25-clock-source-check \
  -o build/u25-clock-source-check/tb_clock_probe.vvp \
  /usr/local/share/yosys/ice40/cells_sim.v \
  build/u25-clock-source-check/top.v firmware/diagnostics/tb_u25_clock_probe.v
vvp build/u25-clock-source-check/tb_clock_probe.vvp
```

Adjust the Yosys `cells_sim.v` installation path when needed. The simulation
checks the divide ratio at three input periods, independence from soft reset,
and high impedance on the other reserved PMOD pins. No radio or FPGA loader is
started by this sequence.
