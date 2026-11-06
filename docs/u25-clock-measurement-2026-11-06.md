# U25 measurement through a divided PMOD clock — 2026-11-06

A temporary FPGA image divides the clock arriving directly from U25 by 32 and
drives one PMOD pin. This brings either the expected 125 MHz clock or a possible
37.1 MHz clock comfortably within the user's 50 MHz oscilloscope bandwidth.
The measurement establishes clock frequency without inferring it from radio
operation or from the Pi's separate SMI peripheral clock.

## Diagnostic status and implementation

The isolated source and build artifacts are under
`build/u25-clock-probe-20261106/`. The image retains the current radio RTL and
adds a free-running five-bit counter on `i_glob_clock`; counter bit 4 drives
`io_pmod_out[3]` through an always-enabled, combinational `SB_IO` with
`PIN_TYPE=6'b011001`. It counts U25 directly, before the normal system-clock
divide-by-two stage. It runs independently of modem clocks and soft reset.
Other reserved PMOD outputs retain their input-only/no-pull-up configuration.

Simulation passed the divide-ratio, reset-independence and other-PMOD-pad
high-impedance checks at 125, 37.125 and 37.1 MHz input. All five routed clock
constraints passed, including 287.44 MHz against the 125 MHz input constraint,
74.90 MHz against the 62.5 MHz system constraint and 65.70 MHz against the
64 MHz LVDS constraint. Routed and ASC configuration checks confirm that only
J6 pin 7 changes from a reserved input to the clock output.

The image was programmed successfully on 2026-11-06 at 21:58:28 UTC. CDONE,
five SPI identity reads and low-power frontend readback passed. The loader
released its hardware claim, leaving the diagnostic image loaded for probing.
The measurement and subsequent restoration are recorded below. Image SHA-256:
`b17ab3901c0c89dc09fd34d7140662c35e977b7e0fe0908d0e7fbff19a448cbb`.

The [exact diagnostic image](../firmware/diagnostics/u25-clock-probe.bin) and
[measurement/provenance record](baselines/20261106-u25/clock-measurement.json)
are versioned for the pending reference-board comparison. Reuse this binary
without rebuilding it. [Helper build instructions](../firmware/diagnostics/README.md)
preserve the checked loader and restoration procedure outside ignored build files.

The source change is retained as a [staging patch](../firmware/diagnostics/u25-clock-probe.patch)
and its [simulation testbench](../firmware/diagnostics/tb_u25_clock_probe.v).
Apply the patch with `patch -p1` only in an isolated copy of the firmware
sources. The production RTL, pin constraints and packaged image were not edited.

The user confirmed that PMOD is disconnected except for the oscilloscope.
Keep external PA/sequencer circuitry disconnected for this measurement.

## Probe connections

The rev2 [schematic, page 4](../hardware/rev2/schematics/CaribouLite.PDF) and
[pin constraints](../firmware/io.pcf) establish these connections:

| Connection | J6 pin | Net / FPGA pin |
| --- | ---: | --- |
| Oscilloscope probe tip | **7** | PMOD_IO4 / `io_pmod_out[3]` / B21 |
| Oscilloscope ground | **9 or 10** | GND |
| Supply, not a signal probe point | 11, 12 | 3.3 V |

Establish connector orientation from the board's pin-1 indication or square
pad. The schematic symbol does not define a physical viewing direction; do
not identify the pin from an assumed generic PMOD layout. Pin 7 is on the
same odd-numbered row as pins 1, 3, 5, 9 and 11.

Use a ×10 probe and the scope's high-impedance input. The FPGA output is
3.3 V logic. A short ground connection helps preserve the waveform. A starting
timebase around 200 ns/div should display either candidate frequency clearly.
Measure a complete rising-edge-to-rising-edge period, or use the scope's
frequency measurement. Edge shape is not the quantity being qualified here.

## Expected readings

| Actual U25 clock | PMOD frequency, U25 / 32 | PMOD period |
| --- | ---: | ---: |
| 125 MHz | **3.90625 MHz** | **256 ns** |
| 37.125 MHz | **1.16015625 MHz** | approximately **862 ns** |
| 37.1 MHz | **1.159375 MHz** | approximately **863 ns** |

Calculate `U25 frequency = 32 × measured PMOD frequency`, or
`U25 frequency = 32 / measured PMOD period`. Record the measured frequency or
period and the scope's timebase/probe settings. The `37.1` package marking and
the earlier [TX timing response](u25-tx-timing-diagnosis-2026-11-06.md) motivate
this check; neither is a direct frequency measurement.

## Measured result

The user measured **1.1602 MHz** on the divided output on 2026-11-06.
Multiplying by 32 gives **37.1264 MHz** at U25 and approximately **18.5632 MHz**
at the normal divide-by-two system clock. This establishes an approximately
**37.125 MHz clock on this board**, rather than the intended 125 MHz.

Nominal 37.125 MHz divided by 32 is 1.16015625 MHz, which rounds to the
reported display reading. The apparent 1.4 kHz difference from 37.125 MHz
does not establish oscillator error: scope timebase accuracy, display rounding
and measurement settings are unspecified. Exact fitted part identity and the
other four boards remain unverified. Raw arithmetic and provenance are saved
in `build/u25-clock-probe-20261106/measurement.json`.

The original image was restored successfully at **22:11:16 UTC** on
2026-11-06. CDONE, all five SPI identity reads and low-power frontend readback
passed; no radio pipeline was started. The diagnostic PMOD output is no longer
configured. The log and receipt are `restoration.log` and `restoration.json`
under the same build directory.

## Restore before radio use

Stop radio applications before programming or restoring the FPGA. The checked
loader claims `/dev/smi`, validates its programming result and retains a
verified copy of the original image. Restore from the repository root with:

```sh
python3 firmware/diagnostics/program_image.py --restore
```

Restoration requires the normal hardware permissions. The script records a
new JSON receipt and log. Confirm successful completion before RF testing.
The original image SHA-256 is
`dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.

Normal backend startup can retain an already operational diagnostic image;
starting the backend is therefore not a substitute for this explicit restore.
The diagnostic does not replace `firmware/top.bin`, EEPROM contents, boot
configuration or installed applications. Its ignored build artifacts and
programming receipts are local; preserve them separately if retaining the
complete measurement record.
