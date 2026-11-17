# Board #4: LED1 pin, routed FPGA and circuit audit — 17 November 2026

No intended logical connection between LED1 and RX framing was found in the
existing configured design. The pin assignment agrees with the original
schematic. LED1 does source current into a 470-ohm/LED load, so an electrical
interaction remains possible. This audit does not identify the physical fault.
No FPGA image, radio settings, or board connections were changed.

## Establishing which routed design was inspected

`icepack firmware/top.asc` produces exactly `firmware/top.bin` and the
`cariboulite_firmware` array in the library source and local build. A read of the
same symbol in pi4f's deployed build library also matches. All are 32,220 bytes,
SHA-256 `dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6`.
This establishes the relevance of the saved routed design to the embedded image;
it is not readback verification of the volatile FPGA configuration.

The routed ASC was reconstructed with IceStorm `icebox_vlog`. Yosys `proc` then
lowered that reconstructed Verilog for graph analysis; no synthesis mapping,
place-and-route, or programming was performed. Conservative fanout tracing
crossed combinational and sequential cells, including control dependencies.
Only LED1 and SPI MISO were reachable from the LED1 state. No FIFO, SMI,
clock, reset or RX-sync named net was reachable. This rules out an intended
Boolean dependency in this reconstructed configuration, not analog interaction,
timing violations, or faulty silicon/assembly.

The old `nextpnr.log`/timing report was not treated as a verified timing report
for this image. The exact ASC and its reconstructed connectivity provide the
basis of the routing conclusions.

## Pins and logic

| Signal | Package pad | FPGA I/O site | Circuit function |
|---|---|---|---|
| LED1 | A39 | X10/Y17/io0 | Schematic net LED4, R51/D11 |
| LED0 | A38 | X11/Y17/io0 | Schematic net LED5, R52/D12 |
| Config input 0 | B29 | X10/Y17/io1 | Shares an I/O tile with LED1, separate pad |
| Reset | A43 | X7/Y17/io0 | Same I/O bank as LEDs |
| SMI SOE | B15 | X10/Y0/io0 | Read strobe, opposite FPGA edge |
| LVDS RX clock | A3 | X0/Y12/io0 | LVDS bank on left edge |

The PCF has no duplicated package-pad assignment. LED1 and its configuration
input tile partner have distinct pads. Current RTL assigns LED1 directly from
`led1_state`; its digital-control write updates that bit and its read returns
that bit. The old commented `o_led1 = channel` line in top.v is inactive.

In the actual routed image, LED1's state register is LC X8/Y16/3. It feeds the
SPI readback logic and a **combinational pass-through LUT at X9/Y16/2**, then
the A39 output site. That extra LUT is a routing implementation detail: its
reconstructed function is identity, not a mux selecting RX/channel state.
The local state-net consumers are the readback LUT at X8/Y15/6 and this
pass-through LUT. Its local routing is in the upper portion of the fabric;
there is no logical branch into the RX data path. Routing adjacency alone
cannot establish or exclude physical coupling inside the chip/package.

## LED circuit and supply connections

Original Full schematic, page 4:

`U16 A39 / LED4 → R51 470 Ω → D11 LTST-C190KRKT → GND`

This is active-high: LED1=1 raises the output and powers the LED. It does not
sink LED current from a resistor tied to 3.3 V. LED0 uses the equivalent
R52/D12 circuit. LED current can be estimated from the measured voltage across
R51 divided by 470 ohms; its actual value was not measured in this audit.

| FPGA bank | Supply shown in schematic | Relevant signals |
|---|---|---|
| Bank 0 / top | VCC_FP_DIG_3V3 at A42 | LED1, LED0, reset, configuration |
| Bank 1 / right | VCC_FP_DIG_3V3 at B25 | U25 system-clock input A29, RF/PMOD/read-request |
| Bank 2 / bottom | VCC_FP_DIG_3V3 at A17 | SMI data and strobes, SPI |
| Bank 3 / left | VCC_FP_DIG_2V5 at B6 | LVDS data and clock |

Thus LED1 does **not** share its I/O bank with SMI or LVDS. The top/right/bottom
banks do share the named 3.3 V rail. U25, the 125 MHz FPGA system-clock source,
also receives that rail through ferrite L18, with C70/C71 decoupling after the
bead. The core uses the separate 1.2 V rail. All share board ground.
These are possible electrical paths of influence, not evidence that voltage
or clock integrity actually changes when LED1 is enabled. Decoupling labels
C72, C119 and C73 apply to the 3.3 V I/O supply group; C120 to 2.5 V.

## What this narrows, and what it does not

The [earlier four-state test](board4-backend-led-control-2026-11-16.md) matters:
LED0=1/LED1=0 works, while LED0=0/LED1=1 fails. Both LED circuits have the same
nominal resistor/LED specification and occupy the same bank. Therefore a generic
explanation based only on adding one healthy LED's total current is inadequate.
An LED1-specific load, leakage, pin/assembly fault, local supply/ground weakness,
or a marginal path affected differently by that particular output remains
possible. The original reference board also worked with LED1 high.

There is no schematic-design short or mux tying LED4 to SMI or LVDS. That does
not exclude an unintended physical connection on board #4. The next strongest
physical discrimination is temporarily opening **R51** (with the board powered
off), then repeating the established low/high/low software test. It separates
external LED-branch current from driving A39 high. No such modification has
been performed or assumed authorized by this inspection request. Measuring
R51 voltage drop and comparing R52 is a useful preceding static check.

## Scope and evidence

The locally available circuit source is the original
[CaribouLite Full schematic PDF](../hardware/rev2/schematics/CaribouLite.PDF),
page 4, with original Gerber and placement exports. There is no editable KiCad
schematic/board in this checkout; the user's KiCad conversion and actual
assembly deviations have not been independently checked. This audit does not
claim a net-aware PCB clearance/coupling analysis from the Gerbers or a verified
supply-impedance analysis. It neither confirms nor contradicts the user's
inspection that the LED4 PCB trace is remote from sensitive traces.

[Reproducible audit, hashes, pin map and routed excerpts](baselines/20261117-board4-led1-route-audit/).

## Disposition — 18 November 2026

The user chose to defer soldering and use the established workaround: keep
software LED1 (A39 / LED4 / D11, PCB label LED2) low before RX on board #4.
No load was disconnected and no automatic application startup behavior was
changed. The root cause remains unresolved.

The user also reported pre-programming voltages relative to ground:

| Resistor | FPGA side | LED side | Approximate current |
|---|---:|---:|---:|
| R51 / D11 / software LED1 | 1.198 V | 1.198 V | Below resolution of reported voltage drop |
| R52 / D12 / software LED0 | 1.618 V | 1.596 V | 47 µA, using 470 Ω |

D11 was dark and D12 weakly illuminated. Both LEDs behave normally when their
outputs are actively driven high/low. These user measurements were not made by
the diagnostic software. Weak pull-up variation and LED forward-voltage spread
remain a plausible explanation of the startup brightness difference; it is not
established as the cause of the RX dependency. No passing-board voltage
comparison was recorded.
