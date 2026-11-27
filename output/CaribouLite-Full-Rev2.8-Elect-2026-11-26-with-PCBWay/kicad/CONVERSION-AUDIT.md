# KiCad conversion audit — CaribouLite Rev 2.8 Full

- Audit date: 2026-09-17
- Host: macOS
- KiCad version: 10.0.6

## Conclusion

The conversion is a strong reconstruction of the physical PCB, but it is not a
lossless migration of the complete Altium/CircuitStudio design.

The converted PCB preserves the released board geometry well enough to be a
useful editable starting point. The converted schematic and project rules must
not yet be treated as authoritative: impedance constraints and stack-up details
were not preserved, and schematic-to-PCB netlist comparison found material
connectivity differences.

Retain the native source and released manufacturing files as the ground truth.

## Sources used for comparison

- Converted project: `CaribouLite/CaribouLite.kicad_pro`
- Native project snapshot:
  `../../rev2.8-native-source/originals/hardware_src/`
- Native Rev 2.x PCB:
  `../../rev2.8-native-source/originals/hardware_src/cariboulite_r2.CSPcbDoc`
- Released Rev 2.8 Gerbers and drill programs: `../../../hardware/rev2/pcb/`
- Released stack-up: `../../../hardware/rev2/pcb/stack/cariboulite_r2.8.xls`

## Confirmed as preserved

### Project and component structure

- All seven schematic sheets are present: the top level and six child sheets.
- The schematic and PCB exports contain the same 271 reference designators.
- All 843 schematic netlist pin records map to corresponding PCB pads.
- The three additional unnumbered PCB pad records are the NPTH mounting holes.
- The imported footprint library contains 31 footprint definitions.
- The PCB includes embedded 3D model data and rendered successfully in KiCad.

### PCB geometry

The converted PCB contains:

- Four copper layers: `F.Cu`, `In1.Cu`, `In2.Cu`, and `B.Cu`.
- 2,546 routed segments and 39 routed arcs.
- 83 copper zones.
- 2,204 vias: 807 through vias and 1,397 blind vias.
- 271 footprint/placement records.
- A 65.000 mm by 30.030 mm board outline.

Fresh Gerber and Excellon exports completed successfully for all four copper
layers and all three drill spans.

### Drill programs

The converted drill output reproduces all 2,253 holes in the released drill
report:

| Layer span | Diameter | Count | Result |
|---|---:|---:|---|
| Layer 1–4, via | 0.2032 mm | 805 | Match |
| Layer 1–4, plated component | 0.4600 mm | 4 | Match |
| Layer 1–4, plated component | 0.6000 mm | 2 | Match |
| Layer 1–4, via | 0.8000 mm | 2 | Match |
| Layer 1–4, plated component | 0.9000 mm | 40 | Match |
| Layer 1–4, NPTH | 2.7500 mm | 3 | Match |
| Layer 1–2, blind via | 0.1016 mm | 661 | Match |
| Layer 3–4, blind via | 0.1016 mm | 736 | Match |

After compensating for the different coordinate origin, all converted drill
locations matched the released Excellon locations within approximately 1.3 µm.
That residual is consistent with the different coordinate precision used by
the two exporters.

### Net-class membership

The conversion retained the native class names and memberships:

| Net class | Assigned nets |
|---|---:|
| `RF50OHM` | 60 |
| `LVDS` | 14 |
| `SMI` | 15 |

## Information not preserved correctly

### Impedance and routing constraints

The class names survived, but their electrical constraints did not. KiCad
assigned the same generic defaults to the imported classes:

- Track width: 0.20 mm
- Differential-pair width: 0.20 mm
- Differential-pair gap: 0.25 mm
- Clearance: 0.20 mm

This does not reproduce the native rules. Examples recovered from the native
PCB include approximately 0.1950 mm RF routing for the 50 ohm class and
0.1206/0.1016 mm width/gap geometry for the 100 ohm LVDS class.

Existing routed copper retains its imported physical widths, but KiCad will not
enforce the original constraints when traces are edited or rerouted. The target
impedances, tolerances, layer-specific widths, reference layers, and applicable
rule scopes must be rebuilt before design work.

### Stack-up

The KiCad PCB records only a total thickness of 1.58002 mm. It has no complete
stack-up definition for copper thicknesses, dielectric thicknesses, material,
dielectric constant, loss tangent, or finish.

Reconstruct the KiCad stack-up from the released spreadsheet and confirm the
final construction with the PCB manufacturer. Do not infer impedance from the
overall thickness alone.

### Schematic connectivity

A connectivity-partition comparison was made between the KiCad schematic XML
netlist and the PCB IPC-D-356 netlist. It found eleven material discrepancies:

- Ten AT86 receive/transmit nets are each continuous on the PCB but split into
  separate nets in the converted schematic:
  `AT86_RX_CLK_N`, `AT86_RX_CLK_P`, `AT86_RX24_N`, `AT86_RX24_P`,
  `AT86_RX09_N`, `AT86_RX09_P`, `AT86_TX_N`, `AT86_TX_P`,
  `AT86_TX_CLK_N`, and `AT86_TX_CLK_P`.
- The converted schematic combines PCB nets `RFFE_LNA_VCC` and `VCC_ANA` into
  one `VCC_3V3` net. On the PCB these nets remain separated through `L1`.

These differences must be repaired and reviewed against the native schematic,
not resolved by blindly updating the PCB from the converted schematic.

### ERC results

KiCad reported 1,834 ERC findings: 32 errors and 1,802 warnings.

| ERC category | Severity | Count |
|---|---|---:|
| Endpoint off grid | Warning | 1,252 |
| Library-symbol issues | Warning | 437 |
| Unconnected wire endpoint | Warning | 64 |
| Multiple net names | Warning | 23 |
| Input power pin not driven | Error | 13 |
| Net not a bus member | Warning | 10 |
| Isolated pin label | Warning | 10 |
| Unresolved variable | Error | 7 |
| Input pin not driven | Error | 6 |
| Pin not connected | Error | 6 |
| Dangling no-connect marker | Warning | 6 |

Many findings are conversion noise, especially the off-grid endpoints and
library warnings. They should nevertheless be triaged rather than globally
excluded because the audit also found real connectivity errors.

### Fonts and Altium-specific project data

KiCad substituted four fonts during export and rendering:

- `icomoon` with Verdana Regular
- `Alef` with Verdana Bold
- `Noto Serif` with Times New Roman Regular
- `BN Machine` with Verdana Bold

This affects visual fidelity of text but not electrical connectivity.

The converted directory also has no native KiCad equivalent for every
Altium-specific artifact, including the original output job and harness files.
Those files remain part of the required native-source archive.

## Checks that remain incomplete

- KiCad 10.0.6 aborted while running PCB DRC, both with and without schematic
  parity enabled. No complete PCB DRC report was produced. This is a validation
  blocker, not by itself proof of a PCB defect.
- Fresh Gerbers exported successfully, but a complete geometric XOR of every
  converted Gerber against every released Gerber has not yet been performed.
- Component values, manufacturer fields, variants, BOM population, and
  placement orientation have not yet received a field-by-field migration audit.

## Required work before making design changes

1. Rebuild and document the complete layer stack in KiCad.
2. Recreate the impedance, width, clearance, differential-pair, and via rules
   from the native PCB and manufacturer-approved stack-up.
3. Repair the eleven schematic/PCB connectivity discrepancies.
4. Triage ERC findings, starting with unconnected endpoints and multiple net
   names, then run schematic-to-PCB parity again.
5. Resolve or work around the KiCad DRC crash and obtain a complete DRC report.
6. Regenerate Gerber, drill, IPC netlist, BOM, and placement files and compare
   them against the released Rev 2.8 outputs before fabrication.

Until these steps are complete, use the converted project for inspection and
reconstruction only. Do not use it to regenerate manufacturing data or update
the PCB from the schematic without a controlled review.
