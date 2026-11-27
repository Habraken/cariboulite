UPDATED PACKAGE: PCBWay returned files are included under pcbway/.
Read pcbway/READ-FIRST.txt for changes and production-status limits.
The original KiCad project does not include the supplier CAM changes.

CaribouLite Full Rev 2.8 — Elect investigation package
Prepared 26 November 2026

Purpose: reference design data for investigation of the reproduction board
that has no FPGA communication.

START HERE
schematics/CaribouLite-Full-Rev2.8.pdf: original released Full schematic.
  Consult the FPGA and digital-interface sheets for the communication paths.
pcb/gerber/: original released Gerber files, unchanged.
pcb/ncdrill/: original drill programs and reports, unchanged.
pcb/stack/: original PCB stack-up spreadsheet.
assembly/: original Full BOM, placement list and assembly drawing.
kicad/CaribouLite/CaribouLite.kicad_pro: converted project (KiCad 10 format),
  with PCB, all seven schematic sheets and the local footprint library.
kicad/CONVERSION-AUDIT.md: existing conversion audit; read before using KiCad.

REFERENCE STATUS
The original released PDF schematic and PCB manufacturing outputs are the
reference design data. These files are not a measured as-built record of the
faulty board or a record of any subsequent assembly substitutions or rework.

The KiCad project is supplied for inspection. The existing audit documents
schematic/PCB connectivity discrepancies, including AT86 LVDS nets and power
net naming, and incomplete imported stack-up/routing rules. It is not a
validated source for regenerating manufacturing files or updating the PCB
from the schematic. The historical audit is included unchanged; its source
paths refer to the original workspace, not this ZIP. No new electrical audit
or design changes were performed for this package.

PROVENANCE
Original PDFs, PCB and assembly data: hardware/rev2 in the local CaribouLite
repository. KiCad files: recovered-kicad/rev2.8-full local working project.
All included design files are byte-for-byte copies. Backups, session settings,
private correspondence and unrelated software are excluded.
Hardware attribution: CaribouLabs / CaribouLite; hardware/README.md identifies
the hardware license as Creative Commons Attribution 4.0 International:
https://creativecommons.org/licenses/by/4.0/

SHA256SUMS lists checksums for every included file except itself.
