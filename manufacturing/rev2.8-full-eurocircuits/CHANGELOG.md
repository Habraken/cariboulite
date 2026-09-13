# Package change log

## 2026-09-12 — Initial technical quotation package

- Copied Full/shared production originals without changing file contents.
- Inventoried source paths, sizes and SHA-256 hashes; excluded ISM variant.
- Created separate PCB ZIP, BOM CSV and CPL CSV for quotation upload.
- Selected first explicitly listed original J1 MPN for quotation.
- Kept all other primary MPNs; flagged U25 and J7 source ambiguities.
- Excluded four schematic-confirmed DNP placements and bare test point TP13.
- Aligned placement coordinates to Gerbers by verified common translation,
  converted mil to mm, normalized 360° to 0°, and corrected J1's centre to the
  centroid of its 40 drilled holes. Preserved source sides and other angles.
- Identified 1,397 blind-via holes omitted from the handover's feasibility summary.
- Corrected stack thickness interpretation: reported total includes mask.
- Documented shielding, manufacturing and assembly release holds.
- Added deterministic generation and validation outputs. No PCB design change,
  supplier contact, upload, procurement or manufacturing release performed.
