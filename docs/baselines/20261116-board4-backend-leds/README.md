# Explicit backend LED control and state matrix

See the [report](../../board4-backend-led-control-2026-11-16.md).
`summary.json` retains command outcomes and final alignment counters for each
trial. Raw events contain full controller stdout/stderr and status windows.
Scripts are the actual hardware experiment drivers; they invoke the built
controller, do not use the test app, and never start TX.

Full data archives on pi4g:

- `build/board4-backend-leds-20261116/raw.tar.gz`: initial isolation, exact-write
  pattern and per-state matrix on #4, copied from pi4f's persistent build folder.
- `build/reference-backend-leds-20261116/raw.tar.gz`: original-board comparison,
  including the excluded attempt with an unavailable audio card.

`raw-artifacts.sha256` hashes archives; `source.sha256` hashes code and scripts.
`release-check.json` verifies both radios released. Test output and installed
binary/library hashes are retained alongside these files. The optional FPGA
reply tail appears only on explicit LED commands. No automatic startup write,
firmware modification, RF measurement sweep or EEPROM change was made.
