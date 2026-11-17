# LED1 pin, route and circuit audit

See [report](../../board4-led1-route-audit-2026-11-17.md).
audit.py verifies existing image equivalence and reconstructs the routed netlist
to trace conservative LED1 fanout. It does not run FPGA synthesis or place-and-route.
Requires icepack, icebox_vlog, and yosys already installed on this Pi.
The pin map given to icebox_vlog omits option syntax unsupported by its parser;
this affects naming only, not configuration reconstruction from ASC.
