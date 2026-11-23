# Repaired device #2 RX frequency and rate sweep — 23 November 2026

[Analysis report and plots](REPORT.md). [Reciprocal-direction comparison and historical board #1 reference](COMPARISON.md).

Repaired CaribouLite device #2 on **pi0b receives**, and the original reference on **pi0a transmits**, through the user-confirmed nominal 90 dB attenuated HiF/RF24 link. Requested and reported TX power is 0 dBm. The operator reports the correct 125 MHz oscillator is fitted; this run uses no oscillator timing workaround.

The operator moved the same Logitech H650e USB headset to pi0b for clocked RX playback.

All 172 frequencies × eight supported matched TX/RX rates completed. `results.csv` contains 1,376 matrix rows and eight preflight rows; select `phase=matrix` for the sweep. Missing numeric SINAD remains blank. Both radios were released at completion, and the receiver command audit found no TX commands.

`results.jsonl` retains point statistics and settings. Compressed paired snapshots and both host archives retain raw evidence. `verification.json` and `matrix-verification.json` record independent checks of identities, roles, full grid, statistics, settings, sample freshness, transport counters and CSV consistency.

The report documents offline regeneration. Acquisition sources preserve historical deployment paths and role assertions; review before reuse, since acquisition causes RF transmission. Verification and analysis are offline.

`SOURCE-SHA256SUMS` preserves the original completion manifest. `SHA256SUMS` covers this archive; `analysis/SHA256SUMS` covers the report and regenerated figures. This is a combined TX/link/RX measurement, without independently calibrated RF power or receiver sensitivity.
