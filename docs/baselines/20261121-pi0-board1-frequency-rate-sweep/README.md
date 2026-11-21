# Pi Zero board #1 frequency and rate sweep — 21 November 2026

[Analysis report and plots](REPORT.md).

Original board on pi0a-trixie transmits to reconstructed board #1 on pi0b-trixie through nominal 90 dB attenuation, at requested 0 dBm. All 172 frequencies × eight matched sample rates completed. The receiver was RX only.

`results.csv` includes eight preflight rows and 1,376 matrix rows; use `phase=matrix` for the main sweep. `results.jsonl` retains full summaries and configuration. Compressed paired snapshots and both host archives preserve raw evidence. `verification.json` records the independent acquisition checks.

The report documents offline regeneration. `run_sweep.py`, `radio_helper.py`, and `finalize_sweep.py` preserve the acquisition sources and original deployment paths; they are historical records, not portable launch commands. Review addresses, key paths, role assertions, plan inputs, ownership, and RF connection before any reuse. The collectors cause TX on the original board.

`SOURCE-SHA256SUMS` preserves the original acquisition manifest; `SHA256SUMS` covers the committed package, and `analysis/SHA256SUMS` covers the regenerated analysis.
