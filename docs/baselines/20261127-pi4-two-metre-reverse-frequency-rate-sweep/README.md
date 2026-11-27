# Reciprocal Pi 4 full frequency/rate sweep — 27 November 2026

Repaired board #2 on pi4f (192.168.6.163) transmits; original reference on pi4g (192.168.6.77) receives. The H650e headset was verified on pi4g. The operator reports moving the headset while attempting to minimize disturbance of the approximately 2 m separated setup, added 2 m RG179 and nominal 90 dB attenuators.

Repeat of the full 172-frequency × eight matched-rate matrix at 0 dBm requested TX. Same NBFM tone, gain profiles, disabled squelch/signalling, 2 s settling and 5 s measurement. Both hosts retain GitHub revision a997ec4 and the libraries/backends used for the forward run. Board UUID and RX-only assertions are reversed explicitly. Eight clean preflight measurements gate the matrix.

Controller runs independently on pi4f in build/pi4-two-metre-reverse-sweep-20261127. Automatic finalization collects both hosts, verifies the raw snapshots and matrix, compares reverse minus forward, and checks radio release. Local collection generates the four plot sets. Acquisition sources operate live RF hardware; analysis and verification are offline.

Live run: `pi4-two-metre-reverse-20261127T120321Z`, started 13:03 Europe/Brussels. All eight 430.1 MHz preflight rates passed, with mean SINAD 22.30–24.81 dB and clean selected transport counters. The full matrix began at 13:04:33. Automatic comparison uses the completed forward run `pi4-two-metre-20261127T081042Z`; results are reverse minus forward. Completed at 16:32 Europe/Brussels: all points verified, 13 TX-start failures recovered on retry, zero selected transport flags within retained measurement windows.

[Completed reciprocal report and comparison](pi4-two-metre-reverse-20261127T120321Z/REPORT.md).
