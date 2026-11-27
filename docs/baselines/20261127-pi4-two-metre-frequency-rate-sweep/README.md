# Pi 4 sweep with approximately two-metre separation — 27 November 2026

Requested repeat of the 23 November full frequency/rate matrix: original board on pi4g TX (192.168.6.77) to repaired board #2 on pi4f RX (192.168.6.163). Board UUIDs are checked before acquisition. H650e playback remains on RX.

Operator reports approximately 2 m separation, an added 2 m RG179 cable and unchanged nominal 90 dB attenuators. Cable loss is additional and uncalibrated. Hosts and cable changed relative to the Pi Zero reference, so differences cannot isolate separation alone.

172 frequencies from 30 MHz to 5999.999 MHz, eight matched rates, 0 dBm requested TX, 1 kHz NBFM tone, amplitude 0.4, deviation setting 2500 Hz; 2 s settling and 5 s measurement. Eight 430.1 MHz preflight points gate full acquisition. RX-only role protection, exclusive-owner checks, heartbeat timeout, cleanup and LED restoration are inherited from the reference procedure.

Both Pis are being updated to GitHub `dev-fpga-pa-control` revision `a997ec4b92e28fc79fe38b320654d334dae61619`. Local changes are preserved in `build/pre-github-update-20261127` and, where present, a named Git stash. Rebuild/update logs remain beside the acquisition controller.

Acquisition and automatic verification run on pi4f under /home/pi/src/cariboulite/build/pi4-two-metre-sweep-20261127. See remote controller.log, latest-run.txt and the run progress.json. Finalization collects host evidence, checks raw snapshots and writes REPORT.txt; verify_matrix.py independently checks the complete grid. Sources here are live RF acquisition tools.

`compare_reference.py` produces the paired comparison against the 23 November same-board Pi Zero reference after successful verification. `plot_sweep.py <run-directory>` regenerates the four response figures with matplotlib and numpy after collection.

Live run: `pi4-two-metre-20261127T081042Z`, started 09:10 Europe/Brussels. Both GitHub updates and installations succeeded with clean working trees. Both backends link the rebuilt `build/github-latest-20261127/libcariboulite.so`. All eight 430.1 MHz preflight measurements passed with 23.50–25.12 dB mean SINAD and clean measured transport counters; the 1,376-point matrix began at 09:11:55. The run completed at 12:39 Europe/Brussels. All 1,376 matrix points completed without retries or transport-flagged windows; both verification checks passed. Data, the comparison report and four chart sets were automatically collected locally.

[Completed report and interpretation](pi4-two-metre-20261127T081042Z/REPORT.md).
