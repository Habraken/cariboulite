# Separate TX and RX frequencies in menu 14

Menu 14 now keeps two independent frequency settings. Both start at
430.100000 MHz. Press **F** to edit TX or **G** to edit RX, enter MHz (for example
`430.125` or `145.500`), and press Enter. Escape cancels; Backspace edits the
entry. Empty or invalid input leaves the previous setting unchanged.

Stop both streams before editing frequencies. The monitor rejects frequency
controls during interface loopback as well. Saving changes configuration only:
it does not transmit, start RX, or immediately tune the radio. The display shows
both requested frequencies to six decimal places in MHz. The hardware may round
the requested value to its synthesizer resolution.

**T** starts TX using the saved TX frequency; **R** starts RX using the saved RX
frequency. Starting either direction stops the other. Both use the same RF24/HiF
radio and tuner, so these are alternate receive/transmit frequencies, not
simultaneous duplex operation. Before activation, the monitor explicitly tunes
the shared hardware for the requested direction, even after pipeline recreation.
A reported tuning failure prevents that stream from starting and displays an
error notice. The requested settings remain available for correction or retry.

Both settings survive TX/RX switches, stopped 2/4 MS/s changes and pipeline
recreation within the monitor session. Leaving and re-entering menu 14 restores
defaults; there is no configuration-file persistence in this increment.
Noise/carrier squelch choices remain independent of these frequencies.

Input validation follows the current driver's HiF ranges: 1 MHz inclusive to
6000 MHz exclusive on the full board, and 2385–2495 MHz inclusive on the ISM
board. These are driver limits, not a claim of verified performance throughout
those ranges. This increment retains the existing 430.1 MHz monitor default;
ISM-board default initialization is not changed.

## Implementation and validation

The monitor parses and saves configuration, then calls the pipeline frequency
setters before starting each direction. Setters now propagate driver failures;
TX power is applied only after tuning succeeds. Pipeline initialization passes
local frequency copies to the driver, which writes back the achieved frequency,
so it no longer modifies the caller's const parameter object.

Hardware-free lifecycle tests cover different TX/RX frequencies on the shared
radio at both sample rates, restoration before activation, failed-tune start
blocking and recovery, and preservation of requested values when the driver
rounds its output. Input tests cover decimal MHz, whitespace, non-finite values,
trailing junk and board ranges. Loopback tests cover both new hotkeys.

The application build, RX lifecycle, TX stop-deadline, monitor loopback and
baseline-runner software checks pass. Physical acceptance is pending. With an
appropriate test setup:

1. Stop TX/RX; use F and G to select two distinct frequencies.
2. Start RX with R and verify the signal on the RX frequency; stop/switch to TX
   and verify the transmitted frequency with the monitoring receiver.
3. Switch back to RX, then repeat at the other sample rate with streams stopped
   during the rate change. Verify both requested frequencies remain displayed
   and each direction uses its own setting.
4. Check Escape, invalid input, and attempted edits while streaming. Confirm
   these leave the saved settings unchanged.

These checks exercise the new tuning controls. Earlier refactoring acceptance
records correctly deferred retuning when no interface control existed.
