"""Render a board-specific report from independently reconciled RX data."""
import json,pathlib,sys
base=pathlib.Path(sys.argv[1]);out=pathlib.Path(sys.argv[2]);r=json.loads((base/'summary.json').read_text());v=json.loads((base/'raw-verification.json').read_text());c=json.loads((base/'comparison-summary.json').read_text())
s=['# Board #3: EEPROM identity and RX-only qualification — 16 November 2026','',
'Board **#3**, UUID **`f23c0015-fd1b-47b7-8314-f5c9e7de0ea7`**, initialized successfully and completed the recorded RX-only NBFM rate/power sweep on pi4f-trixie. All eight native RX rates completed, with zero recorded transport errors and valid SINAD throughout all transmitting measurement windows.','',
'## Identities and initialization','',
'| Role | Label | UUID |','| --- | --- | --- |',
'| Current receiver, pi4f | #3 | `f23c0015-fd1b-47b7-8314-f5c9e7de0ea7` |',
'| Previous receiver; comparison measurements | #1 | `d8810044-c444-46f4-a548-703412cbf971` |',
'| Original TX reference, pi4g | Original reference, separate from #1 | `c132d0e7-d7dd-457a-b105-4a5cd97e7e80` |','',
'Two initial full 4096-byte EEPROM reads matched and were entirely FF. A new unique UUID was generated, and the existing checked blank-only writer programmed the image with page verification and the header page last. Two final full readbacks matched SHA256 `7543f5c481f0680f5e46a2b2bf0fd4c5fcf096731eac4b2705ab8bf567e400fe`; independent image parsing/CRCs and eepdump passed. After reboot, the HAT identity, Full product ID/version and SMI device were present. The EEPROM programming and boot evidence is retained in the [baseline](baselines/20261116-board3-rx/).','',
'The user reports an incorrect U25 oscillator on #3. Its actual frequency was not measured during this qualification; #1\u2019s prior scope measurement is not a measurement of #3. No TX trial was run on #3. These RX results do not qualify TX, the oscillator or the complete hardware design.','',
'## Measured RX results','',
'Original TX reference fixed at 4 MS/s; 430.100 MHz RF24; nominal 90 dB attenuated wired link. TX tone 1 kHz, amplitude 0.4, deviation setting 2500 Hz (nominal 1000 Hz peak deviation). Each of eight RX rates has TX off and fourteen TX-power requests (-12 to +14 dBm in 2 dB steps): **120 points**, including 112 transmitting points and eight off baselines.','',
'Cells are **mean raw RSSI dBm / mean estimated SINAD dB**.','',
'| RX native rate | -12 dBm TX request | 0 dBm TX request | +8 dBm TX request |','| --- | ---: | ---: | ---: |']
for rate,label in zip([4000000,2000000,1333333,1000000,800000,666667,500000,400000],['4 MS/s','2 MS/s','4/3 MS/s','1 MS/s','800 kS/s','2/3 MS/s','500 kS/s','400 kS/s']):
 vals=[]
 for power in [-12,0,8]:
  p=next(x for x in r if x['rx_rate_hz']==rate and x['tx_power_dbm']==power);vals.append(f"{p['rssi_dbm']['mean']:.1f} / {p['sinad_db']['mean']:.2f}")
 s.append('| '+label+' | '+' | '.join(vals)+' |')
s+=['','## Comparison with receiver #1','',
'The comparison uses only the RX-rate axis of the [post-wire-fix #1 sweep](nbfm-post-wire-fix-sweep-2026-11-16.md), at the same rate and requested power, with matching PA codes and the same backend binary. It does not compare against the older pre-wire-fix transmitter results.','']
for m,label in [('sinad_db','Estimated SINAD'),('rssi_dbm','Raw RSSI')]:
 d=c[m+'_delta'];s += [f"{label}, #3 minus #1, spans **{d['min']:+.2f} to {d['max']:+.2f} dB**, with arithmetic mean **{d['mean']:+.2f} dB** across 112 transmitting points.",'']
d2=c['comparison_board2']['sinad_db_delta'];r2=c['comparison_board2']['rssi_dbm_delta']
s += [f"Against #2, estimated SINAD differences span **{d2['min']:+.2f} to {d2['max']:+.2f} dB**, averaging **{d2['mean']:+.2f} dB** at matching settings. Raw RSSI averages **{r2['mean']:+.2f} dB** relative to #2. See the [separate #2/#3 comparison CSV](baselines/20261116-board3-rx/comparison-board2.csv).",'']
s+=['These are observed link differences, not isolated receiver-quality effects. Boards were swapped and the link reconnected; calibration/startup/temperature and clock states can differ. The fixed 1 kHz notch is sensitive to tone/clock offsets and can limit strong-signal readings. Raw RSSI includes rate-dependent noise-floor offsets, and the link/power levels are uncalibrated. Higher SINAD here does not by itself establish better receiver sensitivity or less intrinsic RF distortion.','',
'- [RSSI and difference plots](baselines/20261116-board3-rx/rssi.svg)',
'- [SINAD and difference plots](baselines/20261116-board3-rx/sinad.svg)',
'- [All point statistics, CSV](baselines/20261116-board3-rx/results.csv)',
'- [Matched #1/#3 statistics, CSV](baselines/20261116-board3-rx/comparison.csv)',
'- [Setup and profiles](baselines/20261116-board3-rx/metadata.json)','',
'## Verification and limits','',
f"All **{v['snapshots']} retained snapshots** reconcile independently with the 120 summaries. Each power change had two seconds settling and five seconds measurement; fresh RX sequences/timestamps were required. Status sampling can repeat 500 ms SINAD estimates, so snapshot counts are not independent observations. All transmitting points had valid SINAD throughout. Largest observed within-point SINAD spread: **{c['max_within_point_sinad_range']:.3f} dB**. Plot bands are observed min/max, not uncertainty bounds.",'',
'Checks covered RX alignment failures/invalid frames, reader errors, IQ/audio drops, ALSA underruns/write errors and TX-reference queue drops/writer errors/driver underruns: all retained measurement windows had zero counters. Native rates and filter readbacks were checked; these are register observations, not physical clock metrology. UUID/host guards confined the receiver to #3, and saved RX backend states/commands were checked for absence of TX operation.','',
'The 12 dB SINAD point was not reached in this sweep. +10/+12/+14 dBm requests all use PA code 31 and repeat a hardware setting. SINAD is the existing raw-discriminator, 300–3000 Hz fixed-notch estimate before squelch/playback controls. No calibrated sensitivity, RF power or emissions compliance, indefinite continuity or TX qualification is established.','',
'Both backends used SHA256 `25a9b8cf2c764fc4945e2fc5facb4b1143bf3bad9b7d606abc997cc4d3e390b9`; loaded and installed driver srcversion was `1AC65CD6AE81A1E2ED63F14`. No DSP/driver/FPGA software change was made for this test. The temporary SSH trust file was restored with pi4g\u2019s existing public host key before collection. Both radios were stopped and released afterward.','',
'See [per-board tracking](board-rx-validation.md) and the [baseline README](baselines/20261116-board3-rx/README.md) for retained raw data and reproduction.','']
out.write_text('\n'.join(s))
