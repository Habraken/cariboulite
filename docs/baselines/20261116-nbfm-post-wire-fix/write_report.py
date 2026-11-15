"""Write the measured NBFM regression report from reconciled results."""
import json,pathlib,sys
root=pathlib.Path(sys.argv[1]);out=pathlib.Path(sys.argv[2])
r=json.loads((root/'summary.json').read_text());c=json.loads((root/'comparison-summary.json').read_text());v=json.loads((root/'raw-verification.json').read_text())
rates=[4000000,2000000,1333333,1000000,800000,666667,500000,400000];labels=['4 MS/s','2 MS/s','4/3 MS/s','1 MS/s','800 kS/s','2/3 MS/s','500 kS/s','400 kS/s']
s=['# NBFM sample-rate and TX-power retest after TX wire fixes','',
'The restored **90 dB nominal attenuated link** connects pi4g-trixie as transmitter to pi4f-trixie as receiver at **430.100 MHz, RF24**. All eight native RX rates and all eight native TX rates passed the recorded power sweep, with zero recorded transport errors. This is 224 transmitting points and 16 TX-off baselines, rather than an 8-by-8 matrix of rate pairs.','',
'## What changed in measured performance','',
'This repeat does **not establish a NBFM SINAD improvement** over the [historical sweep](native-sample-rate-power-sweep-2026-11-14.md). The shared TX wire corrections remain necessary for sample accuracy, but their benefit cannot be inferred from a small change in this fixed-notch link-quality estimate.','']
for axis in ['rx','tx']:
 d=c[axis]['all']['sinad_db'];s.append(f"For the {axis.upper()}-rate axis, current minus historical mean SINAD spans **{d['min']:+.2f} to {d['max']:+.2f} dB**, averaging **{d['mean']:+.2f} dB** across all 112 transmitting points. These averages include the repeated PA-code-31 requests and are descriptive, not independent observations or causal estimates.")
 s.append('')
s+= [f"Within this run, the two separately measured 4 MS/s reference curves differ by up to **{c['repeat_4M']['sinad_db']:.2f} dB SINAD** and **{c['repeat_4M']['rssi_dbm']:.2f} dB RSSI** at matching requests. The largest observed within-point SINAD range is **{c['max_point_sinad_spread']:.3f} dB**. Neither quantity is a calibrated uncertainty bound.",'',
'Intervening changes include the TX byte-order correction, preservation of the low I-data bit, RX playback calibration, AM support and the corrected driver quiet-frame encoding. The SINAD implementation is unchanged, reads the raw discriminator before de-emphasis, volume and squelch, and AM DAC shaping is disabled throughout this NBFM run. Separate dates, calibration/startup state, temperature and the reconnected attenuation path prevent attributing differences solely to an individual fix.','',
'The fixed 1 kHz notch can leave residual fundamental energy from small tone/clock offsets. The measured strong-signal plateau therefore does not establish the RF distortion limit or receiver sensitivity. See the [estimator definition](sinad.md).','',
'## Figures and complete data','',
'- [SINAD curves](baselines/20261116-nbfm-post-wire-fix/sinad.svg)',
'- [RSSI curves](baselines/20261116-nbfm-post-wire-fix/rssi.svg)',
'- [Change in SINAD from historical sweep](baselines/20261116-nbfm-post-wire-fix/sinad_db-delta.svg)',
'- [Change in RSSI from historical sweep](baselines/20261116-nbfm-post-wire-fix/rssi_dbm-delta.svg)',
'- [All current point statistics, CSV](baselines/20261116-nbfm-post-wire-fix/results.csv)',
'- [Matched historical/current comparison, CSV](baselines/20261116-nbfm-post-wire-fix/comparison.csv)',
'- [Setup and exact profiles](baselines/20261116-nbfm-post-wire-fix/metadata.json)','',
'Bands show observed minima and maxima, not confidence intervals. Statistics average reported dB values arithmetically. Status snapshots at 5 Hz can repeat the estimator’s 500 ms windows and are not independent samples.','',
'## Selected measured points','',
'Cells show **mean raw RSSI dBm / mean estimated SINAD dB**. Requested powers are not power-meter readings.','']
for axis in ['rx','tx']:
 s += ['### '+axis.upper()+' rates; '+('TX' if axis=='rx' else 'RX')+' fixed at 4 MS/s','', '| Native rate | −12 dBm request | 0 dBm request | +8 dBm request |','| --- | ---: | ---: | ---: |']
 for rate,label in zip(rates,labels):
  vals=[]
  for power in [-12,0,8]:
   p=next(x for x in r if x['axis']==axis and x[axis+'_rate_hz']==rate and x['tx_power_dbm']==power)
   vals.append(f"{p['rssi_dbm']['mean']:.1f} / {p['sinad_db']['mean']:.2f}")
  s.append('| '+label+' | '+' | '.join(vals)+' |')
 s.append('')
s += ['## Isolated low-power dip and follow-up','',
'Point 167 (TX 1 MS/s, RX 4 MS/s, -12 dBm request) briefly reported 13.388 dB SINAD within a 15.192 dB mean window. No transport error accompanied it. This point remains in all original statistics and plots. Three subsequent repetitions using the same settling/measurement protocol returned means of 15.706, 15.643 and 15.611 dB, with respective minima of 15.532, 15.552 and 15.517 dB. The dip was not reproduced; its cause is unresolved. These are supplementary observations, not replacements for the original point. See [recheck results](baselines/20261116-nbfm-post-wire-fix/recheck-results.json).','',
'## Method and validation','',
'TX tone: 1 kHz, amplitude 0.4, 2500 Hz deviation setting (nominal 1000 Hz peak deviation). RX: NBFM, 50 microsecond de-emphasis, legacy trim 8000, SINAD enabled, Jabra USB playback. TX power requests run from -12 through +14 dBm in 2 dB steps. Each block contains an off baseline. The other direction stays at 4 MS/s. Every power transition stops, configures and restarts TX; two seconds of settling precede five seconds of measurement. Startup/tail cues are excluded.','',
'Each retained RX snapshot must be newer than the measurement start and have a fresh sequence. RSSI is the signed raw RF24 register. The profile verifier checks active tone/mode/rates and native hardware filter settings on every transmitting snapshot. The collector aborts on a radio-state/transport fault rather than discarding the failed point.','',
f"All **{v['snapshots']} retained snapshots** reconcile independently to the point statistics. Every transmitting point has valid SINAD throughout its retained window. RX alignment failures/invalid frames, reader errors, IQ/audio drops and playback underruns/write errors stayed zero, as did TX queue drops, writer errors and driver underruns. This verifies the measured windows, not indefinite uptime or every hardware initial condition.",'',
'+10, +12 and +14 dBm requests map to PA code 31 and repeat the same physical setting. Requested power minus 90 dB is nominal input only. Raw RSSI noise floors depend on RX rate, so cross-rate RSSI is not a calibrated sensitivity comparison. The 12 dB SINAD point was not reached. No calibrated receiver sensitivity, complete THD, spectral compliance or causal single-fix effect is established.','',
'Both backend binaries match SHA256 `25a9b8cf2c764fc4945e2fc5facb4b1143bf3bad9b7d606abc997cc4d3e390b9` at implementation commit `3edde8e`; both loaded/installed drivers have srcversion `1AC65CD6AE81A1E2ED63F14`. FPGA images and production settings were not modified. Both radios were stopped and released after collection.','',
'Collection began at 2026-11-15 23:11:39 UTC, which is 16 November in Europe/Brussels. The UTC run ID is retained in the artifacts. See the [baseline README](baselines/20261116-nbfm-post-wire-fix/README.md) for raw data locations, reproduction and verification.','']
out.write_text('\n'.join(s))
