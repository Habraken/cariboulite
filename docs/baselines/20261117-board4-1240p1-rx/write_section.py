"""Render an additional RF-frequency section without replacing the430.1 MHz record."""
import json,pathlib,sys
base=pathlib.Path(sys.argv[1]);dest=pathlib.Path(sys.argv[2]);rows=json.loads((base/'summary.json').read_text());raw=json.loads((base/'raw-verification.json').read_text());led=json.loads((base/'led-verification.json').read_text());c=json.loads((base/'comparison-summary.json').read_text());meta=json.loads((base/'metadata.json').read_text())
valid=sum(r['tx_power_dbm'] is not None and r['sinad_valid_fraction']==1 for r in rows)
passed=len(rows)==120 and not c['transport_error_points'] and valid==112 and not meta['startup_failures']
s=['## Additional qualification at 1240.1 MHz — 17 November 2026','',f"The same RX-rate/TX-power sweep at **1240.1 MHz** **{'passed' if passed else 'had failures or unavailable measurements'}**: **{len(rows)} points**, **{valid} transmitting points with fully valid SINAD**, and **{len(c['transport_error_points'])} points with nonzero transport counters**.",'',
'Receiver #4 remained on pi4f-trixie; the original CaribouLite on pi4g-trixie transmitted through the user-confirmed nominal 90 dB coax link. LED0 was preserved and LED1 explicitly set to 0 with checked readback before each RX rate. No reboot, EEPROM rewrite or test-app initialization was requested or performed for this sweep. No TX trial was run on #4.','',
'TX stayed at 4 MS/s, RF24 1240.100 MHz, with the same 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. Each of the eight native RX rates has a TX-off baseline and fourteen power requests from -12 to +14 dBm in 2 dB steps. Settling and measurement times remained 2 seconds and 5 seconds.','',
'Cells show **mean raw RSSI dBm / mean estimated SINAD dB**.','',
'| RX rate | -12 dBm TX request | 0 dBm TX request | +8 dBm TX request |','| --- | ---: | ---: | ---: |']
for rate,label in [(4000000,'4 MS/s'),(2000000,'2 MS/s'),(1333333,'4/3 MS/s'),(1000000,'1 MS/s'),(800000,'800 kS/s'),(666667,'2/3 MS/s'),(500000,'500 kS/s'),(400000,'400 kS/s')]:
 cells=[]
 for power in [-12,0,8]:
  r=next((r for r in rows if r['rx_rate_hz']==rate and r['tx_power_dbm']==power),None)
  if not r:cells.append('Startup failed')
  else:cells.append(f"{r['rssi_dbm']['mean']:.1f} / "+(f"{r['sinad_db']['mean']:.2f}" if r['sinad_db'] else 'unavailable'))
 s.append('| '+label+' | '+' | '.join(cells)+' |')
s+=['','### Verification and comparison','',f"All **{raw['snapshots']} retained measurement snapshots** independently reconcile with the point summaries. Mode/tone, requested/native rate readbacks, modem filters, board roles and transport counters were verified. LED setup checks cover all **{led['verified_rx_rate_cases']} RX rate starts**. Both radios were stopped and released afterward.",'']
for metric,label in [('rssi_dbm','Raw RSSI'),('sinad_db','Estimated SINAD')]:
 d=c[metric+'_1240p1_minus_430p1']
 if d:s.append(f"{label}, 1240.1 MHz minus 430.1 MHz on the same board, averaged **{d['mean']:+.2f} dB** across {d['n']} matched transmitting settings (range {d['min']:+.2f} to {d['max']:+.2f} dB).")
s+=['','These are end-to-end link differences, not isolated frequency-dependent receiver sensitivity. TX/mixer output, frontend gain, cables and attenuator response can vary with frequency; the nominal 90 dB link was not independently calibrated at either frequency. Fixed-notch SINAD also includes transmitter distortion and tone/clock-offset effects. +10/+12/+14 requests repeat PA code 31. The LED1 dependency and oscillator frequency remain unresolved and unmeasured respectively.','',f"[All 1240.1 MHz results](baselines/{base.name}/results.csv) · [RSSI plots](baselines/{base.name}/rssi.svg) · [SINAD plots](baselines/{base.name}/sinad.svg) · [Matched 430.1 MHz comparison](baselines/{base.name}/comparison-430p1.csv) · [Raw evidence and reproduction](baselines/{base.name}/README.md)",'']
dest.write_text('\n'.join(s))
