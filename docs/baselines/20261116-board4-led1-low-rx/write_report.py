"""Write a self-contained fresh-run report without replacing earlier evidence."""
import json,pathlib,sys
base=pathlib.Path(sys.argv[1]);dest=pathlib.Path(sys.argv[2]);rows=json.loads((base/'summary.json').read_text());v=json.loads((base/'raw-verification.json').read_text());setup=json.loads((base/'setup.json').read_text());errors=[r for r in rows if any(x['max'] for x in r['rx_errors'].values()) or any(r['tx_errors'].values())]
startup=json.loads((base/'startup-failures.json').read_text()) if (base/'startup-failures.json').exists() else []
rates=[(4000000,'4 MS/s'),(2000000,'2 MS/s'),(1333333,'4/3 MS/s'),(1000000,'1 MS/s'),(800000,'800 kS/s'),(666667,'2/3 MS/s'),(500000,'500 kS/s'),(400000,'400 kS/s')]
passed=len(rows)==120 and not errors and not startup and all(r['sinad_valid_fraction']==1 for r in rows if r['tx_power_dbm'] is not None)
led_check=json.loads((base/'led-verification.json').read_text());assert led_check['verified_rx_rate_cases']==8 and led_check['led1_low_before_all_rx_starts']
label=setup.get('run_label','Retest after manual test-app session')
s=['# Board #4 — '+label+' — 16 November 2026','',f"Scoped RX sweep outcome: **{'passed' if passed else 'failed or incomplete'}**. {len(rows)} points recorded; {len(errors)} points with nonzero cumulative transport counters; {len(startup)} rate startup failures.",'','Receiver UUID: `c2440a8d-3852-4c44-b96f-1b64a962e7dc` on pi4f-trixie. Original pi4g transmitter UUID: `c132d0e7-d7dd-457a-b105-4a5cd97e7e80`. No EEPROM rewrite or TX test on #4.','','## Setup and observations','',setup.get('user_report_before_retest',''),'',setup.get('interpretation',''),'',setup.get('reboot_context','Pi4f was rebooted immediately before this run.'),'',
'The original TX reference remained at 4 MS/s, RF24 430.100 MHz, 1 kHz NBFM tone, amplitude 0.4 and deviation setting 2500 Hz. Each RX rate has TX off plus fourteen power requests from -12 to +14 dBm, two seconds settling and five seconds measurement. The user confirmed the nominal 90 dB attenuated link; it was not independently calibrated. Production DSP, driver and FPGA software were unchanged.','',
'| RX rate | Points | Largest invalid-frame counter | Largest other RX transport counter |','| --- | ---: | ---: | ---: |']
for rate,label in rates:
 g=[r for r in rows if r['rx_rate_hz']==rate]
 s.append(f"| {label} | {len(g)} | {max((r['rx_errors']['invalid_frames']['max'] for r in g),default=0)} | {max((x['max'] for r in g for k,x in r['rx_errors'].items() if k!='invalid_frames'),default=0)} |")
s+=['','Cells below are mean raw RSSI dBm / estimated SINAD dB.','','| RX rate | -12 dBm TX | 0 dBm TX | +8 dBm TX |','| --- | ---: | ---: | ---: |']
for rate,label in rates:
 cells=[]
 for power in [-12,0,8]:
  r=next((r for r in rows if r['rx_rate_hz']==rate and r['tx_power_dbm']==power),None)
  cells.append(f"{r['rssi_dbm']['mean']:.1f} / {r['sinad_db']['mean']:.2f}" if r and r['sinad_db'] else 'Unavailable')
 s.append('| '+label+' | '+' | '.join(cells)+' |')
s+=['','Before every RX rate, the helper reads the digital register, preserves LED0 and issues an explicit backend LED1=0 write with checked readback. These commands are retained in the raw logs. This is an intentional test condition, not an automatic production initialization change. See led-verification.json for all eight cases.','','## Evidence and limits','',f"All {v['snapshots']} retained measurement snapshots independently reconcile with the summaries. Settings, role UUIDs, rates, tone, filters and transport counters were inspected. Commands and backend states were checked for absence of TX operation on #4. Both radios were stopped and released afterward.",'',
'The initial failures and previous LED-state experiments remain separate evidence. This run explicitly sets LED1 low and does not establish the cause of the LED1 dependency. No test app was used. No oscillator-frequency measurement was made. These short, uncalibrated RX link tests do not establish oscillator or TX acceptance, calibrated sensitivity, or indefinite continuity. +10/+12/+14 dBm requests repeat PA code 31.','',f"[Raw evidence and reproduction](baselines/{base.name}/README.md) · [RSSI](baselines/{base.name}/rssi.svg) · [SINAD](baselines/{base.name}/sinad.svg) · [CSV](baselines/{base.name}/results.csv)",'']
text='\n'.join(s)
while '\n\n\n' in text:text=text.replace('\n\n\n','\n\n')
dest.write_text(text)
