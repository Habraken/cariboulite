"""Render the measured report from the validated compact sweep summary."""
import json,pathlib,sys
base=pathlib.Path(sys.argv[1]);target=pathlib.Path(sys.argv[2]);rows=json.loads((base/'summary.json').read_text());meta=json.loads((base/'metadata.json').read_text())
rates=[4000000,2000000,1333333,1000000,800000,666667,500000,400000]
labels=['4 MS/s','2 MS/s','4/3 MS/s','1 MS/s','800 kS/s','2/3 MS/s','500 kS/s','400 kS/s']
tx=[r for r in rows if r['tx_power_dbm'] is not None];off=[r for r in rows if r['tx_power_dbm'] is None]
assert len(tx)==224 and len(off)==16
spread=max(tx,key=lambda r:r['sinad_db']['max']-r['sinad_db']['min']);worst=spread['sinad_db']['max']-spread['sinad_db']['min']
low=[r for r in tx if r['tx_power_dbm']==-12]
weaklo=min(r['sinad_db']['mean'] for r in low);weakhi=max(r['sinad_db']['mean'] for r in low)
s='''# Native sample-rate and TX power sweeps after continuity fixes

The original CaribouLite on **pi4g-trixie** transmitted a 1 kHz NBFM tone
through the user's wired connection with **90 dB nominal attenuation** to
**pi4f-trixie**, at **430.100 MHz**. The Jabra USB speaker was used for playback.

Two sweeps isolate the directions: all eight RX rates with TX fixed at 4 MS/s,
then all eight TX rates with RX fixed at 4 MS/s. Each block includes TX off
and requested TX power −12 through +14 dBm in 2 dB steps. This gives
**224 transmitting points and 16 TX-off baselines**. The 4 MS/s–4 MS/s curve
is measured independently in both sweeps; this is not an 8-by-8 rate matrix.

'''
s+=f"**Every recorded transport error counter stayed zero.** The largest within-point SINAD spread was **{worst:.3f} dB**. At the lowest requested power, mean SINAD across the tested rate pairs was **{weaklo:.2f}–{weakhi:.2f} dB**.\n\n"
s+='''The alignment fix, burst-aware playback buffer and FIFO-preserving TX restart
were in place for this run. The user reported clean tones after the driver
change. These results cover the recorded runs, not indefinite uptime or every
possible startup condition.

## Figures and complete data

- [SINAD curves](baselines/20261114-native-power-sweep/sinad.svg)
- [RSSI curves](baselines/20261114-native-power-sweep/rssi.svg)
- [All point statistics and error counters, CSV](baselines/20261114-native-power-sweep/results.csv)
- [Structured summary](baselines/20261114-native-power-sweep/summary.json)
- [Setup and native filter profiles](baselines/20261114-native-power-sweep/metadata.json)

Curve bands show observed minima/maxima, not confidence intervals. Means of
reported dB values are arithmetic averages, not power-domain aggregates.
Status is sampled at 5 Hz while SINAD updates at 2 Hz; repeated snapshots are
not independent measurements. All recorded values are retained, with no
filter based on the measured SINAD value.

## Selected points

Each cell is **mean raw RSSI dBm / mean estimated SINAD dB**. The CSV contains
every power step; these tables highlight the weakest input, 0 dBm request and
+8 dBm request.
'''
for axis in ['rx','tx']:
 s+='\n### '+('RX rates, TX fixed at 4 MS/s' if axis=='rx' else 'TX rates, RX fixed at 4 MS/s')+'\n\n| Native rate | −12 dBm TX | 0 dBm TX | +8 dBm TX |\n| --- | ---: | ---: | ---: |\n'
 for rate,label in zip(rates,labels):
  s+='| '+label+' |'
  for power in [-12,0,8]:
   r=next(r for r in tx if r['axis']==axis and r[axis+'_rate_hz']==rate and r['tx_power_dbm']==power)
   s+=f" {r['rssi_dbm']['mean']:.1f} / {r['sinad_db']['mean']:.2f} |"
  s+='\n'
s+='''
## Power and sensitivity limits

Requested power minus 90 dB is nominal input, not a calibrated RF level.
The RF24 mapping sends +10, +12 and +14 dBm requests to the same PA code 31;
those are repeated physical settings. The backend's actual-power field is
an estimate from its model, not a power-meter measurement. The CSV records
requested power, observed PA code and that model estimate separately.

'''
if weaklo>12:s+='The 12 dB SINAD point was not reached with the available attenuation and TX power range. No absolute receiver sensitivity is established.\n\n'
s+='''The estimator measures the 300–3000 Hz, unweighted link with a fixed 1 kHz
notch. Transmitter noise/distortion, tone-frequency error, filtering and the
uncalibrated RF path limit interpretation. The stronger-signal plateau is not
an absolute receiver SINAD ceiling. See [estimator definition](sinad.md).

## TX-off baselines and RSSI

| Sweep | Variable native rate | Mean RSSI (dBm) | Mean noise RMS | Gate open fraction |
| --- | --- | ---: | ---: | ---: |
'''
for r in off:
 label=labels[rates.index(r[r['axis']+'_rate_hz'])]
 s+=f"| {r['axis'].upper()} | {label} | {r['rssi_dbm']['mean']:.2f} | {r['noise_rms']['mean']:.4f} | {r['gate_open_fraction']:.3f} |\n"
s+='''
Raw RSSI noise floors depend strongly on RX rate/filter settings even with TX
off. They should not be compared across rates as calibrated input levels.
Post-demodulation noise RMS provides a different measurement. This test does
not establish the mechanism or calibration of the RSSI offset.

## Repeated 4 MS/s reference

'''
r1={r['tx_power_dbm']:r for r in tx if r['axis']=='rx' and r['rx_rate_hz']==4000000}
r2={r['tx_power_dbm']:r for r in tx if r['axis']=='tx' and r['tx_rate_hz']==4000000}
s+=f"The independently repeated 4 MS/s–4 MS/s curves differed by at most {max(abs(r1[p]['sinad_db']['mean']-r2[p]['sinad_db']['mean']) for p in r1):.3f} dB in mean SINAD and {max(abs(r1[p]['rssi_dbm']['mean']-r2[p]['rssi_dbm']['mean']) for p in r1):.3f} dB in mean RSSI at matching requests. These are temporal comparisons, not calibrated uncertainty estimates.\n\n"
s+='''## Collection and validation

The TX tone amplitude was 0.4 with a 2500 Hz deviation setting: nominal peak
modulation deviation 1000 Hz. RX used NBFM, 2500 Hz audio normalization,
50 µs de-emphasis, gain 8000 and noise squelch thresholds 0.20/0.30; carrier
squelch was off. SINAD observes the audio before squelch muting and volume.
All native clocks use exact 4 MHz/divider arithmetic; fractional-Hz command
labels are rounded. Profile details are in the metadata.

For each power step, TX was stopped, configured and restarted. Status was
drained throughout two seconds of settling, then recorded for five seconds.
Only RX snapshots with timestamps newer than the measurement start were
accepted. Intentional TX power-transition gaps and Quindar cues are outside
the measurement windows. Every point has at least 20 fresh RSSI snapshots;
every transmitting point has valid SINAD throughout its retained window.

The collector checked cumulative RX alignment failures/invalid frames, RX
read errors, IQ/audio queue drops, ALSA underruns/write errors, TX queue drops,
TX writer errors and driver underruns. All were zero. It would stop on a
transport fault or an unexpected TX/RX state rather than discard that point.
SINAD spread itself was retained and summarized independently of those checks.

The source manifest matched the committed implementation on pi4f-trixie.
Both radios used the same installed FIFO-preserving driver image, retained
rollback modules and their existing driver parameters. Backend, library,
controller, test application and Soapy builds were refreshed. No backend
startup service was added.

## Provenance and reproduction

Implementation commits:

- `8618ddd`: streaming RX framing and fragmented-sample preservation.
- `be0a0b0`: all native RX rates and DMA-burst-aware playback buffering.
- `24f1411`: preserve TX IQ at finite SMI-window restarts.
- `1e4948c`: all native TX rates and exact fractional frame pacing.

See the [alignment proof](alignment-fix-validation-2026-11-13.md),
[RX-rate validation](rx-native-rates-validation-2026-11-13.md),
[TX restart comparison](tx-window-restart-validation-2026-11-14.md) and
[TX-rate validation](tx-native-rates-validation-2026-11-13.md).

The collector, radio helper, source manifest and analysis/report scripts are
committed alongside the data. Full per-snapshot records and backend logs are
retained under `build/radio-backend/implementation-20261113/`; temporary
originals remain on the radios under the run directory recorded in metadata.
Both backends/controllers were stopped and playback was closed after the run.
'''
s+='\nIndependent analysis reconciled all **5760 saved snapshots** with the point\nsummaries, including statistics, state/profile/tone settings and monotonic\ntimestamp order. [Verification record](baselines/20261114-native-power-sweep/raw-verification.json)\nand [final hardware-release check](baselines/20261114-native-power-sweep/release-check.jsonl)\nare committed with the results. No radio application or SMI owner remained,\nand every recorded playback substream was closed on both hosts.\n'
target.write_text(s)
print('Wrote',target)
