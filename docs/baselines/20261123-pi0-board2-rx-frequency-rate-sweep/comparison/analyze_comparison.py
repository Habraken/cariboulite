"""Offline comparison of reciprocal sweeps and the earlier board #1 RX run."""
import csv
import hashlib
import json
import pathlib
import statistics
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = pathlib.Path(sys.argv[1]).resolve()
FORWARD = ROOT.parent / '20261123-pi0-board2-tx-frequency-rate-sweep'
BOARD1 = ROOT.parent / '20261121-pi0-board1-frequency-rate-sweep'
DEST = ROOT / 'comparison'
DEST.mkdir(exist_ok=True)

def read(root):
    assert json.loads((root / 'verification.json').read_text())['passed']
    rows = [json.loads(line) for line in (root / 'results.jsonl').read_text().splitlines()]
    matrix = [r for r in rows if r['phase'] == 'matrix']
    lookup = {(r['frequency_hz'], r['sample_rate_hz']): r for r in matrix}
    assert len(matrix) == len(lookup) == 1376
    return lookup, json.loads((root / 'plan.json').read_text())

rev, rp = read(ROOT)
fwd, fp = read(FORWARD)
old, op = read(BOARD1)
assert set(rev) == set(fwd) == set(old)
assert rp['tx_board_uuid'] == fp['rx_board_uuid'] == op['tx_board_uuid']
assert rp['rx_board_uuid'] == fp['tx_board_uuid']
for p in (rp, fp, op):
    for field, expected in [('tone_hz', 1000), ('tone_amplitude', .4),
                            ('tx_deviation_hz', 2500), ('settle_seconds', 2),
                            ('measure_seconds', 5), ('tx_power_dbm', 0)]:
        assert p[field] == expected, (field, p[field])

rates = sorted({k[1] for k in rev}, reverse=True)
freqs = sorted({k[0] for k in rev})
labels = {4000000:'4', 2000000:'2', 1333333:'1.333333', 1000000:'1',
          800000:'0.8', 666667:'0.666667', 500000:'0.5', 400000:'0.4'}

def value(row, field):
    return row[field]['mean'] if row[field] is not None else None

def usable(row):
    return row['transport_clean'] and row['sinad_valid_fraction'] == 1

def clean_pair(a, b):
    return usable(a) and usable(b)

flat = []
for key in sorted(rev, key=lambda k: (-k[1], k[0])):
    r, f, o = rev[key], fwd[key], old[key]
    row = dict(frequency_hz=key[0], sample_rate_hz=key[1])
    for label, entry in [('forward', f), ('reverse', r), ('board1', o)]:
        row.update({label+'_rssi_dbm': value(entry, 'rssi_dbm'),
                    label+'_sinad_db': value(entry, 'sinad_db'),
                    label+'_sinad_valid_fraction': entry['sinad_valid_fraction'],
                    label+'_transport_clean': entry['transport_clean']})
    row['reverse_minus_forward_sinad_db'] = (value(r,'sinad_db') - value(f,'sinad_db')) if clean_pair(r,f) else None
    row['reverse_minus_forward_rssi_db'] = (value(r,'rssi_dbm') - value(f,'rssi_dbm')) if r['transport_clean'] and f['transport_clean'] else None
    row['board2rx_minus_board1rx_sinad_db'] = (value(r,'sinad_db') - value(o,'sinad_db')) if clean_pair(r,o) else None
    flat.append(row)
with (DEST / 'paired-results.csv').open('w', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(flat[0]), lineterminator='\n')
    writer.writeheader()
    writer.writerows(flat)

summary = []
for rate in rates:
    rows = [r for r in flat if r['sample_rate_hz'] == rate]
    direction = [r['reverse_minus_forward_sinad_db'] for r in rows if r['reverse_minus_forward_sinad_db'] is not None]
    receivers = [r['board2rx_minus_board1rx_sinad_db'] for r in rows if r['board2rx_minus_board1rx_sinad_db'] is not None]
    rssi = [r['reverse_minus_forward_rssi_db'] for r in rows if r['reverse_minus_forward_rssi_db'] is not None]
    summary.append(dict(rate_hz=rate, clean_full_direction_pairs=len(direction),
                        median_reverse_minus_forward_sinad_db=statistics.median(direction),
                        median_reverse_minus_forward_rssi_db=statistics.median(rssi),
                        clean_full_board1_pairs=len(receivers),
                        median_board2rx_minus_board1rx_sinad_db=statistics.median(receivers)))

plt.rcParams.update({'font.size':10, 'axes.titlesize':12, 'axes.labelsize':11,
                     'figure.facecolor':'white', 'axes.spines.top':False,
                     'axes.spines.right':False, 'svg.fonttype':'none'})
def save(fig, name):
    fig.savefig(DEST/(name+'.png'), dpi=180, bbox_inches='tight')
    fig.savefig(DEST/(name+'.svg'), bbox_inches='tight')
    plt.close(fig)
    svg = DEST/(name+'.svg')
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')

fig, axes = plt.subplots(2, 1, figsize=(13.5,8), sharex=True, layout='constrained')
for lookup, label, color in [(fwd,'Today: board #2 TX → original RX','#2166ac'),
                            (rev,'Reverse: original TX → board #2 RX','#b2182b'),
                            (old,'21 Nov: original TX → board #1 RX','#666666')]:
    for ax, field in zip(axes, ('rssi_dbm','sinad_db')):
        yy = [value(lookup[(hz,1000000)], field) for hz in freqs]
        ax.plot(np.array(freqs)/1e9, yy, '.-', color=color, ms=3, lw=1, label=label)
for ax in axes:
    ax.grid(alpha=.2)
    ax.set_xlim(.03,6)
    for boundary in (2.385,2.495):
        ax.axvline(boundary, color='#777777', ls=':', lw=.8)
axes[0].legend(fontsize=9, loc='upper left')
axes[0].set_title('Complete conducted links at 1 MS/s — requested 0 dBm, nominal 90 dB attenuation')
axes[0].set_ylabel('Mean reported RSSI (dBm)')
axes[1].set_ylabel('Mean estimated SINAD (dB)')
axes[1].set_ylim(bottom=0)
axes[1].set_xlabel('Requested frequency (GHz)')
save(fig, 'three-links-1msps')

f = np.array(freqs)/1e9
edges = np.r_[f[0],(f[:-1]+f[1:])/2,f[-1]]
lookup_flat = {(r['frequency_hz'],r['sample_rate_hz']):r for r in flat}
fig, axes = plt.subplots(2,1,figsize=(13.5,8),sharex=True,layout='constrained')
for ax, field, label in zip(axes,
        ('reverse_minus_forward_rssi_db','reverse_minus_forward_sinad_db'),
        ('RSSI difference (dB)','SINAD difference (dB)')):
    data = np.array([[lookup_flat[(hz,rate)][field] if lookup_flat[(hz,rate)][field] is not None else np.nan for hz in freqs] for rate in rates])
    limit = max(1, float(np.ceil(np.nanmax(np.abs(data)))))
    cmap = plt.get_cmap('RdBu_r').copy()
    cmap.set_bad('#c7cbd1')
    im = ax.pcolormesh(edges,np.arange(9),np.ma.masked_invalid(data),
                       cmap=cmap,vmin=-limit,vmax=limit,shading='flat',rasterized=True)
    ax.set_yticks(np.arange(8)+.5,[labels[rate] for rate in rates])
    ax.invert_yaxis()
    ax.set_ylabel('Matched rate (MS/s)')
    ax.set_title(label+' — reverse minus earlier direction')
    fig.colorbar(im,ax=ax,label=label,pad=.015)
axes[1].set_xlabel('Requested frequency (GHz)')
save(fig, 'direction-differences')

direction = [r['reverse_minus_forward_sinad_db'] for r in flat if r['reverse_minus_forward_sinad_db'] is not None]
receivers = [r['board2rx_minus_board1rx_sinad_db'] for r in flat if r['board2rx_minus_board1rx_sinad_db'] is not None]
metrics = dict(direction_clean_full_pairs=len(direction),
               median_reverse_minus_forward_sinad_db=statistics.median(direction),
               board1_clean_full_pairs=len(receivers),
               median_board2rx_minus_board1rx_sinad_db=statistics.median(receivers),
               rate_summary=summary)
health = {}
for label, root, lookup in [('Earlier direction',FORWARD,fwd),
                             ('Reverse direction',ROOT,rev),
                             ('Historical board #1',BOARD1,old)]:
    progress = json.loads((root/'progress.json').read_text())
    health[label] = dict(points=len(lookup),
                        no_numeric_sinad=sum(r['sinad_db'] is None for r in lookup.values()),
                        partial_sinad=sum(0<r['sinad_valid_fraction']<1 for r in lookup.values()),
                        transport_flagged=sum(not r['transport_clean'] for r in lookup.values()),
                        failed_setup_attempts=len(progress['failures']))
metrics['acquisition_quality'] = health
regions = []
for label, lo, hi in [('Below 2385 MHz',0,2385000000),
                       ('2385 to below 2495 MHz',2385000000,2495000000),
                       ('2495 MHz and above',2495000000,6000000000)]:
    rows = [r for r in flat if lo<=r['frequency_hz']<hi]
    dd = [r['reverse_minus_forward_sinad_db'] for r in rows if r['reverse_minus_forward_sinad_db'] is not None]
    oo = [r['board2rx_minus_board1rx_sinad_db'] for r in rows if r['board2rx_minus_board1rx_sinad_db'] is not None]
    regions.append(dict(region=label,direction_pairs=len(dd),
                        median_reverse_minus_forward_sinad_db=statistics.median(dd),
                        board1_pairs=len(oo),median_board2rx_minus_board1rx_sinad_db=statistics.median(oo)))
metrics['frequency_regions'] = regions
(DEST/'summary.json').write_text(json.dumps(metrics,indent=2)+'\n')

lines = ['# Reciprocal sweep comparison: original board and repaired device #2',
 '', 'Both directions use the same two boards, software builds, Logitech H650e headset, frequency/rate grid and nominal 90 dB HiF link. The operator moved the headset to whichever Pi was receiving. TX requests and reports 0 dBm; the source is a 1 kHz NBFM tone of amplitude 0.4 with deviation setting 2,500 Hz.',
 '', '| Dataset | TX | RX |', '|---|---|---|',
 '| Earlier direction, 23 November | Repaired device #2, pi0b | Original, pi0a |',
 '| Reverse direction, 23 November | Original, pi0a | Repaired device #2, pi0b |',
 '| Historical reference, 21 November | Original, pi0a | Board #1, pi0b |',
 '', f'Across **{len(direction):,} matched points** with fully valid SINAD and clean transport in both directions, median **reverse minus earlier SINAD** is **{statistics.median(direction):+.2f} dB**. Each frequency/rate point carries equal weight; missing estimates are excluded, not assigned zero.',
 '', f'Using the original board as TX in both runs, the median **board #2 RX link minus historical board #1 RX link** is **{statistics.median(receivers):+.2f} dB** across **{len(receivers):,} clean, fully valid pairs**. The historical run used a different software build and playback device, and occurred on a different day. This is a qualified link comparison, not an isolated oscillator-repair result.',
 '', '## Frequency response at 1 MS/s',
 '', '![Three complete links at 1 MS/s](comparison/three-links-1msps.png)',
 '', '[Vector figure](comparison/three-links-1msps.svg). Lines connect measured settings; gaps mean no numeric estimate. The curves include partially valid SINAD windows, whose fractions remain in the paired CSV. Paired statistics below require complete validity and clean transport.',
 '', '| MHz | Earlier SINAD dB | Reverse SINAD dB | Historical board #1 SINAD dB |', '|---:|---:|---:|---:|']
fmt = lambda row: 'Unavailable' if row['sinad_db'] is None else f'{row["sinad_db"]["mean"]:.2f}'
for hz in [30000000,430100000,1000000000,1350000000,1750000000,2000000000,2300000000,2385000000,2399900000,2400000000,2450000000,2494900000,2650000000,3150000000,4650000000,5999999000]:
    key = hz,1000000
    lines.append(f'| {hz/1e6:.3f} | {fmt(fwd[key])} | {fmt(rev[key])} | {fmt(old[key])} |')
lines += ['', '## Paired sample-rate summary', '',
 '| Rate MS/s | Clean full pairs, directions | Median reverse − earlier SINAD dB | Median reverse − earlier RSSI dB | Clean full pairs, board #1 | Median board #2 RX − board #1 RX SINAD dB |',
 '|---:|---:|---:|---:|---:|---:|']
for row in summary:
    lines.append(f'| {labels[row["rate_hz"]]} | {row["clean_full_direction_pairs"]} | {row["median_reverse_minus_forward_sinad_db"]:+.2f} | {row["median_reverse_minus_forward_rssi_db"]:+.2f} | {row["clean_full_board1_pairs"]} | {row["median_board2rx_minus_board1rx_sinad_db"]:+.2f} |')
lines += ['', 'RSSI differences require clean transport at both points. SINAD differences additionally require numeric estimates throughout both windows, so the RSSI and SINAD populations can differ. Full validity fractions and flags are retained in [paired-results.csv](comparison/paired-results.csv).',
 '', '## Frequency-region comparison', '',
 '| Frequency region | Direction pairs | Median reverse − earlier SINAD dB | Board #1 pairs | Median board #2 RX − board #1 RX SINAD dB |',
 '|---|---:|---:|---:|---:|']
for row in regions:
    lines.append(f'| {row["region"]} | {row["direction_pairs"]} | {row["median_reverse_minus_forward_sinad_db"]:+.2f} | {row["board1_pairs"]} | {row["median_board2rx_minus_board1rx_sinad_db"]:+.2f} |')
lines += ['', 'These regions separate the native RF24 bypass band from the frequency settings below and above it. All eight rates contribute equally per retained frequency/rate point. Clustered frequency anchors remain equally weighted, so the figures are descriptive grid summaries rather than uniform-band integrals.',
 '', '## Direction differences across the grid',
 '', '![Direction differences across frequency and rate](comparison/direction-differences.png)',
 '', '[Vector figure](comparison/direction-differences.svg). Positive red means the reverse direction reports a higher value; negative blue means the earlier direction reports a higher value. Gray cells are excluded pairs. The actual nonuniform frequency spacing is preserved.',
 '', '## Acquisition and estimator availability',
 '', '| Dataset | Matrix points | No numeric SINAD | Partially valid SINAD | Transport-flagged windows | Failed setup attempts |',
 '|---|---:|---:|---:|---:|---:|']
for label, row in health.items():
    lines.append(f'| {label} | {row["points"]} | {row["no_numeric_sinad"]} | {row["partial_sinad"]} | {row["transport_flagged"]} | {row["failed_setup_attempts"]} |')
lines += [
 '', 'A failed setup attempt is distinct from a retained measurement-window error. All three datasets contain the complete matrix; fault and retry details remain in each dataset’s progress and controller logs. An unavailable estimate does not establish absent RF.',
 '', '## What this comparison establishes',
 '', 'Reversing the pair tests whether the complete link is symmetric. It changes both the transmitter and receiver roles together. A difference cannot uniquely identify TX modulation quality, actual RF output power, RX noise/distortion, oscillator behavior, conversion spurs or external interference. Requested 0 dBm and nominal attenuation are not independently calibrated power at the receiver connector.',
 '', 'The two directions ran sequentially, so environmental changes and temperature are also possible contributors. Sample rates changed on both boards together. The measurements do not establish a standalone receiver-sensitivity or manufacturing-quality ranking.',
 '', '## Evidence and regeneration',
 '', '- [Reverse sweep report](REPORT.md) and [point CSV](results.csv).',
 '- [Earlier direction report](../20261123-pi0-board2-tx-frequency-rate-sweep/REPORT.md).',
 '- [Historical board #1 report](../20261121-pi0-board1-frequency-rate-sweep/REPORT.md).',
 '- [Comparison summary](comparison/summary.json), [paired CSV](comparison/paired-results.csv), and [source](comparison/analyze_comparison.py).',
 '', 'From this baseline directory, with NumPy and Matplotlib installed:', '', '```sh',
 'MPLCONFIGDIR=/tmp/cariboulite-matplotlib python3 comparison/analyze_comparison.py .', '```', '']
(ROOT/'COMPARISON.md').write_text('\n'.join(lines))
(DEST/'analyze_comparison.py').write_bytes(pathlib.Path(__file__).read_bytes())
(DEST/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(DEST.iterdir()) if p.is_file() and p.name!='SHA256SUMS')+hashlib.sha256((ROOT/'COMPARISON.md').read_bytes()).hexdigest()+'  ../COMPARISON.md\n')
print(json.dumps(metrics,indent=2),flush=True)
