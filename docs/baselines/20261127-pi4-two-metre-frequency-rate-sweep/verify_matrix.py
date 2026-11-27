"""Recheck completed acquisition against its plan, raw samples, and CSV export."""
import collections
import csv
import gzip
import json
import math
import pathlib
import statistics
import sys
import tarfile

ROOT = pathlib.Path(sys.argv[1])
plan = json.loads((ROOT / 'plan.json').read_text())
progress = json.loads((ROOT / 'progress.json').read_text())
verification = json.loads((ROOT / 'verification.json').read_text())
rows = [json.loads(line) for line in (ROOT / 'results.jsonl').read_text().splitlines()]
matrix = [row for row in rows if row['phase'] == 'matrix']
errors = []

def check(ok, message):
    if not ok:
        errors.append(message)

expected = {(p['point'], p['frequency_hz'], p['sample_rate_hz']) for p in plan['points']}
actual = [(r['point'], r['frequency_hz'], r['sample_rate_hz']) for r in matrix]
check(len(expected) == 1376, 'Unexpected plan size')
check(set(actual) == expected and len(actual) == len(expected), 'Incomplete or duplicated matrix')
check(progress['state'] in ('complete', 'complete_with_failures'), 'Acquisition did not complete')
check(verification['passed'], 'Host/snapshot verification failed')
check(len([r for r in rows if r['phase'] == 'preflight']) == 8, 'Preflight incomplete')
lookup = {(r['phase'], r['point']): r for r in rows}
check(len(lookup) == len(rows), 'Duplicate result identifiers')
samples = collections.defaultdict(list)
with gzip.open(ROOT / 'paired-snapshots.jsonl.gz', 'rt') as stream:
    for line in stream:
        d = json.loads(line)
        key = d['phase'], d['point']
        samples[key].append(d)
        r, t = d['rx'], d['tx']
        checks = {
            'roles': r['state'] == 'RX' and t['state'] == 'TX' and not r['tx']['running'],
            'TX fault': not t['tx'].get('fault'),
            'power': t['config']['tx_power_dbm'] == t['config']['tx_actual_power_dbm'] == 0,
            'frequency': r['config']['requested_frequency_hz'] == t['config']['tx_frequency_hz'] == d['frequency_hz'],
            'rate': r['config']['sample_rate_hz'] == t['config']['tx_sample_rate_hz'] == d['sample_rate_hz'],
            'tone': t['config']['tx_tone_hz'] == 1000 and abs(t['config']['tx_tone_amp'] - .4) < 1e-6,
        }
        for role, snap in [('rx', r), ('tx', t)]:
            c = snap['config']
            checks[role + ' signalling'] = (c['ctcss']['rx_frequency_hz'] == c['ctcss']['tx_frequency_hz'] == 0
                and c['cdcss']['rx_code'] == c['cdcss']['tx_code'] == 'off')
        for label, ok in checks.items():
            check(ok, f'{key}: {label}')
        # Retain only the fields needed below, avoiding a full multi-hour status
        # history in memory while preserving the original compressed evidence.
        rssi = r['hardware']['rf24']['registers'].get('RSSI')
        for role, snap in [('rx', r), ('tx', t)]:
            keep = {name: snap[name] for name in ('sequence', 'monotonic_s', role)}
            if role == 'rx':
                keep['hardware'] = {'rf24': {'registers': {'RSSI': rssi}}}
            snap.clear()
            snap.update(keep)

counter_keys = {
    'rx': [('queue', 'drops'), ('audio_queue', 'drops'), ('playback', 'underruns'),
           ('playback', 'write_errors'), ('reader', 'errors'), ('alignment', 'failures'),
           ('alignment', 'invalid_frames')],
    'tx': [('driver', 'underrun_periods'), ('writer', 'errors'), ('queue', 'drops')],
}
flagged = []
for key, row in lookup.items():
    pairs = samples[key]
    check(len(pairs) == row['snapshots'] and len(pairs) >= 15, f'{key}: sample count')
    if not pairs:
        continue
    values = {'rssi_dbm': [], 'sinad_db': []}
    for pair in pairs:
        value = pair['rx']['hardware']['rf24']['registers'].get('RSSI')
        if isinstance(value, int) and value != 127:
            values['rssi_dbm'].append(value - 256 if value > 127 else value)
        value = pair['rx']['rx']['sinad'].get('db')
        if isinstance(value, (int, float)) and math.isfinite(value):
            values['sinad_db'].append(value)
    for field, v in values.items():
        check((row[field] is not None) == bool(v), f'{key}: {field} availability')
        if v and row[field]:
            computed = dict(n=len(v), mean=statistics.mean(v), median=statistics.median(v),
                            min=min(v), max=max(v), stdev=statistics.stdev(v) if len(v) > 1 else 0)
            for metric, value in computed.items():
                check(abs(value - row[field][metric]) < 1e-8, f'{key}: {field}.{metric}')
    check(abs(len(values['sinad_db']) / len(pairs) - row['sinad_valid_fraction']) < 1e-10,
          f'{key}: validity fraction')
    clean = True
    for role in ('rx', 'tx'):
        for field in ('sequence', 'monotonic_s'):
            v = [p[role][field] for p in pairs]
            check(all(b > a for a, b in zip(v, v[1:])), f'{key}: {role} {field} freshness')
        for group, name in counter_keys[role]:
            v = [p[role][role][group][name] for p in pairs]
            computed = dict(first=v[0], last=v[-1], min=min(v), max=max(v), delta=v[-1] - v[0],
                            reset=any(b < a for a, b in zip(v, v[1:])))
            check(computed == row[role + '_errors'][group + '.' + name], f'{key}: {role} {group}.{name}')
            if len(set(v)) > 1:
                clean = False
    check(clean == row['transport_clean'], f'{key}: transport flag')
    if not clean:
        flagged.append(key)
check(set(samples) == set(lookup), 'Raw samples and result identifiers disagree')

csvrows = list(csv.DictReader((ROOT / 'results.csv').open()))
check(len(csvrows) == len(rows), 'CSV row count')
for flat, row in zip(csvrows, rows):
    check(flat['phase'] == row['phase'], 'CSV phase mismatch')
    for field in ('point', 'frequency_hz', 'sample_rate_hz', 'snapshots', 'tx_power_dbm'):
        check(int(flat[field]) == row[field], f'CSV {field} mismatch')
    for prefix, field in [('rssi', 'rssi_dbm'), ('sinad', 'sinad_db')]:
        for metric in ('mean', 'min', 'max'):
            column = f'{prefix}_{metric}_' + ('dbm' if prefix == 'rssi' else 'db')
            if row[field] is None:
                check(flat[column] == '', f'CSV missing value not blank: {column}')
            else:
                check(abs(float(flat[column]) - row[field][metric]) < 1e-8, f'CSV {column}')
    check(flat['transport_clean'] == str(row['transport_clean']), 'CSV transport flag')

host_audits = {}
for role, host, uuid in [('rx', 'pi4f-trixie', '36b67ac4-b313-4369-99bd-4b3e09caabfa'),
                         ('tx', 'pi4g-trixie', 'c132d0e7-d7dd-457a-b105-4a5cd97e7e80')]:
    sessions = []
    initial_build = json.loads((ROOT / (role + '-session1-ready.json')).read_text())['metadata']
    with tarfile.open(ROOT / (role + '-host-artifacts.tar.gz')) as archive:
        for member in archive.getmembers():
            if not member.name.endswith('/metadata.json'):
                continue
            meta = json.load(archive.extractfile(member))
            check((meta['host'], meta['board_uuid'], meta['role']) == (host, uuid, role),
                  'Host metadata identity ' + member.name)
            check(meta['diagnostic_preload'] is False, 'Diagnostic preload ' + member.name)
            check(meta['git_revision'] == plan['software_revision'], 'GitHub revision ' + member.name)
            check(meta['library_sha256'] == initial_build['library_sha256'], 'Library changed ' + member.name)
            check(meta['binary_sha256'] == initial_build['binary_sha256'],
                  'Backend changed ' + member.name)
            session = dict(path=member.name, binary_sha256=meta['binary_sha256'],
                           library_sha256=meta['library_sha256'])
            if role == 'rx':
                restored_name = str(pathlib.PurePosixPath(member.name).parent / 'led-restored.json')
                try:
                    restored = json.load(archive.extractfile(restored_name))
                    initial = int(meta['led_setup']['before_reply'].split()[-1])
                    actual = int(restored['reply'].split()[-1])
                    check(restored['initial_pins'] == initial and (actual & 3) == (initial & 3),
                          'LED restoration ' + restored_name)
                    session['led_restoration_verified'] = True
                except KeyError:
                    check(False, 'Missing LED restoration ' + restored_name)
            sessions.append(session)
    check(bool(sessions), 'No host metadata ' + role)
    host_audits[role] = sessions
for role, sessions in host_audits.items():
    check(len({s['library_sha256'] for s in sessions}) == 1, 'Radio library changed within host ' + role)

result = dict(passed=not errors, errors=errors, matrix_points=len(matrix), host_audits=host_audits,
              planned_points=len(expected), total_result_rows=len(rows),
              raw_paired_snapshots=sum(map(len, samples.values())),
              transport_flagged=flagged, recovered_setup_failures=progress['failures'])
(ROOT / 'matrix-verification.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
sys.exit(0 if result['passed'] else 1)
