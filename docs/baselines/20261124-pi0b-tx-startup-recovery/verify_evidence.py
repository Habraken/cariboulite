"""Offline verification of conducted TX startup recovery; never accesses RF."""
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import tarfile
from zoneinfo import ZoneInfo

root=Path(__file__).resolve().parent
archive=root/'host-artifacts.tar.gz'
phases=[('before_reload','stress-000744',26,1),('after_reload','program-000924',60,0),
        ('after_backend_restart','stress-001148',26,0)]
summary={'board_uuid':'36b67ac4-b313-4369-99bd-4b3e09caabfa','host':'pi0b-trixie',
         'rf_link':'operator-confirmed nominal 90 dB conducted HiF link',
         'frequency_hz':430100000,'requested_power_dbm':-12,
         'signal':'NBFM 1000 Hz internal tone, amplitude 0.4; deviation setting 2500 Hz',
         'phases':[],'after_reload_starts':86,'after_reload_failures':0,
         'root_cause_confirmed':False,'production_source_changed':False}
with tarfile.open(archive,'r:gz') as tar:
    def read(directory,name):
        return tar.extractfile('tx-startup-20261124/'+directory+'/'+name).read().decode()
    for label,directory,expected_count,expected_failures in phases:
        results=json.loads(read(directory,'results.json'))
        meta=json.loads(read(directory,'metadata.json'))
        events=[json.loads(line) for line in read(directory,'events.jsonl').splitlines()]
        statuses=[e['value'] for e in events if e['kind']=='status']
        failures=[r for r in results if r['start_state']!='TX']
        assert len(results)==expected_count and len(failures)==expected_failures
        assert [r['cycle'] for r in results]==list(range(1,expected_count+1))
        assert statuses[-1]['state']=='IDLE' and not statuses[-1]['tx']['running']
        for r in results:
            assert r['source']=='tone'
            s=r['snapshot'];assert s['config']['tx_source']=='tone'
            assert s['config']['tx_frequency_hz']==430100000
            assert s['config']['tx_power_dbm']==-12
            assert s['config']['tx_sample_rate_hz']==r['rate']
            if r['start_state']=='TX':
                tx=s['tx'];assert tx['running'] and tx['driver']['completed_periods']>0
                assert tx['driver']['fifo_dequeued_bytes']>0 and tx['writer']['samples']>0
                assert tx['fault'] is None
                assert s['hardware']['rf24']['registers']['STATE']==4
                for worker in ('producer','writer'):
                    w=tx['workers'][worker]
                    assert w['alive'] and w['last_error']==0
                    assert w['scheduling']['rt_error']==0 and w['scheduling']['policy_name']=='SCHED_FIFO'
            else:
                assert s['tx']['fault']=='TX_PIPELINE_START'
                d=s['tx']['driver']
                assert d['completed_periods']==0 and d['fifo_dequeued_bytes']==0
                assert d['fifo_queued_bytes']==d['fifo_limit_bytes']==262144
        drivers=[s['tx']['driver'] for s in statuses if s['tx']['driver'].get('valid')]
        max_underruns=max(d['underrun_periods'] for d in drivers)
        if not expected_failures:assert max_underruns==0
        summary['phases'].append({'label':label,'directory':directory,'starts':len(results),
            'failures':len(failures),'failed_cycles':[r['cycle'] for r in failures],
            'rate_counts':dict(sorted(Counter(r['rate'] for r in results).items())),
            'status_snapshots':len(statuses),'max_recorded_driver_underruns':max_underruns,
            'final_state':statuses[-1]['state'],'final_tx_running':statuses[-1]['tx']['running'],
            'start_brussels':datetime.fromtimestamp(meta['started'],ZoneInfo('Europe/Brussels')).isoformat(),
            'binary_sha256':meta['binary_sha256'],'library_sha256':meta['library_sha256'],
            'fpga_sha256':meta.get('fpga_sha256')})
        if label=='after_reload':
            log=read(directory,'backend.log')
            assert 'FPGA programming - Success!' in log
            assert '[radio_backend] FPGA programmed from firmware/top.bin' in log
    # These runs did not exercise the original DMA stall: the headset was on
    # pi0a, and the initial harness retained microphone mode. Preserve them,
    # but exclude every setup failure from the valid conducted comparison.
    excluded=json.loads(read('stress-000628','results.json'))
    assert len(excluded)==31
    assert all(r['snapshot']['tx']['fault']=='TX_INIT' for r in excluded)
    summary['excluded_audio_setup_failures']=31
release=json.loads((root/'release.json').read_text())
assert release['uuid']==summary['board_uuid'] and release['owners']==[]
assert release['smi_fuser_exit']==1 and not release['smi_fuser_stdout'] and not release['smi_fuser_stderr']
summary['release_verified']=True
assert len({p['binary_sha256'] for p in summary['phases']})==1
assert len({p['library_sha256'] for p in summary['phases']})==1
(root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
