"""Check retained board5 startup evidence, roles, failures and lack of RX measurements."""
import json,pathlib,sys
root=pathlib.Path(sys.argv[1]);rates={1:4000000,2:2000000,3:1333333,4:1000000,5:800000,6:666667,8:500000,10:400000}
failures=json.loads((root/'startup-failures.json').read_text());assert sorted(r['rx_rate_hz'] for r in failures)==sorted(rates.values())
assert not (root/'points.jsonl').read_text().strip()
for divider,rate in rates.items():
 case=root/('rx_div'+format(divider,'X'));ready=json.loads((case/'rx-ready.json').read_text())['metadata']
 assert ready['board_uuid']=='b43713d5-7ebf-45ff-b327-aa2e0962c9e2' and ready['host']=='pi4f-trixie'
 commands=[json.loads(s) for s in (case/'rx/commands.jsonl').read_text().splitlines()]
 assert any(c['request'].endswith(' start') and ' ERROR HARDWARE ' in c['reply'] for c in commands)
 assert not any('tx_start' in c['request'] for c in commands)
 assert 'SMI data synchronization failed' in (case/'rx/backend.log').read_text()
ready=json.loads((root/'rx_axis_source/tx-ready.json').read_text())['metadata'];assert ready['board_uuid']=='c132d0e7-d7dd-457a-b105-4a5cd97e7e80' and ready['host']=='pi4g-trixie'
print(json.dumps({'board_label':'#5','startup_attempts':8,'startup_failures':8,'measurement_points':0,'rx_uuid_and_host_checked':True,'tx_reference_checked':True,'no_rx_tx_start_commands':True,'all_failure_logs_contain_smi_sync_error':True}))
