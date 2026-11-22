#!/usr/bin/env python3
"""CDCSS control/status integration; every backend is simulated."""
import importlib.util
from pathlib import Path
import subprocess
import unittest
spec=importlib.util.spec_from_file_location('cdcss_helpers',Path(__file__).with_name('test_tx_control_backend.py'))
helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)

class CdcssBackendTests(unittest.TestCase):
    setUpClass=helper.TxControlBackendTests.__dict__['setUpClass']
    setUp=helper.TxControlBackendTests.setUp
    stop_backend=staticmethod(helper.TxControlBackendTests.stop_backend)
    start_backend=helper.TxControlBackendTests.start_backend
    connect=helper.TxControlBackendTests.connect
    controller=helper.TxControlBackendTests.controller
    snapshot_matching=helper.TxControlBackendTests.snapshot_matching
    command=helper.TxControlBackendTests.command
    exchange=helper.TxControlBackendTests.exchange
    wait_state=helper.TxControlBackendTests.wait_state

    def test_defaults_and_controller(self):
        self.start_backend(start=False);status=self.connect(self.status_path)
        cfg=self.snapshot_matching(status)['config']['cdcss']
        self.assertEqual(cfg['rx_code'],'off');self.assertEqual(cfg['tx_code'],'off')
        self.assertEqual(cfg['tx_deviation_hz'],250);self.assertFalse(cfg['valid']);self.assertIsNone(cfg['detected'])
        result=subprocess.run([str(helper.CONTROLLER),'--socket',str(self.control_path)],
            input='rx cdcss 023\ntx cdcss 125i\ntx cdcss deviation 200\nq\n',capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(result.stderr,'')
        self.assertIn('RX CDCSS: 023N',result.stdout);self.assertIn('TX CDCSS: 125I',result.stdout)
        snap=self.snapshot_matching(status,lambda s:s['config']['cdcss']['tx_deviation_hz']==200)
        self.assertEqual(snap['config']['cdcss']['rx_code'],'023N');self.assertEqual(snap['config']['cdcss']['tx_code'],'125I')
        rendered='\n'.join(helper.helpers.monitor.format_snapshot(snap))
        self.assertIn('CDCSS: RX=enabled 023N',rendered);self.assertIn('TX=enabled 125I',rendered)

    def test_legacy_aliases_and_launch_help(self):
        self.start_backend('--rx-dcs','023N','--tx-dcs','125I','--tx-dcs-deviation','200',start=False)
        status=self.connect(self.status_path)
        result=subprocess.run([str(helper.CONTROLLER),'--socket',str(self.control_path)],
            input='rx dcs 023N\ntx dcs 125I\ntx dcs deviation 200\nq\n',capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(result.stderr,'')
        self.assertIn('RX CDCSS: 023N',result.stdout);self.assertIn('TX CDCSS: 125I',result.stdout)
        snap=self.snapshot_matching(status,lambda s:s['config']['cdcss']['tx_deviation_hz']==200)
        self.assertEqual(snap['config']['cdcss']['rx_code'],'023N')
        self.assertEqual(snap['config']['cdcss']['tx_code'],'125I')
        peer=self.controller()
        self.command(peer,'rx_dcs 025N');self.command(peer,'tx_dcs 023I')
        self.command(peer,'tx_dcs_deviation 250')
        for binary,options in ((helper.helpers.BACKEND,('--rx-cdcss','--tx-cdcss','--tx-cdcss-deviation')),
                               (helper.CONTROLLER,('rx cdcss','tx cdcss','tx cdcss deviation'))):
            result=subprocess.run([str(binary),'--help'],capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            for option in options:self.assertIn(option,result.stdout+result.stderr)

    def test_mode_bypass_independence_and_locks(self):
        self.start_backend('--rx-cdcss','023N','--tx-cdcss','125I',start=False)
        peer=self.controller();status=self.connect(self.status_path)
        self.command(peer,'mode am');self.command(peer,'tx_mode usb')
        snap=self.snapshot_matching(status,lambda s:s['config']['tx_mode']=='usb')
        self.assertFalse(snap['config']['cdcss']['rx_enabled']);self.assertFalse(snap['config']['cdcss']['tx_enabled'])
        self.assertEqual(snap['config']['cdcss']['tx_code'],'125I')
        self.command(peer,'mode nbfm');self.command(peer,'tx_mode nbfm');self.command(peer,'start')
        self.command(peer,'rx_cdcss off',code='RX_RUNNING');self.command(peer,'stop')
        self.command(peer,'tx_source tone');self.command(peer,'tx_start',code='ACCEPTED');self.wait_state(peer,'TX')
        self.command(peer,'tx_cdcss off',code='TX_RUNNING');self.command(peer,'tx_cdcss_deviation 200',code='TX_RUNNING')
        self.command(peer,'tx_stop',code='ACCEPTED');self.wait_state(peer,'IDLE')
        self.command(peer,'tx_cdcss off');self.command(peer,'rx_cdcss 000I')
        snap=self.snapshot_matching(status,lambda s:s['config']['cdcss']['rx_code']=='000I')
        self.assertEqual(snap['config']['cdcss']['tx_code'],'off')

    def test_exclusion_and_budget(self):
        self.start_backend('--rx-ctcss','88.5','--tx-cdcss','023',start=False)
        peer=self.controller()
        before=self.command(peer,'get')
        conflict=self.command(peer,'rx_cdcss 023N',code='CONFLICT');self.assertEqual(before['config_generation'],conflict['config_generation'])
        self.command(peer,'tx_ctcss 88.5',code='CONFLICT')
        self.command(peer,'rx_ctcss 0');self.command(peer,'rx_cdcss 023N')
        self.command(peer,'rx_ctcss 88.5',code='CONFLICT')
        self.command(peer,'tx_cdcss off');self.command(peer,'tx_ctcss 88.5')

    def test_invalid_wire_cli_and_small_deviation(self):
        self.start_backend('--tx-deviation','200',start=False);peer=self.controller()
        self.command(peer,'tx_cdcss 023N',code='INVALID')
        self.command(peer,'tx_cdcss_deviation 100');self.command(peer,'tx_cdcss 023N')
        self.command(peer,'tx_cdcss_deviation 200',code='INVALID')
        for wire in ('rx_cdcss 23','rx_cdcss 089','tx_cdcss 023B','rx_cdcss on','tx_cdcss 0023','tx_cdcss 023N extra','tx_cdcss_deviation nan','tx_cdcss_deviation 49','tx_cdcss_deviation 1001'):
            self.command(peer,wire,code='PROTOCOL')
        for args in (('--rx-cdcss','89'),('--tx-cdcss','023B'),('--rx-cdcss','023','--rx-ctcss','88.5'),
                     ('--tx-cdcss','023','--tx-ctcss','88.5'),('--tx-cdcss-deviation','0'),('--tx-deviation','200','--tx-cdcss','023')):
            result=subprocess.run([str(helper.helpers.BACKEND),'--dry-run',*args],capture_output=True,timeout=3)
            self.assertNotEqual(result.returncode,0)

if __name__=='__main__':unittest.main()
