#!/usr/bin/env python3
"""CTCSS configuration/control/status tests. Every backend uses --dry-run."""
import importlib.util
from pathlib import Path
import subprocess
import unittest
spec = importlib.util.spec_from_file_location("ctcss_helpers", Path(__file__).with_name("test_tx_control_backend.py"))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)

class CtcssBackendTests(unittest.TestCase):
    setUpClass = helper.TxControlBackendTests.__dict__["setUpClass"]
    setUp = helper.TxControlBackendTests.setUp
    stop_backend = staticmethod(helper.TxControlBackendTests.stop_backend)
    start_backend = helper.TxControlBackendTests.start_backend
    connect = helper.TxControlBackendTests.connect
    controller = helper.TxControlBackendTests.controller
    snapshot_matching = helper.TxControlBackendTests.snapshot_matching
    command = helper.TxControlBackendTests.command
    exchange = helper.TxControlBackendTests.exchange
    wait_state = helper.TxControlBackendTests.wait_state

    def test_defaults_and_controller(self):
        self.start_backend(start=False)
        status=self.connect(self.status_path)
        cfg=self.snapshot_matching(status)["config"]["ctcss"]
        self.assertFalse(cfg['rx_enabled']);self.assertFalse(cfg['tx_enabled'])
        self.assertEqual(cfg['tx_deviation_hz'],500)
        self.assertFalse(cfg['valid']);self.assertIsNone(cfg['detected'])
        result=subprocess.run([str(helper.CONTROLLER),'--socket',str(self.control_path)],
            input='rx ctcss 123.0\ntx ctcss 88.5\ntx ctcss deviation 350\nq\n',capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(result.stderr,'')
        self.assertIn('RX CTCSS: 123.0',result.stdout)
        self.assertIn('TX CTCSS: 88.5',result.stdout)
        snap=self.snapshot_matching(status,lambda s:s['config']['ctcss']['tx_deviation_hz']==350)
        cfg=snap['config']['ctcss'];self.assertEqual(cfg['rx_frequency_hz'],123)
        self.assertEqual(cfg['tx_frequency_hz'],88.5)
        self.assertTrue(cfg['rx_enabled']);self.assertTrue(cfg['tx_enabled'])
        rendered='\n'.join(helper.helpers.monitor.format_snapshot(snap))
        self.assertIn('CTCSS: RX=enabled',rendered);self.assertIn('TX=enabled',rendered)

    def test_independence_mode_bypass_and_locks(self):
        self.start_backend('--rx-ctcss','100','--tx-ctcss','88.5',start=False)
        peer=self.controller();status=self.connect(self.status_path)
        self.command(peer,'mode am');self.command(peer,'tx_mode am')
        snap=self.snapshot_matching(status,lambda s:s['config']['tx_mode']=='am')
        cfg=snap['config']['ctcss'];self.assertFalse(cfg['rx_enabled']);self.assertFalse(cfg['tx_enabled'])
        self.assertEqual(cfg['rx_frequency_hz'],100);self.assertEqual(cfg['tx_frequency_hz'],88.5)
        self.command(peer,'mode nbfm');self.command(peer,'tx_mode nbfm')
        self.command(peer,'start')
        self.command(peer,'rx_ctcss 123',code='RX_RUNNING')
        self.command(peer,'stop')
        self.command(peer,'tx_ctcss 0')
        self.command(peer,'tx_source tone')
        self.command(peer,'tx_start',code='ACCEPTED');self.wait_state(peer,'TX')
        self.command(peer,'tx_ctcss 100',code='TX_RUNNING')
        self.command(peer,'tx_ctcss_deviation 400',code='TX_RUNNING')
        self.command(peer,'tx_stop',code='ACCEPTED');self.wait_state(peer,'IDLE')
        self.command(peer,'rx_ctcss 0')
        snap=self.snapshot_matching(status,lambda s:s['config']['ctcss']['rx_frequency_hz']==0)
        self.assertEqual(snap['config']['ctcss']['tx_frequency_hz'],0)

    def test_invalid_wire_and_launch_budget(self):
        self.start_backend('--tx-deviation','500',start=False)
        peer=self.controller()
        for wire in ('rx_ctcss nan','rx_ctcss inf','rx_ctcss 66.9','tx_ctcss 254.2',
                     'tx_ctcss_deviation 49','tx_ctcss_deviation 1001','tx_ctcss 100 extra'):
            self.command(peer,wire,code='PROTOCOL')
        self.command(peer,'tx_ctcss 100',code='INVALID')
        self.command(peer,'tx_ctcss_deviation 250');self.command(peer,'tx_ctcss 100')
        self.command(peer,'tx_ctcss_deviation 500',code='INVALID')
        for arguments in (('--rx-ctcss','nan'),('--tx-ctcss','66'),('--tx-ctcss-deviation','0'),
                          ('--tx-deviation','400','--tx-ctcss','100')):
            result=subprocess.run([str(helper.helpers.BACKEND),'--dry-run',*arguments],capture_output=True,timeout=3)
            self.assertNotEqual(result.returncode,0)

if __name__=='__main__':unittest.main()
