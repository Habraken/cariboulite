#!/usr/bin/env python3
"""Compressor launch/control/status tests; every backend is simulated."""
import importlib.util
from pathlib import Path
import subprocess
import unittest
spec = importlib.util.spec_from_file_location("tx_control_helpers", Path(__file__).with_name("test_tx_control_backend.py"))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class TxDynamicsBackendTests(unittest.TestCase):
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
        status = self.connect(self.status_path)
        cfg = self.snapshot_matching(status)["config"]["tx_compressor"]
        self.assertTrue(cfg["enabled"])
        self.assertEqual(cfg["threshold_dbfs"], -24)
        self.assertEqual(cfg["ratio"], 3)
        result = subprocess.run([str(helper.CONTROLLER), "--socket", str(self.control_path)],
            input="tx compressor threshold -30\ntx compressor ratio 4\ntx compressor on\nq\n",
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        self.assertIn("TX compressor enabled: 1", result.stdout)
        snap = self.snapshot_matching(status, lambda s: s["config"]["tx_compressor"]["enabled"]
            and s["config"]["tx_compressor"]["threshold_dbfs"] == -30
            and s["config"]["tx_compressor"]["ratio"] == 4)
        self.assertEqual(snap["config"]["tx_compressor"]["threshold_dbfs"], -30)
        self.assertEqual(snap["config"]["tx_compressor"]["ratio"], 4)
        rendered = "\n".join(helper.helpers.monitor.format_snapshot(snap))
        self.assertIn("TX microphone compressor: enabled", rendered)

    def test_default_on_in_all_voice_modes_and_tone_bypass_status(self):
        for mode in ("nbfm", "am", "usb", "lsb"):
            with self.subTest(mode=mode):
                process = self.start_backend("--tx-mode", mode, "--tx-source", "tone", start=False)
                status = self.connect(self.status_path)
                snap = self.snapshot_matching(status)
                self.assertTrue(snap["config"]["tx_compressor"]["enabled"])
                self.assertEqual(snap["config"]["tx_mode"], mode)
                rendered = "\n".join(helper.helpers.monitor.format_snapshot(snap))
                self.assertIn("TX microphone compressor: bypassed", rendered)
                status.close()
                self.stop_backend(process)

    def test_explicit_off_survives_mode_and_source_changes(self):
        self.start_backend("--tx-compressor", "off", start=False)
        peer = self.controller()
        status = self.connect(self.status_path)
        self.assertFalse(self.snapshot_matching(status)["config"]["tx_compressor"]["enabled"])
        for mode in ("am", "usb", "lsb", "nbfm"):
            self.command(peer, "tx_mode " + mode)
        self.command(peer, "tx_source tone")
        self.command(peer, "tx_source mic")
        self.command(peer, "tx_compressor ratio 4")
        snap = self.snapshot_matching(status, lambda s: s["config"]["tx_compressor"]["ratio"] == 4)
        self.assertFalse(snap["config"]["tx_compressor"]["enabled"])

    def test_launch_and_stopped_only_edits(self):
        self.start_backend("--tx-compressor", "on", "--tx-comp-input", "2",
            "--tx-comp-threshold", "-28", "--tx-comp-ratio", "4", "--tx-comp-knee", "8",
            "--tx-comp-attack", "8", "--tx-comp-release", "150", "--tx-comp-makeup", "9",
            "--tx-comp-ceiling", "-2", start=False)
        status = self.connect(self.status_path)
        snap = self.snapshot_matching(status)
        cfg = snap["config"]["tx_compressor"]
        for key, value in dict(enabled=True, input_db=2, threshold_dbfs=-28, ratio=4,
                knee_db=8, attack_ms=8, release_ms=150, makeup_db=9, ceiling_dbfs=-2).items():
            self.assertEqual(cfg[key], value)
        peer = self.controller()
        before = self.command(peer, "get")
        same = self.command(peer, "tx_compressor ratio 4")
        self.assertEqual(before["config_generation"], same["config_generation"])
        for param, value in [("input", -3), ("threshold", -20), ("ratio", 2), ("knee", 0),
                ("attack", .1), ("release", 1000), ("makeup", 12), ("ceiling", -3)]:
            self.command(peer, f"tx_compressor {param} {value}")
        self.command(peer, "tx_start", code="ACCEPTED")
        self.wait_state(peer, "TX")
        self.command(peer, "tx_compressor enabled 0", code="TX_RUNNING")
        self.command(peer, "tx_compressor ratio 6", code="TX_RUNNING")
        self.command(peer, "tx_stop", code="ACCEPTED")
        self.wait_state(peer, "IDLE")
        self.command(peer, "tx_compressor enabled 0")
        off = self.snapshot_matching(status, lambda s: not s["config"]["tx_compressor"]["enabled"])
        self.assertEqual(off["config"]["tx_compressor"]["ceiling_dbfs"], -3)
        self.assertEqual(off["config"]["tx_compressor"]["ratio"], 2)

    def test_invalid_wire_has_no_effect(self):
        self.start_backend(start=False)
        peer = self.controller()
        before = self.command(peer, "get")["config_generation"]
        for text in ["ratio 0", "enabled 2", "attack 0", "release 0", "ceiling 1",
                "makeup -1", "input 25", "threshold -61", "knee 25", "ratio nan",
                "ratio inf", "unknown 1", "ratio 3 extra"]:
            self.command(peer, "tx_compressor " + text, code="PROTOCOL")
        self.assertEqual(before, self.command(peer, "get")["config_generation"])

    def test_invalid_launch_rejected(self):
        for option, value in [("ratio", "0"), ("attack", "nan"), ("release", "0"),
                ("ceiling", "1"), ("makeup", "-1"), ("input", "25"),
                ("threshold", "-61"), ("knee", "25")]:
            result = subprocess.run([str(helper.helpers.BACKEND), "--dry-run",
                "--tx-comp-" + option, value], capture_output=True, timeout=3)
            self.assertEqual(result.returncode, 2, (option, result.stderr))


if __name__ == "__main__":
    unittest.main()
