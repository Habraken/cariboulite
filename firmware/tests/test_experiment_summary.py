#!/usr/bin/env python3
"""Check experiment summaries against complete, failed and absent run fixtures."""

import csv
import importlib.util
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools/summarize_fpga_experiments.py"
SPEC = importlib.util.spec_from_file_location("experiment_summary", SCRIPT)
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


ROUTING_LOG = """Info: Device utilisation:
Info:          ICESTORM_LC: 1022/1280 79%
Info:         ICESTORM_RAM: 16/16 100%
Info: Max frequency for clock 'system': 90.00 MHz (PASS at 62.50 MHz)
Info: Max frequency for clock 'lvds': 70.00 MHz (PASS at 64.00 MHz)
Info: Routing complete.
Info: Max frequency for clock 'system': 73.10 MHz (PASS at 62.50 MHz)
ERROR: Max frequency for clock 'lvds': 56.24 MHz (FAIL at 64.00 MHz)
Warning: example routing warning
"""

REAL_PADDED_ROUTING_LOG = """Info: Max frequency for clock             'i_sck$SB_IO_IN': 185.91 MHz (PASS at 5.00 MHz)
Info: Max frequency for clock      'i_glob_clock$SB_IO_IN': 463.61 MHz (PASS at 125.00 MHz)
Info: Max frequency for clock 'smi_ctrl_ins.soe_and_reset': 168.58 MHz (PASS at 16.00 MHz)
Info: Max frequency for clock                  'r_counter': 78.20 MHz (PASS at 62.50 MHz)
Info: Max frequency for clock             'lvds_clock_buf': 72.26 MHz (PASS at 64.00 MHz)
Info: Routing complete.
Info: Max frequency for clock             'i_sck$SB_IO_IN': 176.52 MHz (PASS at 5.00 MHz)
Info: Max frequency for clock      'i_glob_clock$SB_IO_IN': 463.61 MHz (PASS at 125.00 MHz)
Info: Max frequency for clock 'smi_ctrl_ins.soe_and_reset': 169.95 MHz (PASS at 16.00 MHz)
Info: Max frequency for clock                  'r_counter': 75.90 MHz (PASS at 62.50 MHz)
Info: Max frequency for clock             'lvds_clock_buf': 74.96 MHz (PASS at 64.00 MHz)
"""


class SummaryTests(unittest.TestCase):
    def test_a3_dimensions_prefer_recorded_options_then_plan_then_old_defaults(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            entry = {"name": "baseline", "synthesis": "ce2", "refinement": "serial"}
            defaults = {"tmg_ripup": False, "promote_globals": False, "synthesis_device": "default"}
            planned = {"tmg_ripup": True, "promote_globals": True, "synthesis_device": "lp"}
            actual = {"tmg_ripup": True, "promote_globals": False, "synthesis_device": "lp"}
            for expected, plan_options, result_options in ((defaults, {}, None), (planned, planned, None),
                                                           (actual, defaults, actual)):
                with self.subTest(expected=expected):
                    if result_options is not None:
                        (directory / "result.json").write_text(json.dumps({
                            "status": "passed", "elapsed_s": 1, **result_options}))
                    run = SUMMARY.summarize_run({**entry, **plan_options}, directory, [])
                    self.assertEqual({field: run[field] for field in defaults}, expected)

    def test_a3_option_combinations_form_separate_groups_and_csv_columns(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runs = []
            for index, (ripup, promote, device) in enumerate(itertools.product(
                    (False, True), (False, True), ("default", "lp"))):
                options = {"tmg_ripup": ripup, "promote_globals": promote, "synthesis_device": device}
                entry = {"name": f"variant-{index}", "synthesis": "ce2", "refinement": "serial", **options}
                runs.append(SUMMARY.summarize_run(entry, root / entry["name"], []))
            legacy = SUMMARY.summarize_run({"name": "legacy", "synthesis": "ce2", "refinement": "serial"},
                                           root / "legacy", [])
            runs.append(legacy)
            groups = SUMMARY.grouped(runs)
            self.assertEqual(len(groups), 8)
            self.assertEqual(sum(group["planned"] for group in groups), 9)
            for group in groups:
                expected = 2 if not group["tmg_ripup"] and not group["promote_globals"] and group["synthesis_device"] == "default" else 1
                self.assertEqual(group["planned"], expected)
            csv_path = root / "summary.csv"
            SUMMARY.write_csv(csv_path, runs)
            with csv_path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 9)
            self.assertEqual(len({(row["tmg_ripup"], row["promote_globals"], row["synthesis_device"])
                                  for row in rows}), 8)
            self.assertEqual((rows[-1]["tmg_ripup"], rows[-1]["promote_globals"], rows[-1]["synthesis_device"]),
                             ("False", "False", "default"))

    def test_real_padded_nextpnr_lines_include_all_five_final_clocks(self):
        report = SUMMARY.parse_routing_log(REAL_PADDED_ROUTING_LOG)
        self.assertEqual({name: values["achieved"] for name, values in report["fmax"].items()},
                         {"i_sck$SB_IO_IN": 176.52, "i_glob_clock$SB_IO_IN": 463.61,
                          "smi_ctrl_ins.soe_and_reset": 169.95, "r_counter": 75.9,
                          "lvds_clock_buf": 74.96})
        self.assertTrue(all(item["phase"] == "post_route" for item in report["evidence"]["fmax"].values()))

    def test_routing_log_uses_final_clock_observation_with_evidence(self):
        report = SUMMARY.parse_routing_log(ROUTING_LOG)
        self.assertEqual(report["fmax"]["lvds"], {"achieved": 56.24, "constraint": 64.0})
        self.assertEqual(report["fmax"]["system"]["achieved"], 73.1)
        self.assertEqual(report["utilization"]["ICESTORM_RAM"], {"used": 16, "available": 16})
        self.assertEqual(report["evidence"]["fmax"]["lvds"]["phase"], "post_route")
        self.assertEqual(report["evidence"]["fmax"]["lvds"]["line"], 8)

    def test_metric_priority_and_malformed_report_fallback(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "work").mkdir()
            (root / "logs").mkdir()
            (root / "logs/routing.log").write_text(ROUTING_LOG)
            report = {"fmax": {"system": {"achieved": 80.0, "constraint": 62.5}}}
            (root / "work/timing.json").write_text(json.dumps(report))
            result = {"metrics": {"fmax": {"system": {"achieved": 81.0, "constraint": 62.5}}}}
            metrics, evidence = SUMMARY.metrics_for_run(root, result, [])
            self.assertEqual(metrics["fmax"]["system"]["achieved"], 81.0)
            self.assertEqual(evidence["fmax"]["system"]["origin"], "result_metrics")
            self.assertEqual(evidence["fmax"]["lvds"]["origin"], "routing_log")
            metrics, evidence = SUMMARY.metrics_for_run(root, {}, [])
            self.assertEqual(metrics["fmax"]["system"]["achieved"], 80.0)
            self.assertEqual(evidence["fmax"]["system"]["origin"], "timing_report")
            (root / "work/timing.json").write_text("broken report")
            issues = []
            metrics, evidence = SUMMARY.metrics_for_run(root, {}, issues)
            self.assertEqual(metrics["fmax"]["system"]["achieved"], 73.1)
            self.assertTrue(issues)

    def test_cli_keeps_all_planned_runs_and_writes_comparable_hashes_and_csv(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reference = root / "reference"
            reference.mkdir()
            sources = {"top.v": "module top(); endmodule\n", "io.pcf": "pins\n", "Makefile": "build\n"}
            (reference / "reference.json").write_text(json.dumps({"files": {"source/firmware/" + name: {} for name in sources}}))
            runs = []
            for index, status in enumerate(("passed", "failed", "invalid", "missing", "incomplete")):
                directory = root / "runs" / status
                runs.append({"name": status, "synthesis": "default", "refinement": "parallel", "seed": 16,
                             "repeat": index + 1, "command": [sys.executable, "runner.py", "run", "--output", str(directory)]})
                if status == "missing":
                    continue
                (directory / "work").mkdir(parents=True)
                (directory / "logs").mkdir()
                if status == "invalid":
                    (directory / "result.json").write_text("broken result")
                    continue
                for name, content in sources.items():
                    (directory / "work" / name).write_text(content)
                (directory / "work/top.json").write_text("same netlist\n")
                result = {"status": status, "stages": [{"name": "synthesis", "exit_code": 0, "elapsed_s": 2.0, "status": "passed"}]}
                if status != "incomplete":
                    result["elapsed_s"] = 10.0 + index
                if status == "passed":
                    (directory / "work/top.asc").write_text("placement\n")
                    (directory / "work/top.bin").write_text("image\n")
                    result["metrics"] = {"fmax": {"system": {"achieved": 73.1, "constraint": 62.5},
                                                   "lvds": {"achieved": 74.96, "constraint": 64.0}}}
                else:
                    result["error"] = "routing failed"
                    (directory / "logs/routing.log").write_text(ROUTING_LOG)
                (directory / "result.json").write_text(json.dumps(result))
            plan = root / "plan.json"
            plan.write_text(json.dumps({"reference": str(reference), "runs": runs}))
            output = root / "summary"
            command = [sys.executable, str(SCRIPT), "--plan", str(plan), "--output", str(output)]
            process = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["status_counts"], {status: 1 for status in ("passed", "failed", "invalid", "missing", "incomplete")})
            passed, failed = summary["runs"][:2]
            self.assertEqual(passed["hashes"]["source"], failed["hashes"]["source"])
            self.assertEqual(passed["hashes"]["netlist"], failed["hashes"]["netlist"])
            self.assertIsNotNone(passed["hashes"]["image"])
            self.assertIsNone(failed["hashes"]["image"])
            self.assertEqual(failed["evidence"]["fmax"]["lvds"]["origin"], "routing_log")
            group = summary["groups"][0]
            self.assertEqual(len(summary["groups"]), 1)
            for item in (group, *summary["runs"]):
                self.assertEqual({field: item[field] for field in SUMMARY.EXPERIMENT_DEFAULTS},
                                 SUMMARY.EXPERIMENT_DEFAULTS)
            self.assertEqual(group["planned"], 5)
            self.assertEqual(group["hashes"]["netlist"]["unique_count"], 1)
            self.assertEqual(group["fmax"]["lvds"]["achieved_mhz"], {"min": 56.24, "max": 74.96})
            with (output / "summary.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 5)
            self.assertEqual(rows[1]["lvds_fmax_mhz"], "56.24")
            self.assertEqual(rows[1]["warnings_count"], "1")
            original = (output / "summary.json").read_bytes()
            repeated = subprocess.run(command, text=True, capture_output=True)
            self.assertNotEqual(repeated.returncode, 0)
            self.assertEqual((output / "summary.json").read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
