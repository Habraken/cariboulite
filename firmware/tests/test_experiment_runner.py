#!/usr/bin/env python3
"""Exercise isolated FPGA experiments using temporary repos and fake tools.

No synthesis, routing, hardware access, or installed firmware changes are needed.
Run with ``python3 firmware/tests/test_experiment_runner.py``.
"""

from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import unittest


RUNNER = Path(__file__).resolve().parents[1] / "tools" / "fpga_experiment.py"

FAKE_TOOL = r'''#!/usr/bin/env python3
from pathlib import Path
import json
import os
import shlex
import sys
import time

name = Path(sys.argv[0]).name
stage = {"yosys": "synthesis", "nextpnr-ice40": "routing", "icepack": "packing"}[name]
args = sys.argv[1:]
if (name == "yosys" and ("-p" not in args or args[args.index("-p") + 1].startswith("help "))) or (name == "nextpnr-ice40" and "--json" not in args) or (name == "icepack" and len(args) != 2):
    print(name + " fake version 1.0")
    sys.exit(0)
with open(os.environ["FAKE_TOOL_CALLS"], "a") as trace:
    trace.write(json.dumps({"stage": stage, "args": args, "cwd": str(Path.cwd())}) + "\n")
print(stage + " standard output", flush=True)
print(stage + " standard error", file=sys.stderr, flush=True)
if os.environ.get("FAKE_TIMEOUT_STAGE") == stage:
    time.sleep(5)
if os.environ.get("FAKE_FAIL_STAGE") == stage:
    sys.exit(23)
if stage == "synthesis" and os.environ.get("FAKE_FAIL_RUN") == Path.cwd().parent.name:
    sys.exit(23)
outputs = []
if name == "yosys":
    script = shlex.split(args[args.index("-p") + 1])
    for option in ("-json", "-blif"):
        if option in script:
            outputs.append((Path(script[script.index(option) + 1]), "synthesized " + option))
    print("   Number of cells: 1022")
elif name == "nextpnr-ice40":
    assert "--timing-allow-fail" not in args
    outputs.append((Path(args[args.index("--asc") + 1]), "routed placement"))
    outputs.append((Path(args[args.index("--write") + 1]), json.dumps({"modules": {}})))
    failed = os.environ.get("FAKE_BAD_TIMING") == "1"
    report = {
        "fmax": {
            "clk_125": {"achieved": 73.1, "constraint": 62.5},
            "lvds_rx_inst.clk": {"achieved": 56.24 if failed else 74.96, "constraint": 64.0},
        },
        "utilization": {
            "ICESTORM_LC": {"used": 1022, "available": 1280},
            "ICESTORM_RAM": {"used": 16, "available": 16},
            "SB_GB": {"used": 6, "available": 8},
        },
    }
    if os.environ.get("FAKE_NONFINITE_TIMING"):
        report["fmax"]["lvds_rx_inst.clk"]["achieved"] = float(os.environ["FAKE_NONFINITE_TIMING"])
    outputs.append((Path(args[args.index("--report") + 1]), json.dumps(report)))
else:
    outputs.append((Path(args[-1]), "packed candidate image"))
for output, content in outputs:
    if os.environ.get("FAKE_MISSING_STAGE") != stage:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(content + "\n")
if stage == "packing" and os.environ.get("FAKE_MUTATE_TOOL"):
    with open(os.environ["FAKE_MUTATE_TOOL"], "a") as tool:
        tool.write("\n# tool changed while experiment was running\n")
'''


class ExperimentRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="fpga-runner-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        known_image = "known working image\n"
        known_header = "uint8_t cariboulite_firmware[] = {" + ", ".join(
            f"0x{byte:02x}" for byte in known_image.encode()) + "};\n"
        fixtures = {
            "firmware/top.v": "module top(); endmodule\n",
            "firmware/included.v": "module included(); endmodule\n",
            "firmware/io.pcf": "set_io clk 1\n",
            "firmware/Makefile": "build:\n\tfalse\n",
            "firmware/top.bin": known_image,
            "firmware/h-files/cariboulite_fpga_firmware.h": known_header,
            "software/libcariboulite/src/cariboulite_fpga_firmware.h": known_header,
            "build/cariboulite_test_app": "known working test app\n",
            "build/radio-backend/cariboulite_radio_backend": "known working backend\n",
            ".gitignore": "build/\n",
        }
        for relative, content in fixtures.items():
            path = self.repo / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
            if relative.startswith("build/"):
                path.chmod(0o755)
        for args in (["init", "-q"], ["add", "."], ["-c", "user.name=Runner Test", "-c", "user.email=runner@example.invalid", "commit", "-qm", "reference"]):
            subprocess.run(["git", *args], cwd=self.repo, check=True, capture_output=True)
        self.tool_dir = self.root / "tools"
        self.tool_dir.mkdir()
        for name in ("yosys", "nextpnr-ice40", "icepack"):
            tool = self.tool_dir / name
            tool.write_text(FAKE_TOOL)
            tool.chmod(0o755)
        self.calls_file = self.root / "tool-calls.jsonl"
        self.env = os.environ.copy()
        self.env.update({"PATH": str(self.tool_dir) + os.pathsep + self.env["PATH"],
                         "FAKE_TOOL_CALLS": str(self.calls_file)})
        self.reference = self.root / "reference"
        self.output = self.root / "run"

    def cli(self, *args, success=True, env=None):
        command = [sys.executable, str(RUNNER), *map(str, args)]
        result = subprocess.run(command, cwd=self.repo, env=env or self.env,
                                text=True, capture_output=True, timeout=20)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def freeze(self):
        self.cli("freeze", "--repo", self.repo, "--output", self.reference)
        return json.loads((self.reference / "reference.json").read_text())

    def run_experiment(self, *args, success=True, env=None):
        return self.cli("run", "--reference", self.reference, "--output", self.output,
                        "--synthesis", "default", "--refinement", "parallel",
                        "--seed", "16", "--repeat", "1", "--yosys", self.tool_dir / "yosys",
                        "--nextpnr", self.tool_dir / "nextpnr-ice40",
                        "--icepack", self.tool_dir / "icepack", *args,
                        success=success, env=env)

    def calls(self):
        return [json.loads(line) for line in self.calls_file.read_text().splitlines()] if self.calls_file.exists() else []

    @staticmethod
    def content_hashes(root):
        return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in root.rglob("*") if path.is_file() and ".git" not in path.relative_to(root).parts}

    def test_successful_run_orders_stages_and_preserves_reference_and_checkout(self):
        before_repo = self.content_hashes(self.repo)
        self.freeze()
        before_reference = self.content_hashes(self.reference)
        self.run_experiment()
        self.assertEqual([call["stage"] for call in self.calls()],
                         ["synthesis", "routing", "packing"])
        for call in self.calls():
            self.assertTrue(Path(call["cwd"]).is_relative_to(self.output), call)
        self.assertEqual(self.content_hashes(self.repo), before_repo)
        self.assertEqual(self.content_hashes(self.reference), before_reference)
        self.assertTrue((self.output / "result.json").is_file())
        self.assertTrue(list(self.output.rglob("*.bin")))
        summary = json.loads((self.output / "result.json").read_text())
        self.assertEqual(summary["status"], "passed")
        self.assertEqual([stage["name"] for stage in summary["stages"]],
                         ["synthesis", "routing", "packing"])
        self.assertEqual(summary["metrics"]["utilization"]["ICESTORM_RAM"]["used"], 16)
        self.assertIn("work/top.bin", summary["artifacts"])

    def test_stage_failures_keep_evidence_and_stop_following_stages(self):
        self.freeze()
        for failed, expected in (("synthesis", ["synthesis"]),
                                 ("routing", ["synthesis", "routing"]),
                                 ("packing", ["synthesis", "routing", "packing"])):
            with self.subTest(stage=failed):
                self.output = self.root / ("failed-" + failed)
                self.calls_file.unlink(missing_ok=True)
                env = dict(self.env, FAKE_FAIL_STAGE=failed)
                self.run_experiment(success=False, env=env)
                self.assertEqual([call["stage"] for call in self.calls()], expected)
                result = self.output / "result.json"
                self.assertTrue(result.is_file())
                self.assertIsInstance(json.loads(result.read_text()), dict)
                logs = "\n".join(path.read_text() for path in self.output.rglob("*.log"))
                self.assertIn(failed + " standard output", logs)
                self.assertIn(failed + " standard error", logs)

    def test_missing_success_outputs_are_failures(self):
        self.freeze()
        for missing, expected in (("synthesis", ["synthesis"]),
                                  ("routing", ["synthesis", "routing"]),
                                  ("packing", ["synthesis", "routing", "packing"])):
            with self.subTest(stage=missing):
                self.output = self.root / ("missing-" + missing)
                self.calls_file.unlink(missing_ok=True)
                self.run_experiment(success=False, env=dict(self.env, FAKE_MISSING_STAGE=missing))
                self.assertEqual([call["stage"] for call in self.calls()], expected)
                self.assertTrue((self.output / "result.json").is_file())

    def test_failing_reported_clock_prevents_packing_even_after_zero_exit(self):
        self.freeze()
        self.run_experiment(success=False, env=dict(self.env, FAKE_BAD_TIMING="1"))
        self.assertEqual([call["stage"] for call in self.calls()], ["synthesis", "routing"])
        self.assertTrue((self.output / "result.json").is_file())
        self.assertFalse(list(self.output.rglob("*.bin")))

    def test_nonfinite_timing_preserves_a_valid_json_failure_summary(self):
        self.freeze()
        for invalid in ("nan", "inf"):
            with self.subTest(value=invalid):
                self.output = self.root / ("nonfinite-" + invalid)
                self.calls_file.unlink(missing_ok=True)
                self.run_experiment(success=False, env=dict(self.env, FAKE_NONFINITE_TIMING=invalid))
                summary = json.loads((self.output / "result.json").read_text(),
                                     parse_constant=lambda value: self.fail("invalid JSON number: " + value))
                self.assertEqual(summary["status"], "failed")
                self.assertIsNone(summary["metrics"]["fmax"]["lvds_rx_inst.clk"]["achieved"])
                self.assertEqual([call["stage"] for call in self.calls()], ["synthesis", "routing"])

    def test_ce2_serial_variant_keeps_other_routing_options_fixed(self):
        self.freeze()
        self.run_experiment("--synthesis", "ce2", "--refinement", "serial")
        calls = self.calls()
        synth_args = calls[0]["args"]
        script = synth_args[synth_args.index("-p") + 1]
        self.assertIn("-dffe_min_ce_use 2", script)
        routing = calls[1]["args"]
        self.assertNotIn("--parallel-refine", routing)
        self.assertIn("--opt-timing", routing)
        self.assertIn("--no-promote-globals", routing)
        self.assertNotIn("--timing-allow-fail", routing)
        self.assertEqual(routing[routing.index("--seed") + 1], "16")

    def test_existing_run_directory_is_never_overwritten(self):
        self.freeze()
        self.output.mkdir()
        sentinel = self.output / "sentinel"
        sentinel.write_text("keep existing work\n")
        before = self.content_hashes(self.output)
        self.run_experiment(success=False)
        self.assertEqual(self.content_hashes(self.output), before)
        self.assertEqual(self.calls(), [])

    def test_existing_reference_directory_is_never_overwritten(self):
        self.reference.mkdir()
        (self.reference / "sentinel").write_text("keep existing reference\n")
        before = self.content_hashes(self.reference)
        self.cli("freeze", "--repo", self.repo, "--output", self.reference, success=False)
        self.assertEqual(self.content_hashes(self.reference), before)

    def test_timeout_keeps_stage_log_and_failure_summary(self):
        self.freeze()
        # Allow interpreter startup on a busy Pi; the fake stage then sleeps 5 s.
        self.run_experiment("--timeout", "1", success=False,
                            env=dict(self.env, FAKE_TIMEOUT_STAGE="synthesis"))
        self.assertEqual([call["stage"] for call in self.calls()], ["synthesis"])
        self.assertTrue((self.output / "result.json").is_file())
        logs = "\n".join(path.read_text() for path in self.output.rglob("*.log"))
        self.assertIn("synthesis standard output", logs)

    def test_run_output_cannot_be_created_inside_reference(self):
        self.freeze()
        before = self.content_hashes(self.reference)
        self.output = self.reference / "candidate"
        self.run_experiment(success=False)
        self.assertEqual(self.content_hashes(self.reference), before)
        self.assertEqual(self.calls(), [])

    def test_frozen_reference_verifies_and_contains_recoverable_image_and_hosts(self):
        manifest = self.freeze()
        self.assertEqual(manifest["schema_version"], 1)
        self.assertEqual(manifest["build"]["device"], "lp1k")
        self.assertEqual(manifest["build"]["package"], "qn84")
        self.assertTrue(manifest["git"]["revision"])
        for relative in ("source/firmware/top.v", "source/firmware/io.pcf",
                         "source/firmware/Makefile", "rollback/firmware/top.bin",
                         "rollback/firmware/h-files/cariboulite_fpga_firmware.h",
                         "rollback/software/libcariboulite/src/cariboulite_fpga_firmware.h",
                         "archives/source.tar.gz", "toolchain.json"):
            with self.subTest(artifact=relative):
                self.assertIn(relative, manifest["files"])
                path = self.reference / relative
                self.assertTrue(path.is_file())
                self.assertEqual(manifest["files"][relative]["sha256"],
                                 hashlib.sha256(path.read_bytes()).hexdigest())
        for relative in ("build/cariboulite_test_app", "build/radio-backend/cariboulite_radio_backend"):
            self.assertEqual((self.repo / relative).read_bytes(),
                             (self.reference / "rollback" / relative).read_bytes())
        self.cli("verify", "--reference", self.reference)
        self.assertEqual(self.calls(), [])

    def test_plan_contains_twelve_unique_seed16_configs_without_running_tools(self):
        manifest = self.freeze()
        plan_output = self.root / "plan"
        self.cli("plan", "--reference", self.reference, "--output", plan_output)
        plan = json.loads((plan_output / "plan.json").read_text())
        self.assertEqual(plan["schema_version"], 1)
        runs = plan["runs"]
        self.assertEqual(len(runs), 12)
        self.assertEqual(len({run["name"] for run in runs}), 12)
        self.assertEqual({(run["synthesis"], run["refinement"], run["seed"], run["repeat"])
                          for run in runs},
                         {(synthesis, refinement, 16, repeat)
                          for synthesis in ("default", "ce2")
                          for refinement in ("parallel", "serial")
                          for repeat in (1, 2, 3)})
        self.assertTrue(all(isinstance(run["command"], list) for run in runs))
        for run in runs:
            frozen_runner = Path(run["command"][1])
            self.assertTrue(frozen_runner.is_relative_to(self.reference))
            relative = str(frozen_runner.relative_to(self.reference))
            self.assertIn(relative, manifest["files"])
            self.assertEqual(frozen_runner.read_bytes(), RUNNER.read_bytes())
        self.assertEqual(self.calls(), [])

    def test_generated_matrix_script_continues_after_a_failed_run(self):
        self.freeze()
        plan_output = self.root / "plan"
        self.cli("plan", "--reference", self.reference, "--output", plan_output)
        runs = json.loads((plan_output / "plan.json").read_text())["runs"]
        env = dict(self.env, FAKE_FAIL_RUN=runs[0]["name"])
        script = subprocess.run(["sh", str(plan_output / "commands.sh")], cwd=self.repo,
                                env=env, text=True, capture_output=True, timeout=30)
        self.assertNotEqual(script.returncode, 0, script.stdout + script.stderr)
        for index, run in enumerate(runs):
            command = run["command"]
            result_path = Path(command[command.index("--output") + 1]) / "result.json"
            with self.subTest(run=run["name"]):
                self.assertTrue(result_path.is_file(), script.stdout + script.stderr)
                result = json.loads(result_path.read_text())
                self.assertEqual(result["status"], "failed" if index == 0 else "passed")
        self.assertEqual(len(self.calls()), 1 + 11 * 3)

    def test_tampered_reference_is_rejected_before_tools_run(self):
        self.freeze()
        for relative in ("source/firmware/top.v", "rollback/firmware/top.bin", "archives/source.tar.gz"):
            with self.subTest(artifact=relative):
                artifact = self.reference / relative
                original = artifact.read_bytes()
                artifact.write_bytes(original + b"tampered")
                self.cli("verify", "--reference", self.reference, success=False)
                self.run_experiment(success=False)
                self.assertFalse(self.output.exists())
                self.assertEqual(self.calls(), [])
                artifact.write_bytes(original)

    def test_unlisted_source_include_is_rejected_before_tools_run(self):
        self.freeze()
        (self.reference / "source/firmware/extra.vh").write_text("`define EXTRA_LOGIC 1\n")
        self.cli("verify", "--reference", self.reference, success=False)
        self.run_experiment(success=False)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.calls(), [])

    def test_tool_changed_during_run_cannot_publish_a_passing_result(self):
        self.freeze()
        self.run_experiment(success=False,
                            env=dict(self.env, FAKE_MUTATE_TOOL=str(self.tool_dir / "yosys")))
        self.assertEqual([call["stage"] for call in self.calls()],
                         ["synthesis", "routing", "packing"])
        result = json.loads((self.output / "result.json").read_text())
        self.assertEqual(result["status"], "failed")
        self.assertIn("tool", result["error"].lower())

    def test_operating_system_spawn_failure_keeps_a_failure_result(self):
        interpreter = self.root / "temporary-python"
        interpreter.symlink_to(sys.executable)
        yosys = self.tool_dir / "yosys"
        yosys.write_text(FAKE_TOOL.replace("#!/usr/bin/env python3", "#!" + str(interpreter), 1))
        self.freeze()
        # The FPGA tool still exists, is executable, and matches its frozen hash;
        # only execve's interpreter lookup fails when the experiment starts.
        interpreter.unlink()
        self.run_experiment(success=False)
        result = json.loads((self.output / "result.json").read_text())
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["error"])
        self.assertEqual(self.calls(), [])

    def test_production_source_directories_are_protected_from_output_writes(self):
        self.freeze()
        before = self.content_hashes(self.repo)
        for relative in ("firmware/experiment", "software/experiment", ".git/experiment"):
            with self.subTest(output=relative):
                self.output = self.repo / relative
                self.run_experiment(success=False)
                self.assertFalse(self.output.exists())
        self.assertEqual(self.content_hashes(self.repo), before)
        self.assertEqual(self.calls(), [])

    def test_modified_reference_input_cannot_be_frozen_as_head(self):
        (self.repo / "firmware/top.v").write_text("module changed(); endmodule\n")
        self.cli("freeze", "--repo", self.repo, "--output", self.reference, success=False)
        self.assertFalse((self.reference / "reference.json").exists())

    def test_header_bytes_must_match_the_preserved_rollback_image(self):
        header = self.repo / "firmware/h-files/cariboulite_fpga_firmware.h"
        header.write_text("uint8_t cariboulite_firmware[] = {0x00};\n")
        subprocess.run(["git", "add", str(header)], cwd=self.repo, check=True, capture_output=True)
        subprocess.run(["git", "-c", "user.name=Runner Test", "-c", "user.email=runner@example.invalid",
                        "commit", "-qm", "broken header"], cwd=self.repo, check=True, capture_output=True)
        self.cli("freeze", "--repo", self.repo, "--output", self.reference, success=False)
        self.assertFalse((self.reference / "reference.json").exists())


if __name__ == "__main__":
    unittest.main()
