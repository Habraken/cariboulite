#!/usr/bin/env python3
"""Freeze and build isolated CaribouLite FPGA experiments; never programs hardware."""

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time


REPO = Path(__file__).resolve().parents[2]
IMAGE = "firmware/top.bin"
HEADERS = ("firmware/h-files/cariboulite_fpga_firmware.h",
           "software/libcariboulite/src/cariboulite_fpga_firmware.h")
TOOLS = {"yosys": ["-V"], "nextpnr-ice40": ["--version"], "icepack": ["-h"],
         "iverilog": ["-V"], "vvp": ["-V"], "make": ["--version"],
         "cc": ["--version"], "cmake": ["--version"]}


def timestamp():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def capture(command, cwd=None):
    p = subprocess.run(command, cwd=cwd, text=True, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=30)
    return {"command": [str(x) for x in command], "exit_code": p.returncode,
            "output": p.stdout}


def git(repo, *args):
    p = subprocess.run(["git", "-C", str(repo), *args], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, check=True)
    return p.stdout


def nested(path, parent):
    return path == parent or parent in path.parents


def reserve_output(path, repo, reference=None):
    # Resolve parent symlinks before comparing protected paths.
    path = Path(path).absolute()
    resolved = path.resolve()
    for protected in (repo / "firmware", repo / "software", repo / "docs", repo / ".git"):
        if nested(resolved, protected.resolve()):
            raise ValueError(f"output is inside protected source directory: {path}")
    if reference and (nested(resolved, reference) or nested(reference, resolved)):
        raise ValueError("output and immutable reference directories must be separate")
    if path.exists() or path.is_symlink():
        raise ValueError(f"output already exists; use a fresh directory: {path}")
    path.mkdir(parents=True, exist_ok=False)
    return path.resolve()


def file_inventory(root, excluded=()):
    return {str(p.relative_to(root)): {"sha256": digest(p), "size_bytes": p.stat().st_size,
                                     "mode": p.stat().st_mode & 0o777}
            for p in sorted(root.rglob("*")) if p.is_file()
            and str(p.relative_to(root)) not in excluded}


def toolchain():
    result = {"created_utc": timestamp(), "platform": platform.platform(),
              "machine": platform.machine(), "kernel": platform.release(), "tools": {},
              "python": {"version": sys.version, "executable": sys.executable},
              "environment": {"YOSYS_DATDIR": os.environ.get("YOSYS_DATDIR")}}
    for name, flags in TOOLS.items():
        location = shutil.which(name)
        if not location:
            result["tools"][name] = {"available": False}
            continue
        path = Path(location).resolve()
        result["tools"][name] = {"available": True, "path": str(Path(location).absolute()),
                                "resolved_path": str(path), "sha256": digest(path),
                                **capture([location, *flags])}
    for name in ("yosys", "nextpnr-ice40", "icepack"):
        if not result["tools"][name]["available"]:
            raise ValueError(f"required FPGA tool not found: {name}")
    result["tools"]["yosys"]["synthesis_help"] = capture(
        [result["tools"]["yosys"]["path"], "-Q", "-T", "-p", "help synth_ice40"])
    result["tools"]["nextpnr-ice40"]["help"] = capture(
        [result["tools"]["nextpnr-ice40"]["path"], "--help"])
    # Use the selected Yosys installation, including in fake-tool tests. Never
    # silently pair a custom Yosys with an unrelated global yosys-config.
    directory = Path(result["tools"]["yosys"]["resolved_path"]).parent
    abc = directory / "yosys-abc"
    if abc.is_file():
        result["tools"]["yosys-abc"] = {"available": True, "path": str(abc),
                                           "sha256": digest(abc)}
    config = directory / "yosys-config"
    if config.is_file():
        probe = capture([str(config), "--datdir"])
        if probe["exit_code"] != 0:
            raise ValueError("yosys-config could not report the synthesis data directory")
        datdir = Path(os.environ.get("YOSYS_DATDIR") or probe["output"].strip()).resolve()
        if not datdir.is_dir():
            raise ValueError(f"Yosys synthesis data directory unavailable: {datdir}")
        result["yosys_data"] = {"path": str(datdir), "files": file_inventory(datdir),
                                "config_probe": probe}
    return result


def verify_tools(record, commands):
    for name, executable in commands.items():
        path = Path(executable)
        if not path.is_file() or digest(path) != record["tools"][name]["sha256"]:
            raise ValueError(f"tool differs from frozen reference or is unavailable: {name}")
    abc = record["tools"].get("yosys-abc")
    if abc and (not Path(abc["path"]).is_file() or digest(abc["path"]) != abc["sha256"]):
        raise ValueError("tool differs from frozen reference: yosys-abc")
    if record.get("environment", {}).get("YOSYS_DATDIR") != os.environ.get("YOSYS_DATDIR"):
        raise ValueError("tool environment differs from frozen reference: YOSYS_DATDIR")
    data = record.get("yosys_data")
    if data and file_inventory(Path(data["path"])) != data["files"]:
        raise ValueError("tool synthesis data differs from frozen reference")


def header_image(path):
    text = path.read_text()
    match = re.search(r"\bcariboulite_firmware\s*\[[^\]]*\]\s*=\s*\{(.*?)\}", text, re.S)
    if not match:
        raise ValueError(f"firmware byte array not found in {path}")
    return bytes(int(value, 16) for value in re.findall(r"0[xX]([0-9a-fA-F]{1,2})\b", match[1]))


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination, follow_symlinks=True)


def freeze(args):
    repo = args.repo.resolve()
    revision = git(repo, "rev-parse", "HEAD").decode().strip()
    branch = git(repo, "branch", "--show-current").decode().strip()
    status = git(repo, "status", "--porcelain=v1", "--untracked-files=all").decode()
    inputs = sorted(p for p in (repo / "firmware").rglob("*")
                    if p.suffix in (".v", ".vh", ".sv", ".svh") and p.is_file())
    inputs += [repo / "firmware/io.pcf", repo / "firmware/Makefile"]
    if not (repo / "firmware/top.v").is_file():
        raise ValueError("reference repository has no firmware/top.v")
    # A reference must be reproducible from the stated commit. Preparation scripts
    # may be uncommitted, but FPGA inputs and active firmware must match HEAD.
    committed = {}
    for path in [*inputs, repo / IMAGE, *(repo / name for name in HEADERS)]:
        relative = str(path.relative_to(repo))
        committed[relative] = git(repo, "show", f"{revision}:{relative}")
        if not path.is_file() or path.read_bytes() != committed[relative]:
            raise ValueError(f"reference input differs from committed source: {relative}")
    image = committed[IMAGE]
    for name in HEADERS:
        if header_image(repo / name) != image:
            raise ValueError(f"active firmware header does not match top.bin: {name}")

    out = reserve_output(args.output, repo)
    try:
        for path in inputs:
            relative = path.relative_to(repo)
            target = out / "source" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(committed[str(relative)])
        for name in (IMAGE, *HEADERS):
            target = out / "rollback" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(committed[name])
        copy_file(Path(__file__).resolve(), out / "tools/fpga_experiment.py")
        host = ["build/cariboulite_test_app", "build/radio-backend/cariboulite_radio_backend",
                "build/radio-backend/cariboulite_radio_controller", "build/test/caribou_programmer",
                "build/CMakeCache.txt", "build/compile_commands.json",
                "build/CMakeFiles/cariboulite_test_app.dir/flags.make",
                "build/CMakeFiles/cariboulite_test_app.dir/link.txt",
                "software/libcariboulite/tools/cariboulite_radio_monitor.py",
                "software/libcariboulite/tools/at86rf215_monitor.py"]
        host += [str(p.relative_to(repo)) for p in sorted((repo / "build").glob("*.so*")) if p.is_file()]
        preserved_host = []
        for name in host:
            if (repo / name).is_file():
                copy_file(repo / name, out / "rollback" / name)
                preserved_host.append(name)
        dependencies = {}
        reader = shutil.which("readelf")
        if reader:
            for name in preserved_host:
                if (repo / name).read_bytes()[:4] == b"\x7fELF":
                    dependencies[name] = capture([reader, "-d", str(repo / name)])
        write_json(out / "host-dependencies.json", dependencies)
        # Preserve installed module bytes for same-kernel rollback, never load them.
        modules = {}
        modinfo = shutil.which("modinfo")
        if modinfo:
            for name in ("smi_stream_dev", "bcm2835_smi"):
                info = capture([modinfo, name])
                filename = capture([modinfo, "-n", name])
                entry = {"modinfo": info, "filename": filename}
                module = Path(filename["output"].strip())
                if filename["exit_code"] == 0 and module.is_file():
                    relative = Path("rollback/modules") / module.name
                    copy_file(module, out / relative)
                    entry["artifact"] = str(relative)
                    entry["original_path"] = str(module)
                modules[name] = entry
        write_json(out / "modules.json", modules)
        (out / "archives").mkdir()
        git(repo, "archive", "--format=tar.gz", "--output=" + str(out / "archives/source.tar.gz"), revision)
        submodules = []
        for row in git(repo, "ls-tree", "-r", revision).decode().splitlines():
            metadata, name = row.split("\t", 1)
            mode, _, commit = metadata.split()
            if mode != "160000":
                continue
            item = {"path": name, "revision": commit, "archived": False}
            if (repo / name).is_dir():
                try:
                    git(repo / name, "cat-file", "-e", commit + "^{commit}")
                    archive = "archives/submodule-" + name.replace("/", "_") + ".tar.gz"
                    git(repo / name, "archive", "--format=tar.gz", "--output=" + str(out / archive), commit)
                    item.update(archived=True, archive=archive,
                                working_revision=git(repo / name, "rev-parse", "HEAD").decode().strip(),
                                status=git(repo / name, "status", "--porcelain=v1").decode())
                except subprocess.CalledProcessError:
                    pass
            submodules.append(item)
        tools = toolchain()
        write_json(out / "toolchain.json", tools)
        if args.baseline_json:
            baseline = json.loads(args.baseline_json.read_text())
            if baseline.get("revision") != revision:
                raise ValueError("baseline receipt revision does not match the reference")
            copy_file(args.baseline_json, out / "baseline-start.json")
        if args.checks_json:
            checks = args.checks_json.resolve()
            report = json.loads(checks.read_text())
            for entry in report.get("checks", []) + report.get("tool_versions", []):
                log = Path(entry.get("log", ""))
                if log.is_file():
                    copy_file(log, out / "checks/logs" / log.name)
                    entry["archived_log"] = "checks/logs/" + log.name
            (out / "checks").mkdir(exist_ok=True)
            write_json(out / "checks/results.json", report)
        (out / "RESTORE.md").write_text(
            "# Reference and same-host rollback\n\n"
            "The packaged image and host binaries are preserved; the currently programmed FPGA "
            "was not read or changed. Verify this bundle before using it.\n\n"
            "Restore rollback/firmware/top.bin and both active headers to their original "
            "repository paths together with the compatible host files under rollback/build/. "
            "The executables use the original build directory in RUNPATH. If running from a "
            "different directory, point LD_LIBRARY_PATH at the bundled rollback/build library "
            "directory. System runtime dependencies and the recorded OS/kernel remain required.\n\n"
            "archives/source.tar.gz contains the recorded superproject commit. Extract each "
            "archived submodule into its recorded relative path for an offline source rebuild. "
            "Read reference.json for any unavailable submodules or host files.\n\n"
            "modules.json records any available installed module artifacts and original paths; "
            "these are specific to the recorded kernel. No automatic installation, module "
            "reload, FPGA programming or application launch is performed by this runner.\n")
        reference = {"schema_version": 1, "kind": "fpga-reference", "created_utc": timestamp(),
                     "repository": str(repo), "git": {"revision": revision, "branch": branch, "status": status},
                     "build": {"device": "lp1k", "package": "qn84", "seed": 16,
                               "system_mhz": 62.5, "lvds_mhz": 64.0},
                     "firmware_source_matches_revision": True,
                     "packaged_image_sha256": hashlib.sha256(image).hexdigest(),
                     "active_headers_match_image": True, "programmed_image_verified": False,
                     "host_files": preserved_host, "missing_host_files": [n for n in host if n not in preserved_host],
                     "host_build_provenance": "Copied local artifacts; compiler settings/dependencies recorded where available, not a reproducible-build attestation.",
                     "submodules": submodules, "files": file_inventory(out)}
        write_json(out / "reference.json", reference)
        (out / "reference.sha256").write_text(digest(out / "reference.json") + "\n")
        all_files = file_inventory(out)
        (out / "SHA256SUMS").write_text("".join(f"{v['sha256']}  {k}\n" for k, v in all_files.items()))
        print(out / "reference.json")
    except BaseException as error:
        write_json(out / "failure.json", {"error": str(error), "created_utc": timestamp()})
        raise


def verify_reference(path):
    path = path.resolve()
    manifest = path / "reference.json"
    if digest(manifest) != (path / "reference.sha256").read_text().strip():
        raise ValueError("reference manifest checksum mismatch")
    reference = json.loads(manifest.read_text())
    if reference.get("schema_version") != 1 or reference.get("kind") not in ("fpga-reference", "fpga-candidate"):
        raise ValueError("unsupported reference manifest")
    actual = set()
    for artifact in path.rglob("*"):
        if artifact.is_symlink():
            raise ValueError(f"unsafe reference artifact: {artifact.relative_to(path)}")
        if artifact.is_file():
            actual.add(str(artifact.relative_to(path)))
    expected_files = set(reference["files"]) | {"reference.json", "reference.sha256", "SHA256SUMS"}
    if actual != expected_files:
        raise ValueError(f"reference file inventory mismatch: {sorted(actual ^ expected_files)}")
    for relative, expected in reference["files"].items():
        target = path / relative
        if target.is_symlink() or not nested(target.resolve(), path) or not target.is_file():
            raise ValueError(f"missing or unsafe reference artifact: {relative}")
        if target.stat().st_size != expected["size_bytes"] or digest(target) != expected["sha256"]:
            raise ValueError(f"reference artifact checksum mismatch: {relative}")
        if target.stat().st_mode & 0o777 != expected["mode"]:
            raise ValueError(f"reference artifact mode mismatch: {relative}")
    checksums = "".join(f"{digest(path / name)}  {name}\n" for name in sorted(actual - {"SHA256SUMS"}))
    if (path / "SHA256SUMS").read_text() != checksums:
        raise ValueError("reference checksum index mismatch")
    if reference["kind"] == "fpga-candidate":
        verify_candidate(path, reference)
    return reference


def verify_candidate(path, reference):
    details = reference.get("candidate", {})
    base_path = path / "candidate/base-reference.json"
    if (not isinstance(details, dict) or not base_path.is_file()
            or reference.get("firmware_source_matches_revision") is not False
            or details.get("source_origin") != "working_tree_existing_hdl"
            or details.get("base_reference_sha256") != digest(base_path)):
        raise ValueError("invalid candidate provenance")
    base = json.loads(base_path.read_text())
    if base.get("kind") != "fpga-reference" or details.get("base_revision") != base.get("git", {}).get("revision"):
        raise ValueError("invalid candidate base reference")
    hdl = sorted(name.removeprefix("source/") for name in base["files"]
                 if name.startswith("source/firmware/") and Path(name).suffix in (".v", ".vh", ".sv", ".svh"))
    if details.get("hdl_files") != hdl:
        raise ValueError("candidate HDL inventory differs from its base")
    mutable = {"source/" + name for name in hdl} | {"tools/fpga_experiment.py"}
    for name, expected in base["files"].items():
        if name not in mutable and reference["files"].get(name) != expected:
            raise ValueError(f"candidate changed an inherited artifact: {name}")
    changed = [name for name in hdl if reference["files"].get("source/" + name) != base["files"]["source/" + name]]
    if details.get("changed_files") != changed or reference.get("build") != base.get("build"):
        raise ValueError("candidate changes differ from recorded scope")


def candidate(args):
    """Snapshot edits to existing HDL without relabelling them committed source."""
    parent = args.reference.resolve()
    base = verify_reference(parent)
    if base["kind"] != "fpga-reference":
        raise ValueError("candidate must derive directly from a committed reference")
    repo = args.repo.resolve()
    hdl = sorted(name.removeprefix("source/") for name in base["files"]
                 if name.startswith("source/firmware/") and Path(name).suffix in (".v", ".vh", ".sv", ".svh"))
    current = sorted(str(path.relative_to(repo)) for path in (repo / "firmware").rglob("*")
                     if path.suffix in (".v", ".vh", ".sv", ".svh") and path.is_file())
    if current != hdl:
        raise ValueError("candidate supports only existing HDL filenames; additions/deletions require a new reviewed scope")
    for name in ("firmware/io.pcf", "firmware/Makefile"):
        source = repo / name
        if source.is_symlink() or not source.is_file() or source.read_bytes() != (parent / "source" / name).read_bytes():
            raise ValueError(f"candidate must retain frozen build constraints: {name}")
    cached = {}
    for name in hdl:
        source = repo / name
        if source.is_symlink():
            raise ValueError(f"unsafe candidate HDL input: {name}")
        cached[name] = (source.read_bytes(), source.stat().st_mode & 0o777)
    revision = git(repo, "rev-parse", "HEAD").decode().strip()
    branch = git(repo, "branch", "--show-current").decode().strip()
    status = git(repo, "status", "--porcelain=v1", "--untracked-files=all").decode()
    patch = git(repo, "diff", "--no-ext-diff", "--binary", base["git"]["revision"], "--", *hdl)
    if any((repo / name).read_bytes() != content for name, (content, _) in cached.items()):
        raise ValueError("candidate HDL changed while its snapshot was being prepared")
    out = reserve_output(args.output, repo, parent)
    try:
        for name in base["files"]:
            copy_file(parent / name, out / name)
        copy_file(parent / "reference.json", out / "candidate/base-reference.json")
        (out / "candidate/hdl.patch").write_bytes(patch)
        copy_file(Path(__file__).resolve(), out / "tools/fpga_experiment.py")
        for name, (content, mode) in cached.items():
            destination = out / "source" / name
            destination.write_bytes(content)
            destination.chmod(mode)
        (out / "CANDIDATE.md").write_text(
            "# Uncommitted HDL candidate\n\n"
            "Only the existing frozen HDL input filenames may change. PCF, Makefile, toolchain, "
            "rollback image, headers and host files remain inherited from the committed reference. "
            "The packaged image is for rollback; it was not built from these candidate sources.\n\n"
            "source/firmware is the exact candidate input snapshot. candidate/hdl.patch is the "
            "working-tree HDL diff against the base revision; archives/source.tar.gz still "
            "contains that base revision. candidate/base-reference.json records the original "
            "artifact hashes. New HDL filenames require a separately reviewed scope.\n")
        reference = {**base, "kind": "fpga-candidate", "created_utc": timestamp(), "repository": str(repo),
                     "git": {"revision": revision, "branch": branch, "status": status},
                     "firmware_source_matches_revision": False, "packaged_image_role": "rollback_only"}
        reference["candidate"] = {"source_origin": "working_tree_existing_hdl", "base_reference": str(parent),
                                  "base_reference_sha256": digest(parent / "reference.json"),
                                  "base_revision": base["git"]["revision"], "hdl_files": hdl,
                                  "changed_files": [name for name in hdl if file_inventory(out / "source")[name] != base["files"]["source/" + name]]}
        verify_reference(parent)
        reference["files"] = file_inventory(out)
        write_json(out / "reference.json", reference)
        (out / "reference.sha256").write_text(digest(out / "reference.json") + "\n")
        (out / "SHA256SUMS").write_text("".join(f"{value['sha256']}  {name}\n" for name, value in file_inventory(out).items()))
        verify_reference(out)
        print(out / "reference.json")
    except BaseException as error:
        write_json(out / "failure.json", {"error": str(error), "created_utc": timestamp()})
        raise


def plan(args):
    reference = args.reference.resolve()
    metadata = verify_reference(reference)
    out = reserve_output(args.output, Path(metadata["repository"]), reference)
    runs = []
    for synthesis in ("default", "ce2"):
        for refinement in ("parallel", "serial"):
            for repeat in range(1, 4):
                name = f"{synthesis}-{refinement}-s16-r{repeat}"
                command = [sys.executable, str(reference / "tools/fpga_experiment.py"), "run", "--reference", str(reference),
                           "--output", str(out / "runs" / name), "--synthesis", synthesis,
                           "--refinement", refinement, "--seed", "16", "--repeat", str(repeat)]
                runs.append({"name": name, "synthesis": synthesis, "refinement": refinement,
                             "seed": 16, "repeat": repeat, "command": command})
    write_json(out / "plan.json", {"schema_version": 1, "reference": str(reference),
                                  "reference_sha256": digest(reference / "reference.json"), "runs": runs})
    (out / "commands.sh").write_text(
        "#!/bin/sh\n# A2 execution is explicit; do not source this during A1.\nset -u\nfailures=0\n" +
        "\n".join(shlex.join(r["command"]) + " || failures=$((failures + 1))" for r in runs) +
        '\nprintf "Failed runs: %s\\n" "$failures"\ntest "$failures" -eq 0\n')
    print(out / "plan.json")


def stop_group(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
    except ProcessLookupError:
        process.wait()


def stage(name, command, work, log, timeout):
    started = time.monotonic()
    result = {"name": name, "command": command, "cwd": str(work), "log": str(log),
              "started_utc": timestamp(), "status": "failed", "exit_code": None}
    with log.open("w") as output:
        process = None
        try:
            process = subprocess.Popen(command, cwd=work, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True,
                                       env={**os.environ, "TMPDIR": str(work.parent / "tmp")})
            result["exit_code"] = process.wait(timeout=timeout)
            result["status"] = "passed" if result["exit_code"] == 0 else "failed"
        except subprocess.TimeoutExpired:
            stop_group(process)
            result.update(status="timeout", exit_code=process.returncode)
        except OSError as error:
            result["error"] = str(error)
            output.write(str(error) + "\n")
        except BaseException:
            if process is not None:
                stop_group(process)
            raise
    result["elapsed_s"] = time.monotonic() - started
    result["warnings"] = [line for line in log.read_text(errors="replace").splitlines()
                          if re.search(r"\bwarning\b", line, re.I)]
    return result


def json_safe(value):
    # A malformed timing report remains on disk, but must not prevent a valid
    # failure summary from being saved (JSON NaN/Infinity are not valid numbers).
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def timing_metrics(report):
    clocks = report.get("fmax", {})
    if not clocks:
        raise ValueError("routing report contains no clock timing")
    requirements = []
    margins = {}
    for name, values in clocks.items():
        achieved, constraint = values["achieved"], values["constraint"]
        if (not all(isinstance(v, (float, int)) and math.isfinite(v) and v > 0
                    for v in (achieved, constraint)) or achieved < constraint):
            raise ValueError(f"clock timing failed: {name}: {values}")
        requirements.append(constraint)
        margins[name] = 1000 / constraint - 1000 / achieved
    if not all(any(math.isclose(actual, required, abs_tol=1e-6) for actual in requirements)
               for required in (62.5, 64.0)):
        raise ValueError("routing report lacks required 62.5 MHz system / 64 MHz LVDS constraints")
    resources = report.get("utilization", {})
    if not resources:
        raise ValueError("routing report contains no resource utilization")
    for name, values in resources.items():
        used, available = values["used"], values["available"]
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0
                    for v in (used, available)) or used > available):
            raise ValueError(f"resource capacity exceeded: {name}")
    return {"fmax": clocks, "utilization": resources,
            "clock_period_margin_ns": margins, "worst_slack_ns": None,
            "critical_paths": report.get("critical_paths", [])}


def run(args):
    reference = args.reference.resolve()
    metadata = verify_reference(reference)
    tool_record = json.loads((reference / "toolchain.json").read_text())
    tools = tool_record["tools"]
    commands = {}
    for name, supplied in (("yosys", args.yosys), ("nextpnr-ice40", args.nextpnr), ("icepack", args.icepack)):
        executable = supplied or tools[name]["path"]
        location = shutil.which(str(executable))
        if not location or digest(Path(location).resolve()) != tools[name]["sha256"]:
            raise ValueError(f"tool differs from frozen reference or is unavailable: {name}")
        commands[name] = str(Path(location).absolute())
    verify_tools(tool_record, commands)
    out = reserve_output(args.output, Path(metadata["repository"]), reference)
    work = out / "work"
    shutil.copytree(reference / "source/firmware", work)
    (out / "logs").mkdir()
    (out / "tmp").mkdir()
    started = time.monotonic()
    result = {"schema_version": 1, "created_utc": timestamp(), "status": "failed",
              "reference": str(reference), "reference_sha256": digest(reference / "reference.json"),
              "synthesis": args.synthesis, "refinement": args.refinement,
              "tmg_ripup": args.tmg_ripup, "promote_globals": args.promote_globals,
              "synthesis_device": args.synthesis_device,
              "seed": args.seed, "repeat": args.repeat, "stages": [], "metrics": {}, "artifacts": {}}
    synth = "synth_ice40 -top top" + (" -dffe_min_ce_use 2" if args.synthesis == "ce2" else "")
    if args.synthesis_device == "lp":
        synth += " -device lp"
    pipeline = [
        ("synthesis", [commands["yosys"], "-Q", "-T", "-p", synth + " -json top.json -blif top.blif", "top.v"],
         ["top.json", "top.blif"]),
        ("routing", [commands["nextpnr-ice40"], "--lp1k", "--package", "qn84", "--json", "top.json",
         "--pcf", "io.pcf", "--asc", "top.asc", "--write", "routed.json", "--report", "timing.json",
         "--opt-timing", "--seed", str(args.seed)] +
         ([] if args.promote_globals else ["--no-promote-globals"]) +
         (["--tmg-ripup"] if args.tmg_ripup else []) +
         (["--parallel-refine"] if args.refinement == "parallel" else []),
         ["top.asc", "routed.json", "timing.json"]),
        ("packing", [commands["icepack"], "top.asc", "top.bin"], ["top.bin"]),
    ]
    try:
        for name, command, outputs in pipeline:
            record = stage(name, command, work, out / "logs" / (name + ".log"), args.timeout)
            result["stages"].append(record)
            if record["status"] != "passed":
                raise ValueError(f"{name} {record['status']} (exit {record['exit_code']})")
            if any(not (work / p).is_file() or (work / p).stat().st_size == 0 for p in outputs):
                record["status"] = "failed"
                raise ValueError(f"{name} did not produce all required nonempty outputs")
            if name == "routing":
                report = json.loads((work / "timing.json").read_text())
                result["metrics"] = json_safe({"fmax": report.get("fmax", {}),
                                     "utilization": report.get("utilization", {}), "worst_slack_ns": None})
                try:
                    result["metrics"] = timing_metrics(report)
                except (ValueError, KeyError, TypeError):
                    record["status"] = "failed"
                    raise
            write_json(out / "result.json", result)
        result["status"] = "passed"
    except (ValueError, OSError, KeyError, TypeError) as error:
        result["error"] = str(error)
    except KeyboardInterrupt:
        result["error"] = "interrupted"
    finally:
        result["elapsed_s"] = time.monotonic() - started
        result["artifacts"] = file_inventory(out, excluded=("result.json",))
        try:
            verify_reference(reference)
            verify_tools(tool_record, commands)
        except (ValueError, OSError) as error:
            result.update(status="failed", error=str(error))
        write_json(out / "result.json", result)
    print(out / "result.json")
    return 0 if result["status"] == "passed" else 1


def positive(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive value")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    p = sub.add_parser("freeze", help="preserve committed FPGA inputs, image, host files and tool identity")
    p.add_argument("--repo", type=Path, default=REPO)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--checks-json", type=Path)
    p.add_argument("--baseline-json", type=Path, help="optional pre-preparation revision/status receipt")
    p = sub.add_parser("candidate", help="snapshot changes to existing HDL while retaining reference constraints and rollback")
    p.add_argument("--reference", type=Path, required=True)
    p.add_argument("--repo", type=Path, default=REPO)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("verify", help="verify the reference manifest and artifact hashes")
    p.add_argument("--reference", type=Path, required=True)
    p = sub.add_parser("plan", help="write twelve A2 commands without executing FPGA tools")
    p.add_argument("--reference", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("run", help="explicitly synthesize, route and pack one fresh isolated experiment")
    p.add_argument("--reference", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--synthesis", choices=("default", "ce2"), required=True)
    p.add_argument("--refinement", choices=("parallel", "serial"), required=True)
    p.add_argument("--tmg-ripup", action="store_true", help="enable timing-driven ripup during routing")
    p.add_argument("--promote-globals", action="store_true", help="allow automatic global-net promotion during routing")
    p.add_argument("--synthesis-device", choices=("default", "lp"), default="default",
                   help="Yosys device mapping; default retains the inherited synthesis command")
    p.add_argument("--seed", type=int, default=16)
    p.add_argument("--repeat", type=int, default=1)
    p.add_argument("--timeout", type=positive, default=1800)
    p.add_argument("--yosys")
    p.add_argument("--nextpnr")
    p.add_argument("--icepack")
    args = parser.parse_args()
    if args.action == "freeze":
        freeze(args)
    elif args.action == "candidate":
        candidate(args)
    elif args.action == "verify":
        verify_reference(args.reference)
        print("Reference hashes verified")
    elif args.action == "plan":
        plan(args)
    else:
        if args.seed < 0 or args.repeat < 1:
            parser.error("seed must be nonnegative and repeat must be positive")
        return run(args)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
