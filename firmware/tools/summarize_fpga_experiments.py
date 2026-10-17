#!/usr/bin/env python3
"""Summarize an FPGA experiment plan without running tools or changing runs."""

import argparse
from collections import Counter
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import sys


NUMBER = r"(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
CLOCK = re.compile(r"Max frequency for clock\s+['\"](.+?)['\"]:\s*(" + NUMBER +
                   r")\s*MHz\s*\((PASS|FAIL) at\s*(" + NUMBER + r")\s*MHz\)")
RESOURCE = re.compile(r"^\s*(?:Info:\s*)?([A-Z][A-Z0-9_]+):\s*(\d+)\s*/\s*(\d+)\b")
ARTIFACTS = {"netlist": "top.json", "routed_netlist": "routed.json",
             "placement": "top.asc", "image": "top.bin"}
EXPERIMENT_DEFAULTS = {"tmg_ripup": False, "promote_globals": False,
                       "synthesis_device": "default"}
GROUP_FIELDS = ("synthesis", "refinement", *EXPERIMENT_DEFAULTS)


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_json(path):
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def parse_routing_log(text):
    """Use the last observation of each clock, retaining its timing phase."""
    result = {"fmax": {}, "utilization": {}, "evidence": {"fmax": {}, "utilization": {}}}
    routed = False
    for line_number, line in enumerate(text.splitlines(), 1):
        if "Routing complete." in line:
            routed = True
        match = CLOCK.search(line)
        if match:
            name, achieved, status, constraint = match.groups()
            result["fmax"][name] = {"achieved": float(achieved), "constraint": float(constraint)}
            result["evidence"]["fmax"][name] = {
                "origin": "routing_log", "path": "logs/routing.log", "line": line_number,
                "phase": "post_route" if routed else "pre_route_or_unknown", "reported_status": status}
        match = RESOURCE.search(line)
        if match:
            name, used, available = match.groups()
            result["utilization"][name] = {"used": int(used), "available": int(available)}
            result["evidence"]["utilization"][name] = {
                "origin": "routing_log", "path": "logs/routing.log", "line": line_number}
    return result


def valid_metric(family, value):
    if not isinstance(value, dict):
        return False
    fields = ("achieved", "constraint") if family == "fmax" else ("used", "available")
    return all(finite(value.get(field)) for field in fields)


def metrics_for_run(directory, result, issues):
    metrics = {"fmax": {}, "utilization": {}}
    evidence = {family: {} for family in metrics}
    candidates = [(result.get("metrics", {}), "result.json", "result_metrics")]
    report_path = directory / "work/timing.json"
    if report_path.is_file():
        try:
            candidates.append((load_json(report_path), "work/timing.json", "timing_report"))
        except (OSError, ValueError) as error:
            issues.append(f"{report_path}: {error}")
    for candidate, path, origin in candidates:
        if not isinstance(candidate, dict):
            continue
        for family in metrics:
            values = candidate.get(family, {})
            if not isinstance(values, dict):
                continue
            for name, value in values.items():
                if name not in metrics[family] and valid_metric(family, value):
                    metrics[family][name] = value
                    evidence[family][name] = {"origin": origin, "path": path,
                                               "field": ("metrics." if origin == "result_metrics" else "") + family + "." + name}
    routing_log = directory / "logs/routing.log"
    if routing_log.is_file():
        parsed = parse_routing_log(routing_log.read_text(errors="replace"))
        for family in metrics:
            for name, value in parsed[family].items():
                if name not in metrics[family]:
                    metrics[family][name] = value
                    evidence[family][name] = parsed["evidence"][family][name]
    return metrics, evidence


def run_directory(entry, plan_path):
    command = entry.get("command", [])
    if "--output" not in command:
        raise ValueError(f"plan run has no --output argument: {entry.get('name')}")
    value = Path(command[command.index("--output") + 1])
    return (plan_path.parent / value).resolve() if not value.is_absolute() else value.resolve()


def source_files(reference):
    manifest = reference / "reference.json"
    if manifest.is_file():
        return sorted(name.removeprefix("source/firmware/")
                      for name in load_json(manifest).get("files", {}) if name.startswith("source/firmware/"))
    return []


def summarize_run(entry, directory, sources):
    issues = []
    result = {}
    status = "missing"
    if (directory / "result.json").is_file():
        try:
            result = load_json(directory / "result.json")
            status = result.get("status", "invalid") if "elapsed_s" in result else "incomplete"
        except (OSError, ValueError) as error:
            status = "invalid"
            issues.append(f"{directory / 'result.json'}: {error}")
    elif directory.exists():
        status = "incomplete"
    metrics, evidence = metrics_for_run(directory, result, issues)
    stages = {stage["name"]: {field: stage.get(field) for field in
              ("status", "exit_code", "elapsed_s", "started_utc", "log")}
              for stage in result.get("stages", []) if isinstance(stage, dict) and "name" in stage}
    hashes = {name: digest(directory / "work" / path) if (directory / "work" / path).is_file() else None
              for name, path in ARTIFACTS.items()}
    expected_sources = sources or sorted([p.name for p in (directory / "work").glob("*.v")] + ["io.pcf", "Makefile"])
    source_hashes = {name: digest(directory / "work" / name) for name in expected_sources
                     if (directory / "work" / name).is_file()}
    hashes["source"] = (hashlib.sha256(json.dumps(source_hashes, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
                        if expected_sources and len(source_hashes) == len(expected_sources) else None)
    for name, stored in result.get("artifacts", {}).items():
        path = directory / name
        if isinstance(stored, dict) and (not path.is_file() or stored.get("sha256") != digest(path)):
            issues.append(f"artifact differs from recorded hash: {name}")
    warnings = []
    for log in sorted((directory / "logs").glob("*.log")):
        warnings += [{"path": str(log.relative_to(directory)), "line": number, "text": line}
                     for number, line in enumerate(log.read_text(errors="replace").splitlines(), 1)
                     if re.search(r"\bwarning\b", line, re.I)]
    return {**{field: entry.get(field) for field in ("name", "synthesis", "refinement", "seed", "repeat")},
            **{field: result.get(field, entry.get(field, default))
               for field, default in EXPERIMENT_DEFAULTS.items()},
            "directory": str(directory), "status": status, "recorded_status": result.get("status"),
            "error": result.get("error"), "elapsed_s": result.get("elapsed_s"), "stages": stages,
            "metrics": metrics, "evidence": evidence, "hashes": hashes, "source_files": source_hashes,
            "warnings": warnings, "issues": issues}


def bounds(values):
    values = [value for value in values if finite(value)]
    return {"min": min(values), "max": max(values)} if values else {"min": None, "max": None}


def grouped(runs):
    result = []
    def group_key(run):
        return tuple(run.get(field, EXPERIMENT_DEFAULTS.get(field)) for field in GROUP_FIELDS)

    for key in sorted({group_key(run) for run in runs}):
        members = [run for run in runs if group_key(run) == key]
        clocks = sorted({name for run in members for name in run["metrics"]["fmax"]})
        resources = sorted({name for run in members for name in run["metrics"]["utilization"]})
        hashes = {}
        for name in ("source", *ARTIFACTS):
            values = [run["hashes"][name] for run in members if run["hashes"][name]]
            hashes[name] = {"unique_count": len(set(values)), "values": sorted(set(values)),
                            "missing_count": len(members) - len(values)}
        result.append({**dict(zip(GROUP_FIELDS, key)), "planned": len(members),
                       "status_counts": dict(Counter(run["status"] for run in members)),
                       "elapsed_s": bounds(run["elapsed_s"] for run in members),
                       "stage_elapsed_s": {name: bounds(run["stages"].get(name, {}).get("elapsed_s") for run in members)
                                           for name in ("synthesis", "routing", "packing")},
                       "fmax": {clock: {"achieved_mhz": bounds(run["metrics"]["fmax"].get(clock, {}).get("achieved") for run in members),
                                        "constraints_mhz": sorted({run["metrics"]["fmax"][clock]["constraint"] for run in members if clock in run["metrics"]["fmax"]})}
                                for clock in clocks},
                       "utilization": {name: {"used": bounds(run["metrics"]["utilization"].get(name, {}).get("used") for run in members),
                                              "available": sorted({run["metrics"]["utilization"][name]["available"] for run in members if name in run["metrics"]["utilization"]})}
                                       for name in resources}, "hashes": hashes})
    return result


def clock_value(run, constraint):
    values = [value["achieved"] for value in run["metrics"]["fmax"].values()
              if math.isclose(value["constraint"], constraint, abs_tol=1e-6)]
    return min(values) if values else None


def write_csv(path, runs):
    metadata_fields = ["name", "synthesis", "refinement", *EXPERIMENT_DEFAULTS,
                       "seed", "repeat", "status", "elapsed_s", "error"]
    fields = metadata_fields + ["system_fmax_mhz", "lvds_fmax_mhz", "logic_cells", "ram_blocks", "warnings_count"]
    fields += [f"{stage}_{field}" for stage in ("synthesis", "routing", "packing")
               for field in ("status", "exit_code", "elapsed_s")]
    fields += [name + "_sha256" for name in ("source", *ARTIFACTS)]
    fields += ["fmax_json", "utilization_json", "evidence_json", "issues_json"]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            row = {field: run.get(field, EXPERIMENT_DEFAULTS.get(field)) for field in metadata_fields}
            row.update(system_fmax_mhz=clock_value(run, 62.5), lvds_fmax_mhz=clock_value(run, 64),
                       logic_cells=run["metrics"]["utilization"].get("ICESTORM_LC", {}).get("used"),
                       ram_blocks=run["metrics"]["utilization"].get("ICESTORM_RAM", {}).get("used"),
                       warnings_count=len(run["warnings"]))
            row.update({f"{stage}_{field}": run["stages"].get(stage, {}).get(field)
                        for stage in ("synthesis", "routing", "packing") for field in ("status", "exit_code", "elapsed_s")})
            row.update({name + "_sha256": value for name, value in run["hashes"].items()})
            row.update({family + "_json": json.dumps(run["metrics"][family], sort_keys=True) for family in ("fmax", "utilization")})
            row.update(evidence_json=json.dumps(run["evidence"], sort_keys=True), issues_json=json.dumps(run["issues"]))
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plan_path = args.plan.resolve()
    plan = load_json(plan_path)
    reference = Path(plan["reference"])
    reference = (plan_path.parent / reference).resolve() if not reference.is_absolute() else reference.resolve()
    sources = source_files(reference)
    directories = [run_directory(entry, plan_path) for entry in plan["runs"]]
    if args.output.exists() or args.output.is_symlink():
        raise ValueError(f"output already exists: {args.output}")
    output = args.output.resolve()
    for protected in [reference, *directories]:
        if output == protected or protected in output.parents:
            raise ValueError(f"summary output must be outside reference and run directories: {output}")
    runs = [summarize_run(entry, directory, sources) for entry, directory in zip(plan["runs"], directories)]
    summary = {"schema_version": 1, "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
               "plan": str(plan_path), "plan_sha256": digest(plan_path), "reference": str(reference),
               "reference_sha256": plan.get("reference_sha256"), "summarizer_sha256": digest(Path(__file__)),
               "status_counts": dict(Counter(run["status"] for run in runs)), "runs": runs, "groups": grouped(runs),
               "notes": ["Status is recorded by the experiment runner; missing and unfinished runs remain visible.",
                         "Metrics prefer result.json, then timing.json, then the last routing-log observation; evidence records every origin.",
                         "A log observation marked pre_route_or_unknown is an estimate, not routed timing acceptance.",
                         "Source SHA256 hashes a sorted compact JSON map of input-relative paths to SHA256 values; artifact hashes are read from files."]}
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n")
    write_csv(output / "summary.csv", runs)
    print(output / "summary.json")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
