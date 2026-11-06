#!/usr/bin/env python3
"""Checked temporary FPGA loader with verified original-image restoration.

This script accesses hardware only when invoked without --check-only. It does
not alter firmware/top.bin, EEPROM, boot files, or any installed application.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ORIGINAL_HASH = 'dcdf0733ef1ee8088b82381148475a6e4de87c6f00b8d14480a46d48621d31f6'
ORIGINAL = ROOT / 'firmware/top.bin'
OUTPUT = ROOT / 'build/u25-clock-probe-20261106'
LOADER = OUTPUT / 'load_checked'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_image(path, expected):
    path = path.resolve(strict=True)
    if not path.is_file() or path.stat().st_size != 32220:
        raise ValueError(f'Expected a 32220-byte FPGA image: {path}')
    actual = sha256(path)
    if actual != expected:
        raise ValueError(f'Image SHA-256 mismatch for {path}: {actual} != {expected}')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', nargs='?', type=Path)
    parser.add_argument('--sha256', help='Expected SHA-256 for the explicit image')
    parser.add_argument('--restore', action='store_true', help='Restore the verified original FPGA image')
    parser.add_argument('--check-only', action='store_true', help='Validate inputs without hardware access')
    parser.add_argument('--receipt', type=Path, help='New JSON receipt path; log uses the same basename')
    args = parser.parse_args()
    if args.restore and (args.image is not None or args.sha256 is not None):
        parser.error('--restore cannot be combined with image or --sha256')
    if not args.restore and (args.image is None or args.sha256 is None):
        parser.error('Supply image and --sha256, or --restore')
    rollback = checked_image(ORIGINAL, ORIGINAL_HASH)
    image = rollback if args.restore else checked_image(args.image, args.sha256.lower())
    if not LOADER.is_file():
        raise ValueError(f'Missing checked loader: {LOADER}')
    command = [str(LOADER), str(image), str(rollback)]
    receipt = {
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'operation': 'restore' if args.restore else 'load',
        'command': command,
        'input_sha256': sha256(image),
        'rollback_sha256': sha256(rollback),
        'loader_sha256': sha256(LOADER),
        'library_sha256': sha256(ROOT / 'build/libcariboulite.so'),
        'rf_tested': False,
        'hardware_accessed': False,
    }
    if args.check_only:
        print(json.dumps(receipt, indent=2))
        return 0
    if OUTPUT.is_symlink():
        raise ValueError(f'Refusing a symlink output directory: {OUTPUT}')
    OUTPUT.mkdir(parents=True, exist_ok=True)
    output = args.receipt or OUTPUT / ('programming-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '.json')
    # Keep the final pathname unresolved: exclusive creation must also reject
    # an existing symlink rather than create the symlink's target.
    output = output.absolute()
    log_path = output.with_suffix('.log')
    # Refuse receipt/log overwrites before accessing the board.
    with output.open('x') as receipt_stream, log_path.open('x') as log:
        receipt['hardware_accessed'] = True
        json.dump(receipt, receipt_stream, indent=2)
        receipt_stream.flush()
        try:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=30)
            receipt['exit_code'] = result.returncode
        except subprocess.TimeoutExpired:
            receipt['exit_code'] = 124
            receipt['error'] = 'Loader timed out; inspect hardware state before continuing'
        log.flush()
        contents = log_path.read_text()
        receipt['candidate_programmed'] = 'CANDIDATE_PROGRAMMED=1' in contents
        receipt['rollback_restored'] = 'ROLLBACK_RESTORED=1' in contents
        receipt['finished_utc'] = datetime.now(timezone.utc).isoformat()
        receipt['log_sha256'] = sha256(log_path)
        receipt_stream.seek(0)
        receipt_stream.truncate()
        json.dump(receipt, receipt_stream, indent=2)
        receipt_stream.write('\n')
    print(json.dumps(receipt, indent=2))
    print(f'Log: {log_path}')
    return receipt['exit_code']


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f'Input validation failed: {error}', file=sys.stderr)
        raise SystemExit(64)
