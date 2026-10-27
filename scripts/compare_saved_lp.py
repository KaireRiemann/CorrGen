#!/usr/bin/env python3
"""Run isolated, bounded LP backend trials on saved 50x50x5 map requests."""
import argparse
import json
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('request_dir', type=Path)
parser.add_argument('--binary', required=True)
parser.add_argument('--solvers', nargs='+', default=['glpk', 'sdlp'])
parser.add_argument('--degree', type=int, default=6)
parser.add_argument('--seconds', type=float, default=60)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
reports = []
for solver in args.solvers:
    started = time.monotonic()
    log_path = args.output / f'{solver}_degree{args.degree}.log'
    with log_path.open('w') as log:
        child = subprocess.Popen([args.binary, str(args.request_dir), solver, str(args.degree)],
                                 stdout=log, stderr=subprocess.STDOUT)
        try:
            code = child.wait(timeout=args.seconds)
            status = 'PASS' if code == 0 else 'FAIL'
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
            code, status = child.returncode, 'TIMEOUT'
    report = dict(solver=solver, degree=args.degree, status=status, exit_code=code,
                  elapsed_s=time.monotonic()-started, request_dir=str(args.request_dir),
                  log=str(log_path))
    reports.append(report)
    print(json.dumps(report), flush=True)
(args.output / f'report_degree{args.degree}.json').write_text(json.dumps(reports, indent=2)+'\n')
