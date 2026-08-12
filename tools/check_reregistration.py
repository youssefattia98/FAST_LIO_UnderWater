#!/usr/bin/env python3
"""Check latest-scan registration diagnostics."""

import argparse
import csv
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--require-residual-decrease", action="store_true")
    parser.add_argument("--require-full-se3", action="store_true")
    args = parser.parse_args()
    with (args.run / "reregistrations.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    valid = [row for row in rows if int(row["valid"]) == 1]
    failures = []
    if not valid:
        failures.append("no valid registration")
    if args.require_residual_decrease:
        for row in valid:
            if float(row["final_mean_m"]) > float(row["initial_mean_m"]) + 1e-8:
                failures.append("mean residual increased")
    if args.require_full_se3:
        for row in valid:
            if float(row["information_min_eigenvalue"]) <= 0.0:
                failures.append("registration information is rank deficient")
    print("registration check:", "PASS" if not failures else "; ".join(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
