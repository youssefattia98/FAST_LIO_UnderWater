#!/usr/bin/env python3
"""Validate pose-graph loop diagnostics produced by a benchmark run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--require-full-se3", action="store_true")
    parser.add_argument("--require-one-accepted-loop", action="store_true")
    parser.add_argument("--check-residuals", action="store_true")
    args = parser.parse_args()

    summary = json.loads((args.run / "loop_closure_summary.json").read_text())
    with (args.run / "loops.csv").open(newline="") as stream:
        loops = list(csv.DictReader(stream))
    accepted = [row for row in loops if int(row["accepted"]) == 1]
    failures: list[str] = []
    if args.require_one_accepted_loop and len(accepted) != 1:
        failures.append(f"expected one accepted loop, observed {len(accepted)}")
    if summary.get("graph_loop_factors") != len(accepted):
        failures.append("summary loop-factor count disagrees with loops.csv")
    if args.check_residuals:
        for row in accepted:
            before = float(row["graph_error_before"])
            after = float(row["graph_error_after"])
            translation_before = float(row["translation_error_before"])
            translation_after = float(row["translation_error_after"])
            rotation_before = float(row["rotation_error_before_rad"])
            rotation_after = float(row["rotation_error_after_rad"])
            if not all(
                math.isfinite(value)
                for value in (
                    before,
                    after,
                    translation_before,
                    translation_after,
                    rotation_before,
                    rotation_after,
                )
            ):
                failures.append("accepted loop contains non-finite diagnostics")
            if after > before + 1e-9:
                failures.append("accepted loop increased graph error")
            if translation_after > translation_before + 1e-9:
                failures.append("accepted loop increased translation residual")
            if rotation_after > rotation_before + 1e-9:
                failures.append("accepted loop increased rotation residual")
    if args.require_full_se3:
        request_path = args.run / "manual_loop_request.yaml"
        if not request_path.exists():
            request_path = args.run / "injected_loop.yaml"
        if not request_path.exists():
            failures.append("manual-loop request artifact is missing")

    result = {
        "run": str(args.run),
        "graph_nodes": summary.get("graph_nodes"),
        "graph_factors": summary.get("graph_factors"),
        "accepted_loops": len(accepted),
        "rejected_loops": summary.get("loops_rejected"),
        "failures": failures,
        "passed": not failures,
    }
    (args.run / "pose_graph_check.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
