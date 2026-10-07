#!/usr/bin/env python3
"""Short, isolated FL2/INS/UWFL2 characterization replays; never edits input bags/configs."""

import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workspace-setup", required=True, type=Path)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--rate", type=float, default=5.0)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=False)
    cases = (
        ("sim3", "sim.yaml"),
        ("backAndforth_CSSN3_processed", "backAndforth_CSSN3_processed.yaml"),
    )
    results = []
    for bag, configuration in cases:
        document = yaml.safe_load((root / "config" / configuration).read_text())
        for variant in ("FL2", "INS", "UWFL2"):
            runtime = copy.deepcopy(document)
            parameters = runtime["/**"]["ros__parameters"]
            parameters["loop_closure"]["enable"] = False
            for sensor in ("dvl", "pressure", "magnetometer"):
                parameters[sensor]["enable"] = variant != "FL2"
            if variant == "INS":
                parameters["sonar"]["topic"] = "/refactor_test_no_sonar"
            configuration_path = args.output / f"{bag}_{variant}.yaml"
            configuration_path.write_text(yaml.safe_dump(runtime, sort_keys=False))
            for repetition in range(args.repeats):
                label = f"{bag}_{variant}_{repetition}"
                output = args.output / label
                command = [sys.executable, str(root / "tools/run_lc_benchmark.py"),
                           "--label", label, "--domain-id", str(211 + len(results)),
                           "--bag", str(Path("/home/attia/ros2_ws/bags/DONE") / bag),
                           "--config", str(configuration_path), "--output", str(output),
                           "--workspace-setup", str(args.workspace_setup),
                           "--duration", str(args.duration), "--rate", str(args.rate),
                           "--loop-closure", "false", "--map-publication", "false",
                           "--map-save", "false" if variant == "INS" else "true",
                           "--drain-seconds", "3"]
                with (args.output / f"{label}.log").open("w") as stream:
                    completed = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
                results.append({"label": label, "command": command, "returncode": completed.returncode})
                (args.output / "runs.json").write_text(json.dumps(results, indent=2) + "\n")
                print(f"{label}: replay exit={completed.returncode}", flush=True)
                if completed.returncode:
                    return 1
                with (output / "analysis.log").open("w") as stream:
                    analysis = subprocess.run(
                        [sys.executable, str(root / "tools/analyze_lc_run.py"), "--run", str(output)],
                        stdout=stream, stderr=subprocess.STDOUT)
                if analysis.returncode:
                    print(f"Analysis failed: {output / 'analysis.log'}", flush=True)
                    return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
