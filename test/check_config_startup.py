#!/usr/bin/env python3
"""Start each shipped node YAML and both LC profiles without sensor input."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=False)
    cases = [(path.stem, path, []) for path in sorted((root / "config").glob("*.yaml"))]
    for profile in ("balanced", "simulation"):
        cases.append((f"profile_{profile}", root / "config/sim.yaml",
                      ["-p", "loop_closure.enable:=true", "-p", f"loop_closure.profile:={profile}"]))
    results = []
    for index, (name, config, overrides) in enumerate(cases):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(180 + index)
        command = [str(args.executable), "--ros-args", "--params-file", str(config), *overrides]
        log = args.output / f"{name}.log"
        with log.open("w") as stream:
            process = subprocess.Popen(command, env=env, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 15
                while process.poll() is None and time.monotonic() < deadline:
                    if "Node init finished." in log.read_text():
                        break
                    time.sleep(0.1)
                initialized = process.poll() is None and "Node init finished." in log.read_text()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    code = process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    code = process.wait()
        passed = initialized and code == 0
        results.append({"name": name, "passed": passed, "exit_code": code,
                        "command": command, "domain": env["ROS_DOMAIN_ID"],
                        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest()})
        print(f"{name}: pass={passed}", flush=True)
        (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        if not passed:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
