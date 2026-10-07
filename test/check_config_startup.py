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


def capture_parameters(env):
    from rclpy import init, shutdown, spin_until_future_complete
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient
    from rosidl_runtime_py.convert import message_to_ordereddict

    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    init()
    node = Node("config_characterization")
    try:
        client = AsyncParameterClient(node, "/laser_mapping")
        if not client.wait_for_services(timeout_sec=10.0):
            raise RuntimeError("Parameter services unavailable")
        names_future = client.list_parameters([], depth=0)
        spin_until_future_complete(node, names_future, timeout_sec=10.0)
        names = sorted(names_future.result().result.names)
        values_future = client.get_parameters(names)
        spin_until_future_complete(node, values_future, timeout_sec=10.0)
        descriptors_future = client.describe_parameters(names)
        spin_until_future_complete(node, descriptors_future, timeout_sec=10.0)
        if (len(values_future.result().values) != len(names) or
                len(descriptors_future.result().descriptors) != len(names)):
            raise RuntimeError("Incomplete parameter snapshot")
        return {
            name: {"value": message_to_ordereddict(value),
                   "descriptor": message_to_ordereddict(descriptor)}
            for name, value, descriptor in zip(
                names, values_future.result().values,
                descriptors_future.result().descriptors)
        }
    finally:
        node.destroy_node()
        shutdown()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--executable", type=Path)
    mode.add_argument("--launch", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capture-parameters", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=False)
    cases = [(path.stem, path, []) for path in sorted((root / "config").glob("*.yaml"))]
    if args.capture_parameters and not args.launch:
        cases.insert(0, ("defaults", None, []))
    for profile in ("balanced", "simulation"):
        cases.append((f"profile_{profile}", root / "config/sim.yaml",
                      ["-p", "loop_closure.enable:=true", "-p", f"loop_closure.profile:={profile}"]))
    if args.launch:
        cases.append(("launch_default", root / "config/default.yaml", []))
    results = []
    for index, (name, config, overrides) in enumerate(cases):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(180 + index)
        command = [str(args.executable), "--ros-args"]
        if config is not None:
            command += ["--params-file", str(config)]
        command += overrides
        if args.launch:
            launch_config = config
            if overrides:
                import yaml
                document = yaml.safe_load(config.read_text())
                parameters = document["/**"]["ros__parameters"]
                parameters["loop_closure"]["enable"] = True
                parameters["loop_closure"]["profile"] = name.removeprefix("profile_")
                launch_config = args.output / f"{name}_runtime.yaml"
                launch_config.write_text(yaml.safe_dump(document, sort_keys=False))
            command = ["ros2", "launch", "fast_lio", "mapping.launch.py",
                       "rviz:=false", "use_sim_time:=false"]
            if name != "launch_default":
                command.append(f"config_file:={launch_config}")
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
                if initialized and args.capture_parameters:
                    snapshot = capture_parameters(env)
                    (args.output / f"{name}_parameters.json").write_text(
                        json.dumps(snapshot, indent=2) + "\n")
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
                        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest()
                        if config is not None else None})
        print(f"{name}: pass={passed}", flush=True)
        (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        if not passed:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
