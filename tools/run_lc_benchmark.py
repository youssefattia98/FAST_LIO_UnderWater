#!/usr/bin/env python3
"""Run one isolated, reproducible UWFL2 rosbag benchmark."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import IO, Any

import psutil
import yaml


RECORD_TOPICS = ("/Odometry", "/cloud_registered", "/tf", "/tf_static", "/clock")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_output(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str],
    timeout: float = 60.0,
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        return {
            "command": shlex.join(command),
            "returncode": result.returncode,
            "duration_s": time.monotonic() - started,
            "output": result.stdout,
        }
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return {
            "command": shlex.join(command),
            "returncode": None,
            "duration_s": time.monotonic() - started,
            "timed_out": True,
            "output": output,
        }


def sourced_command(
    command: list[str], ros_setup: Path, workspace_setup: Path
) -> list[str]:
    script = (
        f"source {shlex.quote(str(ros_setup))} && "
        f"source {shlex.quote(str(workspace_setup))} && "
        f"exec {shlex.join(command)}"
    )
    return ["bash", "-lc", script]


def start_process(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str],
    log: IO[str],
) -> subprocess.Popen[str]:
    log.write(f"$ {shlex.join(command)}\n")
    log.flush()
    return subprocess.Popen(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop_process(
    process: subprocess.Popen[str] | None,
    *,
    interrupt_timeout: float = 30.0,
    terminate_timeout: float = 10.0,
) -> int | None:
    if process is None:
        return None
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        return process.wait(timeout=interrupt_timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            return process.wait(timeout=terminate_timeout)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        return process.poll()


class ResourceSampler:
    def __init__(self, root_pid: int, output: Path) -> None:
        self.root_pid = root_pid
        self.output = output
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.started = time.monotonic()
        self.known: dict[int, psutil.Process] = {}

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5.0)

    def _processes(self) -> list[psutil.Process]:
        try:
            root = psutil.Process(self.root_pid)
            processes = [root, *root.children(recursive=True)]
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            processes = []
        unique: dict[int, psutil.Process] = {}
        for process in processes:
            unique[process.pid] = process
        return list(unique.values())

    def _run(self) -> None:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with self.output.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ("elapsed_s", "cpu_percent", "rss_mb", "threads", "processes")
            )
            while not self.stop_event.is_set():
                processes = self._processes()
                cpu_percent = 0.0
                rss_bytes = 0
                thread_count = 0
                live_count = 0
                for process in processes:
                    try:
                        sampled_process = self.known.get(process.pid)
                        if sampled_process is None:
                            sampled_process = process
                            self.known[process.pid] = sampled_process
                            sampled_process.cpu_percent(None)
                        else:
                            cpu_percent += sampled_process.cpu_percent(None)
                        rss_bytes += process.memory_info().rss
                        thread_count += process.num_threads()
                        live_count += 1
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                writer.writerow(
                    (
                        f"{time.monotonic() - self.started:.6f}",
                        f"{cpu_percent:.3f}",
                        f"{rss_bytes / (1024.0 * 1024.0):.3f}",
                        thread_count,
                        live_count,
                    )
                )
                stream.flush()
                self.stop_event.wait(1.0)


def wait_for_service(
    service: str,
    *,
    cwd: Path,
    env: dict[str, str],
    ros_setup: Path,
    workspace_setup: Path,
    launch: subprocess.Popen[str],
    timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    command = sourced_command(
        ["ros2", "service", "list", "--no-daemon", "--spin-time", "1"],
        ros_setup,
        workspace_setup,
    )
    while time.monotonic() < deadline:
        if launch.poll() is not None:
            return False
        result = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=10.0,
            check=False,
        )
        if result.returncode == 0 and service in result.stdout.splitlines():
            return True
        time.sleep(0.5)
    return False


def read_yaml(path: Path) -> Any:
    with path.open() as stream:
        return yaml.safe_load(stream)


def write_runtime_config(args: argparse.Namespace, output: Path) -> Path:
    document = read_yaml(args.config)
    try:
        parameters = document["/**"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Unsupported ROS parameter YAML layout: {args.config}") from exc
    loop_parameters = parameters.setdefault("loop_closure", {})
    if not isinstance(loop_parameters, dict):
        raise ValueError("loop_closure must be a parameter mapping")
    loop_parameters["enable"] = args.loop_closure == "true"
    loop_parameters["automatic_detection_enable"] = args.detection == "true"
    loop_parameters["diagnostics_directory"] = str(output)
    runtime_config = output / "runtime_config.yaml"
    runtime_config.write_text(yaml.safe_dump(document, sort_keys=False))
    return runtime_config


def pcd_point_count(path: Path) -> int | None:
    if not path.exists():
        return None
    with path.open("rb") as stream:
        for _ in range(100):
            line = stream.readline()
            if not line:
                break
            decoded = line.decode("ascii", errors="replace").strip()
            if decoded.startswith("POINTS "):
                return int(decoded.split()[1])
            if decoded.startswith("DATA "):
                break
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument("--domain-id", required=True, type=int)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--loop-closure", choices=("true", "false"), default="false")
    parser.add_argument("--detection", choices=("true", "false"), default="false")
    parser.add_argument("--workspace", type=Path, default=Path("/home/attia/ros2_ws"))
    parser.add_argument("--ros-setup", type=Path, default=Path("/opt/ros/jazzy/setup.bash"))
    parser.add_argument("--drain-seconds", type=float, default=10.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument(
        "--clock-mode",
        choices=("recorded", "generated"),
        default="recorded",
        help="Use the bag's /clock by default; generated asks rosbag2 to synthesize it.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace, source_root: Path) -> None:
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive")
    if args.duration is not None and args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    for path, label in (
        (args.bag, "bag"),
        (args.config, "config"),
        (args.ros_setup, "ROS setup"),
        (args.workspace / "install" / "setup.bash", "workspace setup"),
        (source_root / "launch" / "mapping.launch.py", "mapping launch file"),
    ):
        if not path.exists():
            raise FileNotFoundError(f"{label} does not exist: {path}")
    if args.output.exists():
        raise FileExistsError(
            f"Output already exists: {args.output}. Baseline result directories are immutable."
        )


def main() -> int:
    args = parse_args()
    source_root = Path(__file__).resolve().parents[1]
    args.bag = args.bag.expanduser().resolve()
    args.config = args.config.expanduser().resolve()
    args.output = args.output.expanduser().resolve()
    args.workspace = args.workspace.expanduser().resolve()
    args.ros_setup = args.ros_setup.expanduser().resolve()
    workspace_setup = args.workspace / "install" / "setup.bash"
    validate_args(args, source_root)

    args.output.mkdir(parents=True)
    logs = args.output / "logs"
    ros_logs = args.output / "ros_logs"
    logs.mkdir()
    ros_logs.mkdir()
    shutil.copy2(args.config, args.output / "input_config.yaml")
    runtime_config = write_runtime_config(args, args.output)

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env["ROS_LOG_DIR"] = str(ros_logs)
    env["PYTHONUNBUFFERED"] = "1"

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "status": "running",
        "label": args.label,
        "started_utc": utc_now(),
        "branch_required": "feature/uwfl2-ltaom-loop-closure",
        "loop_closure_requested": args.loop_closure == "true",
        "automatic_detection_requested": args.detection == "true",
        "bag": str(args.bag),
        "bag_metadata_sha256": sha256(args.bag / "metadata.yaml"),
        "config": str(args.config),
        "config_sha256": sha256(args.config),
        "runtime_config": str(runtime_config),
        "runtime_config_sha256": sha256(runtime_config),
        "domain_id": args.domain_id,
        "rate": args.rate,
        "duration_limit_s": args.duration,
        "clock_mode": args.clock_mode,
        "drain_seconds": args.drain_seconds,
        "host": {
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": sys.version,
            "cpu_logical": psutil.cpu_count(logical=True),
            "cpu_physical": psutil.cpu_count(logical=False),
            "memory_bytes": psutil.virtual_memory().total,
        },
        "record_topics": list(RECORD_TOPICS),
        "commands": {},
        "exit_codes": {},
    }
    manifest_path = args.output / "manifest.json"

    def write_manifest() -> None:
        temporary = manifest_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        temporary.replace(manifest_path)

    metadata_env = env.copy()
    manifest["commands"]["git_head"] = command_output(
        ["git", "rev-parse", "HEAD"], cwd=source_root, env=metadata_env
    )
    manifest["commands"]["git_branch"] = command_output(
        ["git", "branch", "--show-current"], cwd=source_root, env=metadata_env
    )
    manifest["commands"]["git_status"] = command_output(
        ["git", "status", "--short", "--branch"], cwd=source_root, env=metadata_env
    )
    manifest["commands"]["default_config_diff"] = command_output(
        ["git", "diff", "--", "config/default.yaml"],
        cwd=source_root,
        env=metadata_env,
    )
    manifest["commands"]["bag_info"] = command_output(
        sourced_command(
            ["ros2", "bag", "info", str(args.bag)], args.ros_setup, workspace_setup
        ),
        cwd=args.output,
        env=metadata_env,
        timeout=120.0,
    )
    manifest["commands"]["uname"] = command_output(
        ["uname", "-a"], cwd=args.output, env=metadata_env
    )
    write_manifest()

    launch: subprocess.Popen[str] | None = None
    recorder: subprocess.Popen[str] | None = None
    monitor: subprocess.Popen[str] | None = None
    player: subprocess.Popen[str] | None = None
    sampler: ResourceSampler | None = None
    opened_logs: list[IO[str]] = []
    failure: str | None = None
    run_started = time.monotonic()

    try:
        launch_log = (logs / "launch.log").open("w")
        opened_logs.append(launch_log)
        launch_command = sourced_command(
            [
                "/usr/bin/time",
                "-v",
                "-o",
                str(args.output / "process_time.txt"),
                "ros2",
                "launch",
                "fast_lio",
                "mapping.launch.py",
                f"config_file:={runtime_config}",
                "rviz:=false",
                "use_sim_time:=true",
            ],
            args.ros_setup,
            workspace_setup,
        )
        manifest["commands"]["launch"] = shlex.join(launch_command)
        launch = start_process(launch_command, cwd=args.output, env=env, log=launch_log)
        sampler = ResourceSampler(launch.pid, args.output / "resource_samples.csv")
        sampler.start()

        if not wait_for_service(
            "/map_save",
            cwd=args.output,
            env=env,
            ros_setup=args.ros_setup,
            workspace_setup=workspace_setup,
            launch=launch,
            timeout=args.startup_timeout,
        ):
            raise RuntimeError("/map_save did not become available before timeout")

        parameter_result = command_output(
            sourced_command(
                [
                    "ros2",
                    "param",
                    "dump",
                    "/laser_mapping",
                    "--no-daemon",
                    "--timeout",
                    "10",
                ],
                args.ros_setup,
                workspace_setup,
            ),
            cwd=args.output,
            env=env,
            timeout=30.0,
        )
        manifest["commands"]["resolved_parameters"] = parameter_result
        (args.output / "resolved_parameters.yaml").write_text(
            parameter_result.get("output", "")
        )

        monitor_log = (logs / "monitor.log").open("w")
        opened_logs.append(monitor_log)
        monitor_command = sourced_command(
            [
                "python3",
                str(source_root / "tools" / "benchmark_monitor.py"),
                "--output",
                str(args.output / "monitor_summary.json"),
            ],
            args.ros_setup,
            workspace_setup,
        )
        manifest["commands"]["monitor"] = shlex.join(monitor_command)
        monitor = start_process(monitor_command, cwd=args.output, env=env, log=monitor_log)

        recorder_log = (logs / "record.log").open("w")
        opened_logs.append(recorder_log)
        recorder_command = sourced_command(
            [
                "ros2",
                "bag",
                "record",
                "--storage",
                "mcap",
                "--output",
                str(args.output / "output_bag"),
                "--node-name",
                f"uwfl2_benchmark_recorder_{args.domain_id}",
                "--topics",
                *RECORD_TOPICS,
            ],
            args.ros_setup,
            workspace_setup,
        )
        manifest["commands"]["record"] = shlex.join(recorder_command)
        recorder = start_process(
            recorder_command, cwd=args.output, env=env, log=recorder_log
        )
        time.sleep(3.0)
        if recorder.poll() is not None:
            raise RuntimeError(f"rosbag recorder exited early with {recorder.returncode}")

        player_log = (logs / "play.log").open("w")
        opened_logs.append(player_log)
        play_args = [
            "ros2",
            "bag",
            "play",
            str(args.bag),
            "--rate",
            str(args.rate),
            "--disable-keyboard-controls",
        ]
        if args.duration is not None:
            play_args.extend(("--playback-duration", str(args.duration)))
        if args.clock_mode == "generated":
            play_args.append("--clock")
        player_command = sourced_command(play_args, args.ros_setup, workspace_setup)
        manifest["commands"]["play"] = shlex.join(player_command)
        player = start_process(player_command, cwd=args.output, env=env, log=player_log)
        player_returncode = player.wait()
        manifest["exit_codes"]["play"] = player_returncode
        if player_returncode != 0:
            raise RuntimeError(f"rosbag playback failed with {player_returncode}")

        time.sleep(args.drain_seconds)
        map_save_result = command_output(
            sourced_command(
                [
                    "ros2",
                    "service",
                    "call",
                    "/map_save",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                args.ros_setup,
                workspace_setup,
            ),
            cwd=args.output,
            env=env,
            timeout=120.0,
        )
        manifest["commands"]["map_save"] = map_save_result
        if map_save_result.get("returncode") != 0 or "success=True" not in map_save_result.get(
            "output", ""
        ):
            raise RuntimeError("map_save service did not report success")
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        manifest["failure"] = failure
    finally:
        manifest["exit_codes"]["record"] = stop_process(recorder, interrupt_timeout=90.0)
        manifest["exit_codes"]["monitor"] = stop_process(monitor)
        if launch is not None:
            manifest["exit_codes"]["launch"] = stop_process(launch)
        if sampler is not None:
            sampler.stop()
        if player is not None and player.poll() is None:
            manifest["exit_codes"]["play_cleanup"] = stop_process(player)
        for stream in opened_logs:
            stream.close()

    manifest["finished_utc"] = utc_now()
    manifest["wall_duration_s"] = time.monotonic() - run_started
    output_bag = args.output / "output_bag"
    if output_bag.exists() and (output_bag / "metadata.yaml").exists():
        manifest["output_bag_metadata_sha256"] = sha256(output_bag / "metadata.yaml")
        manifest["output_bag_info"] = command_output(
            sourced_command(
                ["ros2", "bag", "info", str(output_bag)],
                args.ros_setup,
                workspace_setup,
            ),
            cwd=args.output,
            env=env,
            timeout=120.0,
        )
    map_path = args.output / "test.pcd"
    manifest["map"] = {
        "path": str(map_path),
        "exists": map_path.exists(),
        "size_bytes": map_path.stat().st_size if map_path.exists() else 0,
        "points": pcd_point_count(map_path),
        "sha256": sha256(map_path) if map_path.exists() else None,
    }
    manifest["status"] = "failed" if failure else "complete"
    write_manifest()

    if failure:
        print(f"Benchmark failed: {failure}", file=sys.stderr)
        print(f"Artifacts retained in {args.output}", file=sys.stderr)
        return 1
    print(f"Benchmark complete: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
