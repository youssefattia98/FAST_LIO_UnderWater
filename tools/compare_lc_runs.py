#!/usr/bin/env python3
"""Compare two UWFL2 benchmark runs, including exact output sequencing."""

from __future__ import annotations

import argparse
import json
import math
import hashlib
from pathlib import Path
from typing import Any

import numpy as np
import yaml
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from analyze_lc_run import read_trajectories, rotation_angle


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-no-loop", action="store_true")
    parser.add_argument("--check-message-counts", action="store_true")
    parser.add_argument("--check-timestamps", action="store_true")
    parser.add_argument("--check-map", action="store_true")
    parser.add_argument("--position-tolerance", type=float, default=1e-9)
    parser.add_argument("--rotation-tolerance-rad", type=float, default=1e-9)
    parser.add_argument("--timestamp-tolerance", type=float, default=1e-9)
    parser.add_argument("--latency-regression-fraction", type=float, default=0.02)
    return parser.parse_args()


def header_stamps(path: Path) -> dict[str, np.ndarray]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topics = {"/Odometry": Odometry}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    result: dict[str, list[int]] = {topic: [] for topic in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, topics[topic])
        result[topic].append(
            message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        )
    return {topic: np.asarray(values) for topic, values in result.items()}


def max_pose_difference(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> tuple[float | None, float | None]:
    if (not len(baseline["odom_poses"])
            or len(baseline["odom_poses"]) != len(candidate["odom_poses"])
            or not np.isfinite(baseline["odom_poses"]).all()
            or not np.isfinite(candidate["odom_poses"]).all()):
        return None, None
    position = np.linalg.norm(
        baseline["odom_poses"][:, :3, 3] - candidate["odom_poses"][:, :3, 3], axis=1
    )
    rotation = []
    for a, b in zip(baseline["odom_poses"], candidate["odom_poses"]):
        matrix_difference = np.max(np.abs(a[:3, :3] - b[:3, :3]))
        rotation.append(
            0.0
            if matrix_difference <= np.finfo(float).eps
            else rotation_angle(a[:3, :3].T @ b[:3, :3])
        )
    return float(np.max(position, initial=0.0)), float(np.max(rotation, initial=0.0))


def matching_maps(baseline: dict[str, Any], candidate: dict[str, Any],
                  baseline_run=None, candidate_run=None) -> bool:
    left, right = baseline.get("map", {}), candidate.get("map", {})
    matching = bool(left.get("exists") and right.get("exists")
                and left.get("points", 0) and right.get("points", 0)
                and left.get("sha256") and left["sha256"] == right.get("sha256"))
    if not matching:
        return False
    for evidence, run in ((left, baseline_run), (right, candidate_run)):
        path = Path(evidence.get("path", ""))
        if not path.is_file() or not path.stat().st_size:
            return False
        if hashlib.sha256(path.read_bytes()).hexdigest() != evidence["sha256"]:
            return False
        if run is not None:
            manifest = json.loads((run / "manifest.json").read_text())
            if manifest.get("map") != evidence or path.resolve() != (run / "test.pcd").resolve():
                return False
    return True


def disabled_loop(path: Path) -> bool:
    document = yaml.safe_load((path / "runtime_config.yaml").read_text())
    try:
        disabled = document["/**"]["ros__parameters"]["loop_closure"]["enable"] is False
    except (KeyError, TypeError):
        return False
    return disabled and not (path / "keyframes.csv").exists()


def main() -> int:
    args = parse_args()
    baseline = args.baseline.resolve()
    candidate = args.candidate.resolve()
    baseline_metrics = json.loads((baseline / "metrics.json").read_text())
    candidate_metrics = json.loads((candidate / "metrics.json").read_text())
    baseline_stamps = header_stamps(baseline / "output_bag")
    candidate_stamps = header_stamps(candidate / "output_bag")
    baseline_trajectory = read_trajectories(baseline / "output_bag")
    candidate_trajectory = read_trajectories(candidate / "output_bag")
    position_max, rotation_max = max_pose_difference(
        baseline_trajectory, candidate_trajectory
    )

    failures: list[str] = []
    stamp_differences: dict[str, float | None] = {}
    for topic in baseline_stamps:
        left, right = baseline_stamps[topic], candidate_stamps[topic]
        if len(left) != len(right):
            stamp_differences[topic] = None
            if args.check_message_counts:
                failures.append(f"{topic} count differs: {len(left)} != {len(right)}")
            if args.check_timestamps:
                failures.append(f"{topic} timestamp comparison unavailable: unequal counts")
        else:
            difference = float(np.max(np.abs(left - right), initial=0)) / 1e9
            stamp_differences[topic] = difference
            if args.check_timestamps and difference > args.timestamp_tolerance:
                failures.append(f"{topic} timestamp difference {difference:.3e} s")

    if position_max is None or rotation_max is None:
        failures.append("pose comparison unavailable: empty, unequal or non-finite samples")
    elif position_max > args.position_tolerance:
        failures.append(f"maximum position difference {position_max:.3e} m")
    if rotation_max is not None and rotation_max > args.rotation_tolerance_rad:
        failures.append(f"maximum rotation difference {rotation_max:.3e} rad")
    if baseline_trajectory["odom_frames"] != candidate_trajectory["odom_frames"]:
        failures.append("odometry frame identifiers or frame sequence differ")
    if baseline_metrics.get("status") != "complete" or candidate_metrics.get("status") != "complete":
        failures.append("run status is not complete")
    map_hash_equal = matching_maps(baseline_metrics, candidate_metrics, baseline, candidate)
    if args.check_map and not map_hash_equal:
        failures.append("map evidence missing, empty, or SHA-256 differs")
    if args.require_no_loop:
        if not disabled_loop(baseline) or not disabled_loop(candidate):
            failures.append("both runs must explicitly disable loop closure")

    baseline_lag = baseline_metrics.get("monitor", {}).get(
        "ros_time_processing_lag_s", {}
    ).get("p95")
    candidate_lag = candidate_metrics.get("monitor", {}).get(
        "ros_time_processing_lag_s", {}
    ).get("p95")
    latency_ratio = None
    if baseline_lag and candidate_lag:
        if not math.isfinite(baseline_lag) or not math.isfinite(candidate_lag):
            failures.append("non-finite output-lag evidence")
        else:
            latency_ratio = candidate_lag / baseline_lag
        if latency_ratio is not None and latency_ratio > 1.0 + args.latency_regression_fraction:
            failures.append(
                f"p95 output-lag proxy regressed by {(latency_ratio - 1) * 100:.2f}%"
            )

    report = {
        "baseline": str(baseline),
        "candidate": str(candidate),
        "passed": not failures,
        "failures": failures,
        "maximum_position_difference_m": position_max,
        "maximum_rotation_difference_rad": rotation_max,
        "maximum_header_stamp_difference_s": stamp_differences,
        "output_lag_p95_ratio": latency_ratio,
        "map_hash_equal": map_hash_equal,
    }
    output = args.output.resolve() if args.output else candidate / "comparison.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
