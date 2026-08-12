#!/usr/bin/env python3
"""Compare matched UWFL2-disabled and UWFL2-LC benchmark runs."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial import cKDTree

from analyze_lc_run import distribution, nearest_poses, read_trajectories, rotation_angle


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--map-samples", type=int, default=30000)
    parser.add_argument("--map-overlap-distance", type=float, default=0.30)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def config_path(run: Path) -> Path:
    runtime = run / "runtime_config.yaml"
    return runtime if runtime.exists() else run / "input_config.yaml"


def without_loop_parameters(path: Path) -> dict[str, Any]:
    document = copy.deepcopy(yaml.safe_load(path.read_text()))
    parameters = document.get("/**", {}).get("ros__parameters", {})
    parameters.pop("loop_closure", None)
    return document


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pcd_xyz(path: Path, limit: int) -> np.ndarray:
    header: dict[str, list[str]] = {}
    offset = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"Incomplete PCD header: {path}")
            offset += len(line)
            decoded = line.decode("ascii", errors="strict").strip()
            if not decoded or decoded.startswith("#"):
                continue
            parts = decoded.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    points = int(header["POINTS"][0])
    if header["DATA"][0].lower() != "binary":
        raise ValueError(f"Only binary PCD maps are supported: {path}")
    type_map = {
        ("F", 4): "<f4", ("F", 8): "<f8",
        ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4",
        ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4",
    }
    descriptor = []
    for field, size, kind, count in zip(fields, sizes, types, counts):
        dtype = type_map[(kind.upper(), size)]
        descriptor.append((field, dtype) if count == 1 else (field, dtype, (count,)))
    cloud = np.memmap(path, dtype=np.dtype(descriptor), mode="r", offset=offset, shape=(points,))
    if points > limit:
        indices = np.linspace(0, points - 1, limit, dtype=np.int64)
        xyz = np.column_stack((cloud["x"][indices], cloud["y"][indices], cloud["z"][indices]))
    else:
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
    return np.asarray(xyz, dtype=np.float64)


def map_metrics(left: Path, right: Path, limit: int, threshold: float) -> dict[str, Any]:
    left_hash = file_sha256(left)
    right_hash = file_sha256(right)
    if left_hash == right_hash:
        return {
            "hash_equal": True,
            "samples_each": 0,
            "symmetric_nearest_distance_m": distribution([0.0]),
            "overlap_fraction": 1.0,
        }
    a = pcd_xyz(left, limit)
    b = pcd_xyz(right, limit)
    distance_ab = cKDTree(b).query(a, k=1, workers=-1)[0]
    distance_ba = cKDTree(a).query(b, k=1, workers=-1)[0]
    distances = np.concatenate((distance_ab, distance_ba))
    return {
        "hash_equal": False,
        "samples_each": int(min(len(a), len(b))),
        "symmetric_nearest_distance_m": distribution(distances),
        "overlap_fraction": float(np.mean(distances <= threshold)),
        "overlap_distance_m": threshold,
    }


def trajectory_difference(left: Path, right: Path) -> dict[str, Any]:
    a = read_trajectories(left / "output_bag")
    b = read_trajectories(right / "output_bag")
    if len(a["odom_times"]) == 0 or len(b["odom_times"]) == 0:
        return {}
    start = max(float(a["odom_times"][0]), float(b["odom_times"][0]))
    end = min(float(a["odom_times"][-1]), float(b["odom_times"][-1]))
    mask = (a["odom_times"] >= start) & (a["odom_times"] <= end)
    left_poses = a["odom_poses"][mask]
    right_poses = nearest_poses(b["odom_times"], b["odom_poses"], a["odom_times"][mask])
    translation = np.linalg.norm(left_poses[:, :3, 3] - right_poses[:, :3, 3], axis=1)
    rotation = [
        np.degrees(rotation_angle(x[:3, :3].T @ y[:3, :3]))
        for x, y in zip(left_poses, right_poses)
    ]
    return {
        "samples": int(len(left_poses)),
        "translation_m": distribution(translation),
        "rotation_deg": distribution(rotation),
    }


def get(data: dict[str, Any], *keys: str) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def compact_metrics(metrics: dict[str, Any]) -> dict[str, Any]:
    trajectory = metrics.get("trajectory", {})
    return {
        "ate_translation_m": trajectory.get("ate_translation_m"),
        "rpe_1s_translation_m": trajectory.get("rpe_1s_translation_m"),
        "rpe_1s_rotation_deg": trajectory.get("rpe_1s_rotation_deg"),
        "start_end_translation_m": trajectory.get("start_end_translation_m"),
        "start_end_rotation_deg": trajectory.get("start_end_rotation_deg"),
        "resources": metrics.get("resources", {}),
        "timing": metrics.get("timing", {}),
        "delivery": metrics.get("delivery", {}),
        "output_topic_counts": get(metrics, "output_bag", "topic_counts"),
        "invalid_odom_covariance": trajectory.get("invalid_odom_covariance"),
        "loop_closure": metrics.get("loop_closure", {}),
    }


def fmt(value: Any) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def main() -> int:
    args = parse_args()
    baseline = args.baseline.resolve()
    candidate = args.candidate.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    left = load_json(baseline / "metrics.json")
    right = load_json(candidate / "metrics.json")
    same_bag = get(left, "manifest", "bag_metadata_sha256") == get(
        right, "manifest", "bag_metadata_sha256"
    )
    same_non_loop_config = without_loop_parameters(config_path(baseline)) == (
        without_loop_parameters(config_path(candidate))
    )
    comparison = {
        "baseline": str(baseline),
        "candidate": str(candidate),
        "same_source_bag": same_bag,
        "same_non_loop_parameters": same_non_loop_config,
        "baseline_metrics": compact_metrics(left),
        "candidate_metrics": compact_metrics(right),
        "trajectory_difference": trajectory_difference(baseline, candidate),
        "map_consistency": map_metrics(
            baseline / "test.pcd", candidate / "test.pcd",
            args.map_samples, args.map_overlap_distance,
        ),
    }
    (output / "comparison.json").write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n"
    )
    rows = [
        ("ATE RMSE [m]", get(left, "trajectory", "ate_translation_m", "rmse"), get(right, "trajectory", "ate_translation_m", "rmse")),
        ("ATE max [m]", get(left, "trajectory", "ate_translation_m", "max"), get(right, "trajectory", "ate_translation_m", "max")),
        ("ATE final [m]", get(left, "trajectory", "ate_translation_m", "final"), get(right, "trajectory", "ate_translation_m", "final")),
        ("CPU p95 [%]", get(left, "resources", "cpu_percent", "p95"), get(right, "resources", "cpu_percent", "p95")),
        ("RAM max [MiB]", get(left, "resources", "rss_mb", "max"), get(right, "resources", "rss_mb", "max")),
        ("Scan p95 [ms]", get(left, "timing", "front_end_scan_ms", "p95"), get(right, "timing", "front_end_scan_ms", "p95")),
        ("Graph p95 [ms]", get(left, "timing", "graph_append_ms", "p95"), get(right, "timing", "graph_append_ms", "p95")),
        ("Tree rebuild p95 [ms]", get(left, "timing", "shadow_rebuild_ms", "p95"), get(right, "timing", "shadow_rebuild_ms", "p95")),
        ("Registration p95 [ms]", get(left, "timing", "registration_ms", "p95"), get(right, "timing", "registration_ms", "p95")),
        ("Commit p95 [ms]", get(left, "timing", "atomic_commit_ms", "p95"), get(right, "timing", "atomic_commit_ms", "p95")),
    ]
    markdown = [
        "# UWFL2 Loop-Closure Evaluation",
        "",
        f"- Same source bag: **{same_bag}**",
        f"- Same non-loop parameters: **{same_non_loop_config}**",
        f"- Map overlap: **{comparison['map_consistency']['overlap_fraction']:.6f}**",
        "",
        "| Metric | UWFL2 disabled | UWFL2-LC |",
        "| --- | ---: | ---: |",
    ]
    markdown.extend(f"| {name} | {fmt(a)} | {fmt(b)} |" for name, a, b in rows)
    markdown.append("")
    (output / "REPORT.md").write_text("\n".join(markdown))
    print(json.dumps(comparison, indent=2, sort_keys=True))
    return 0 if same_bag and same_non_loop_config else 1


if __name__ == "__main__":
    raise SystemExit(main())
