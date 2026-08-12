#!/usr/bin/env python3
"""Analyze one UWFL2 loop-closure benchmark result directory."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import rosbag2_py
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-18:
        return np.eye(3)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.asarray(
        (
            (1.0 - yy - zz, xy - wz, xz + wy),
            (xy + wz, 1.0 - xx - zz, yz - wx),
            (xz - wy, yz + wx, 1.0 - xx - yy),
        ),
        dtype=float,
    )


def transform_from_components(translation, rotation) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = quaternion_to_rotation(
        rotation.x, rotation.y, rotation.z, rotation.w
    )
    result[:3, 3] = (translation.x, translation.y, translation.z)
    return result


def rotation_angle(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def distribution(values: np.ndarray | list[float]) -> dict[str, float | int | None]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "p50": float(np.percentile(array, 50.0)),
        "p95": float(np.percentile(array, 95.0)),
        "p99": float(np.percentile(array, 99.0)),
        "max": float(np.max(array)),
    }


def metadata_metrics(path: Path) -> dict[str, Any]:
    with path.open() as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    topic_counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in metadata["topics_with_message_count"]
    }
    return {
        "storage_identifier": metadata["storage_identifier"],
        "duration_s": float(metadata["duration"]["nanoseconds"]) * 1e-9,
        "starting_time_s": float(metadata["starting_time"]["nanoseconds_since_epoch"])
        * 1e-9,
        "message_count": int(metadata["message_count"]),
        "topic_counts": topic_counts,
    }


def read_trajectories(bag_path: Path) -> dict[str, Any]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=["/Odometry", "/tf", "/tf_static"])
    )

    odom: list[tuple[float, np.ndarray]] = []
    gt: list[tuple[float, np.ndarray]] = []
    world_from_camera = np.eye(4)
    invalid_odom_covariance = 0
    min_odom_covariance_eigenvalues: list[float] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/Odometry":
            message = deserialize_message(data, Odometry)
            transform = transform_from_components(
                message.pose.pose.position, message.pose.pose.orientation
            )
            odom.append((stamp_to_sec(message.header.stamp), transform))
            covariance = np.asarray(message.pose.covariance, dtype=float).reshape(6, 6)
            covariance = 0.5 * (covariance + covariance.T)
            if np.all(np.isfinite(covariance)):
                try:
                    min_odom_covariance_eigenvalues.append(
                        float(np.linalg.eigvalsh(covariance)[0])
                    )
                except np.linalg.LinAlgError:
                    invalid_odom_covariance += 1
            else:
                invalid_odom_covariance += 1
            continue

        message = deserialize_message(data, TFMessage)
        for transform_message in message.transforms:
            parent = transform_message.header.frame_id
            child = transform_message.child_frame_id
            transform = transform_from_components(
                transform_message.transform.translation,
                transform_message.transform.rotation,
            )
            if parent == "World" and child == "camera_init":
                world_from_camera = transform
            elif parent == "World" and child == "BROV_low":
                gt.append((stamp_to_sec(transform_message.header.stamp), transform))

    odom.sort(key=lambda item: item[0])
    gt.sort(key=lambda item: item[0])
    odom_times = np.asarray([item[0] for item in odom], dtype=float)
    odom_poses = np.asarray([world_from_camera @ item[1] for item in odom])
    gt_times = np.asarray([item[0] for item in gt], dtype=float)
    gt_poses = np.asarray([item[1] for item in gt])

    return {
        "odom_times": odom_times,
        "odom_poses": odom_poses,
        "gt_times": gt_times,
        "gt_poses": gt_poses,
        "invalid_odom_covariance": invalid_odom_covariance,
        "min_odom_covariance_eigenvalues": np.asarray(
            min_odom_covariance_eigenvalues, dtype=float
        ),
    }


def unique_samples(times: np.ndarray, poses: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if times.size == 0:
        return times, poses
    _, indices = np.unique(times, return_index=True)
    indices.sort()
    return times[indices], poses[indices]


def nearest_poses(
    source_times: np.ndarray, source_poses: np.ndarray, target_times: np.ndarray
) -> np.ndarray:
    right = np.searchsorted(source_times, target_times, side="left")
    right = np.clip(right, 0, len(source_times) - 1)
    left = np.clip(right - 1, 0, len(source_times) - 1)
    choose_left = np.abs(target_times - source_times[left]) <= np.abs(
        source_times[right] - target_times
    )
    indices = np.where(choose_left, left, right)
    return source_poses[indices]


def trajectory_metrics(data: dict[str, Any]) -> dict[str, Any]:
    odom_times, odom_poses = unique_samples(data["odom_times"], data["odom_poses"])
    gt_times, gt_poses = unique_samples(data["gt_times"], data["gt_poses"])
    result: dict[str, Any] = {
        "odometry_samples": int(len(odom_times)),
        "ground_truth_samples": int(len(gt_times)),
        "odometry_duration_s": float(odom_times[-1] - odom_times[0])
        if len(odom_times) > 1
        else None,
        "odometry_header_interval_s": distribution(np.diff(odom_times)),
        "invalid_odom_covariance": data["invalid_odom_covariance"],
        "odom_covariance_min_eigenvalue": distribution(
            data["min_odom_covariance_eigenvalues"]
        ),
    }
    if len(odom_times) < 2:
        return result

    odom_positions = odom_poses[:, :3, 3]
    result["path_length_m"] = float(
        np.sum(np.linalg.norm(np.diff(odom_positions, axis=0), axis=1))
    )
    result["start_end_translation_m"] = float(
        np.linalg.norm(odom_positions[-1] - odom_positions[0])
    )
    result["start_end_rotation_deg"] = math.degrees(
        rotation_angle(odom_poses[0, :3, :3].T @ odom_poses[-1, :3, :3])
    )

    if len(gt_times) < 2:
        return result
    start = max(float(odom_times[0]), float(gt_times[0]))
    end = min(float(odom_times[-1]), float(gt_times[-1]))
    mask = (odom_times >= start) & (odom_times <= end)
    if int(np.count_nonzero(mask)) < 2:
        return result

    eval_times = odom_times[mask]
    slam = odom_poses[mask]
    truth = nearest_poses(gt_times, gt_poses, eval_times)
    slam_relative_position = slam[:, :3, 3] - slam[0, :3, 3]
    gt_relative_position = truth[:, :3, 3] - truth[0, :3, 3]
    translation_errors = np.linalg.norm(
        slam_relative_position - gt_relative_position, axis=1
    )
    z_errors = np.abs(slam[:, 2, 3] - truth[:, 2, 3])

    slam_start_rotation = slam[0, :3, :3]
    gt_start_rotation = truth[0, :3, :3]
    attitude_errors = []
    for slam_pose, gt_pose in zip(slam, truth):
        slam_relative = slam_start_rotation.T @ slam_pose[:3, :3]
        gt_relative = gt_start_rotation.T @ gt_pose[:3, :3]
        attitude_errors.append(rotation_angle(gt_relative.T @ slam_relative))
    attitude_errors = np.asarray(attitude_errors)

    rpe_translation = []
    rpe_rotation = []
    for index, timestamp in enumerate(eval_times):
        next_index = int(np.searchsorted(eval_times, timestamp + 1.0, side="left"))
        if next_index >= len(eval_times):
            break
        slam_delta = np.linalg.inv(slam[index]) @ slam[next_index]
        gt_delta = np.linalg.inv(truth[index]) @ truth[next_index]
        delta_error = np.linalg.inv(gt_delta) @ slam_delta
        rpe_translation.append(float(np.linalg.norm(delta_error[:3, 3])))
        rpe_rotation.append(rotation_angle(delta_error[:3, :3]))

    result["ground_truth_overlap"] = {
        "start_s": start,
        "end_s": end,
        "duration_s": end - start,
        "samples": int(len(eval_times)),
    }
    result["ate_translation_m"] = {
        **distribution(translation_errors),
        "rmse": float(np.sqrt(np.mean(translation_errors**2))),
        "final": float(translation_errors[-1]),
    }
    result["absolute_z_error_m"] = {
        **distribution(z_errors),
        "rmse": float(np.sqrt(np.mean(z_errors**2))),
        "final": float(z_errors[-1]),
    }
    result["relative_attitude_error_deg"] = {
        **distribution(np.degrees(attitude_errors)),
        "rmse": float(np.degrees(np.sqrt(np.mean(attitude_errors**2)))),
        "final": float(np.degrees(attitude_errors[-1])),
    }
    result["rpe_1s_translation_m"] = {
        **distribution(rpe_translation),
        "rmse": float(np.sqrt(np.mean(np.square(rpe_translation))))
        if rpe_translation
        else None,
    }
    result["rpe_1s_rotation_deg"] = {
        **distribution(np.degrees(rpe_rotation)),
        "rmse": float(np.degrees(np.sqrt(np.mean(np.square(rpe_rotation)))))
        if rpe_rotation
        else None,
    }
    return result


def resource_metrics(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    cpu = []
    rss = []
    threads = []
    with path.open() as stream:
        for row in csv.DictReader(stream):
            cpu.append(float(row["cpu_percent"]))
            rss.append(float(row["rss_mb"]))
            threads.append(float(row["threads"]))
    return {
        "cpu_percent": distribution(cpu),
        "rss_mb": distribution(rss),
        "threads": distribution(threads),
    }


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text()) if path.exists() else {}


def csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open() as stream:
        return list(csv.DictReader(stream))


def numeric_column(
    rows: list[dict[str, str]], column: str, *, status: str | None = None
) -> dict[str, Any]:
    values = []
    for row in rows:
        if status is not None and row.get("status") != status:
            continue
        try:
            values.append(float(row[column]))
        except (KeyError, TypeError, ValueError):
            continue
    return distribution(values)


def timing_metrics(run: Path) -> dict[str, Any]:
    front_end = csv_rows(run / "front_end_timing.csv")
    keyframes = csv_rows(run / "keyframes.csv")
    loops = csv_rows(run / "loops.csv")
    shadow = csv_rows(run / "shadow_rebuilds.csv")
    registrations = csv_rows(run / "reregistrations.csv")
    commits = csv_rows(run / "atomic_commits.csv")
    detections = csv_rows(run / "std_detections.csv")
    shadow_total = []
    for row in shadow:
        try:
            shadow_total.append(
                float(row["reconstruction_time_ms"])
                + float(row["downsample_time_ms"])
                + float(row["tree_build_time_ms"])
            )
        except (KeyError, TypeError, ValueError):
            continue
    return {
        "front_end_scan_ms": numeric_column(front_end, "elapsed_ms"),
        "front_end_lidar_update_ms": numeric_column(
            front_end, "elapsed_ms", status="lidar_update"
        ),
        "graph_append_ms": numeric_column(keyframes, "graph_time_ms"),
        "graph_loop_optimization_ms": numeric_column(loops, "optimization_time_ms"),
        "graph_loop_initial_nis": numeric_column(loops, "initial_nis"),
        "shadow_rebuild_ms": distribution(shadow_total),
        "registration_ms": numeric_column(registrations, "time_ms"),
        "atomic_commit_ms": numeric_column(commits, "elapsed_ms"),
        "std_descriptor_ms": numeric_column(detections, "descriptor_ms"),
        "std_search_ms": numeric_column(detections, "search_ms"),
        "std_verification_ms": numeric_column(detections, "verification_ms"),
    }


def input_delivery_metrics(run: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    summary = read_json(run / "front_end_summary.json")
    bag = Path(manifest.get("bag", ""))
    if not bag.exists() or not (bag / "metadata.yaml").exists():
        return {"front_end_summary": summary}
    source = metadata_metrics(bag / "metadata.yaml")
    runtime = yaml.safe_load((run / "runtime_config.yaml").read_text())
    params = runtime.get("/**", {}).get("ros__parameters", {})
    lidar_topic = params.get("common", {}).get("lid_topic", "")
    imu_topic = params.get("common", {}).get("imu_topic", "")
    topics = source.get("topic_counts", {})
    lidar_expected = int(topics.get(lidar_topic, 0)) if lidar_topic else 0
    imu_expected = int(topics.get(imu_topic, 0)) if imu_topic else 0
    lidar_received = int(summary.get("lidar_callbacks_received", 0))
    imu_received = int(summary.get("imu_callbacks_received", 0))
    return {
        "source_topic_counts": {
            "lidar_topic": lidar_topic,
            "lidar_expected": lidar_expected,
            "imu_topic": imu_topic,
            "imu_expected": imu_expected,
        },
        "callback_delivery": {
            "lidar_received": lidar_received,
            "lidar_missing": max(0, lidar_expected - lidar_received),
            "imu_received": imu_received,
            "imu_missing": max(0, imu_expected - imu_received),
        },
        "front_end_summary": summary,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run = args.run.expanduser().resolve()
    output = args.output.expanduser().resolve() if args.output else run / "metrics.json"
    manifest = json.loads((run / "manifest.json").read_text())
    output_bag = run / "output_bag"
    metrics = {
        "schema_version": 1,
        "run": str(run),
        "label": manifest.get("label"),
        "status": manifest.get("status"),
        "manifest": {
            "git_head": manifest.get("commands", {}).get("git_head", {}).get("output", "").strip(),
            "config_sha256": manifest.get("config_sha256"),
            "bag_metadata_sha256": manifest.get("bag_metadata_sha256"),
            "rate": manifest.get("rate"),
            "clock_mode": manifest.get("clock_mode"),
            "wall_duration_s": manifest.get("wall_duration_s"),
            "bag": manifest.get("bag"),
        },
        "output_bag": metadata_metrics(output_bag / "metadata.yaml"),
        "trajectory": trajectory_metrics(read_trajectories(output_bag)),
        "resources": resource_metrics(run / "resource_samples.csv"),
        "map": manifest.get("map", {}),
        "timing": timing_metrics(run),
        "delivery": input_delivery_metrics(run, manifest),
        "loop_closure": read_json(run / "loop_closure_summary.json"),
    }
    monitor_path = run / "monitor_summary.json"
    if monitor_path.exists():
        metrics["monitor"] = json.loads(monitor_path.read_text())
    output.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
