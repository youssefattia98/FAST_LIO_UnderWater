#!/usr/bin/env python3
"""Compare unchanged geometry at shared sensor timestamps, without spatial alignment.

Prediction publishing coalesces differently between replays. Report that coverage
explicitly; require matching dense-map hashes for sonar cases as independent evidence.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from analyze_lc_run import read_trajectories
from compare_lc_runs import disabled_loop, matching_maps


def odometry_fields(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/Odometry"]))
    result = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        message = deserialize_message(data, Odometry)
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        # Compare the published camera_init-frame covariance/twist without alignment.
        result[stamp] = (np.asarray(list(message.pose.covariance) + list(message.twist.covariance)),
                         np.asarray([message.twist.twist.linear.x, message.twist.twist.linear.y,
                                     message.twist.twist.linear.z, message.twist.twist.angular.x,
                                     message.twist.twist.angular.y, message.twist.twist.angular.z]))
    return result


def shared_pose_comparison(left, right):
    first = np.rint(left["odom_times"] * 1e9).astype(np.int64)
    second = np.rint(right["odom_times"] * 1e9).astype(np.int64)
    common, i, j = np.intersect1d(first, second, return_indices=True)
    if not len(common):
        return {"shared_samples": 0, "position_max_m": None, "rotation_matrix_max": None}
    a, b = left["odom_poses"][i], right["odom_poses"][j]
    return {
        "baseline_samples": len(first), "candidate_samples": len(second),
        "shared_samples": len(common),
        "shared_fraction": len(common) / min(len(first), len(second)),
        "shared_time_span_fraction": float((common[-1] - common[0]) /
            max(1, min(first[-1], second[-1]) - max(first[0], second[0]))),
        "position_max_m": float(np.max(np.linalg.norm(a[:, :3, 3] - b[:, :3, 3], axis=1))),
        "rotation_matrix_max": float(np.max(np.abs(a[:, :3, :3] - b[:, :3, :3]))),
        "baseline_final_stamp": float(left["odom_times"][-1]),
        "candidate_final_stamp": float(right["odom_times"][-1]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", type=Path,
                        help="Omit to compare repetition 0 versus 1 of the baseline.")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    reports = []
    for bag in ("sim3", "backAndforth_CSSN3_processed"):
        for variant in ("FL2", "INS", "UWFL2"):
            baseline = args.baseline / f"{bag}_{variant}_0"
            candidate = ((args.candidate / f"{bag}_{variant}_0") if args.candidate
                         else (args.baseline / f"{bag}_{variant}_1"))
            first = read_trajectories(baseline / "output_bag")
            second = read_trajectories(candidate / "output_bag")
            report = shared_pose_comparison(first, second)
            first_fields = odometry_fields(baseline / "output_bag")
            second_fields = odometry_fields(candidate / "output_bag")
            shared_fields = first_fields.keys() & second_fields.keys()
            report["covariance_max_difference"] = max(
                (float(np.max(np.abs(first_fields[t][0] - second_fields[t][0])))
                 for t in shared_fields), default=None)
            report["twist_max_difference"] = max(
                (float(np.max(np.abs(first_fields[t][1] - second_fields[t][1])))
                 for t in shared_fields), default=None)
            first_metrics = json.loads((baseline / "metrics.json").read_text())
            second_metrics = json.loads((candidate / "metrics.json").read_text())
            report.update({"bag": bag, "variant": variant,
                           "map_hash_equal": matching_maps(first_metrics, second_metrics),
                           "invalid_covariance_baseline": first["invalid_odom_covariance"],
                           "invalid_covariance_candidate": second["invalid_odom_covariance"],
                           "baseline": str(baseline), "candidate": str(candidate)})
            report["minimum_covariance_eigenvalue"] = min(
                float(np.min(first["min_odom_covariance_eigenvalues"], initial=np.inf)),
                float(np.min(second["min_odom_covariance_eigenvalues"], initial=np.inf)))
            report["passed"] = bool(
                disabled_loop(baseline) and disabled_loop(candidate)
                and first_metrics.get("status") == "complete"
                and second_metrics.get("status") == "complete"
                and report["shared_samples"] >= 100
                and report["shared_time_span_fraction"] >= 0.9
                and np.isfinite(report["position_max_m"])
                and report["position_max_m"] <= 1e-9
                and np.isfinite(report["rotation_matrix_max"])
                and report["rotation_matrix_max"] <= 1e-9
                and report["covariance_max_difference"] is not None
                and np.isfinite(report["covariance_max_difference"])
                and report["covariance_max_difference"] <= 1e-9
                and report["twist_max_difference"] is not None
                and np.isfinite(report["twist_max_difference"])
                and report["twist_max_difference"] <= 1e-9
                and first["invalid_odom_covariance"] == 0
                and second["invalid_odom_covariance"] == 0
                and np.isfinite(report["minimum_covariance_eigenvalue"])
                and report["minimum_covariance_eigenvalue"] >= -1e-9
                and (variant == "INS" or report["map_hash_equal"]))
            reports.append(report)
    args.output.write_text(json.dumps(reports, indent=2) + "\n")
    for report in reports:
        print(f"{report['bag']} {report['variant']}: pass={report['passed']} "
              f"shared={report['shared_samples']} pos={report['position_max_m']} "
              f"rot={report['rotation_matrix_max']} map={report['map_hash_equal']}")
    return 0 if all(report["passed"] for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
