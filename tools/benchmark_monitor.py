#!/usr/bin/env python3
"""Collect low-overhead ROS-time cadence and covariance metrics for UWFL2."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def distribution(values: list[float]) -> dict[str, float | int | None]:
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if finite.size == 0:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    return {
        "count": int(finite.size),
        "mean": float(np.mean(finite)),
        "p50": float(np.percentile(finite, 50.0)),
        "p95": float(np.percentile(finite, 95.0)),
        "p99": float(np.percentile(finite, 99.0)),
        "max": float(np.max(finite)),
    }


class BenchmarkMonitor(Node):
    def __init__(self, output: Path) -> None:
        super().__init__(
            "uwfl2_benchmark_monitor",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.output = output
        self.wall_start = time.monotonic()
        self.header_stamps: list[float] = []
        self.ros_lag_s: list[float] = []
        self.min_position_cov_eigenvalues: list[float] = []
        self.invalid_pose_count = 0
        self.invalid_covariance_count = 0
        self.nonmonotonic_stamp_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Odometry, "/Odometry", self.odom_callback, qos
        )

    def odom_callback(self, msg: Odometry) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        if self.header_stamps and stamp < self.header_stamps[-1] - 1e-9:
            self.nonmonotonic_stamp_count += 1
        self.header_stamps.append(stamp)

        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if ros_now > 0.0 and stamp > 0.0:
            self.ros_lag_s.append(ros_now - stamp)

        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.invalid_pose_count += 1

        covariance = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        covariance = 0.5 * (covariance + covariance.T)
        if not np.all(np.isfinite(covariance)):
            self.invalid_covariance_count += 1
            return
        try:
            minimum = float(np.linalg.eigvalsh(covariance[:3, :3])[0])
        except np.linalg.LinAlgError:
            self.invalid_covariance_count += 1
            return
        self.min_position_cov_eigenvalues.append(minimum)

    def write_summary(self) -> None:
        intervals = np.diff(np.asarray(self.header_stamps, dtype=float))
        intervals_list = intervals.tolist() if intervals.size else []
        summary = {
            "wall_duration_s": time.monotonic() - self.wall_start,
            "odometry_messages": len(self.header_stamps),
            "first_header_stamp_s": self.header_stamps[0] if self.header_stamps else None,
            "last_header_stamp_s": self.header_stamps[-1] if self.header_stamps else None,
            "header_interval_s": distribution(intervals_list),
            "ros_time_processing_lag_s": distribution(self.ros_lag_s),
            "position_covariance_min_eigenvalue": distribution(
                self.min_position_cov_eigenvalues
            ),
            "invalid_pose_count": self.invalid_pose_count,
            "invalid_covariance_count": self.invalid_covariance_count,
            "nonmonotonic_stamp_count": self.nonmonotonic_stamp_count,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
        temporary.replace(self.output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = BenchmarkMonitor(args.output.resolve())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
