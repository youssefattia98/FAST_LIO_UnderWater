#!/usr/bin/env python3
"""Exercise live sonar outage/recovery and orderly shutdown on an isolated domain."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = "230"
    rclpy.init()
    node = rclpy.create_node("sensor_timeline_fixture")
    imu = node.create_publisher(Imu, "/fixture/imu", 1000)
    sonar = node.create_publisher(PointCloud2, "/fixture/sonar", 100)
    outputs = []
    subscription = node.create_subscription(Odometry, "/Odometry", outputs.append, 1000)
    command = [args.executable, "--ros-args", "-p", "imu.topic:=/fixture/imu",
               "-p", "sonar.topic:=/fixture/sonar", "-p", "imu.frequency:=200.0",
               "-p", "mapping.minimum_scan_points:=1000"]
    with (args.output / "node.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 15.0
            while (imu.get_subscription_count() == 0 or sonar.get_subscription_count() == 0
                   or subscription.get_publisher_count() == 0):
                assert process.poll() is None and time.monotonic() < deadline
                rclpy.spin_once(node, timeout_sec=0.01)

            def stamp(seconds):
                nanoseconds = round(seconds * 1e9)
                return Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)

            def cloud(seconds):
                sonar.publish(create_cloud_xyz32(
                    Header(stamp=stamp(seconds), frame_id="fixture_sonar"),
                    [(float(x), float(y), -1.0) for x in (1, 2) for y in (-1, 0, 1)]))

            def samples(begin, end, scan_times=()):
                for nanoseconds in range(round(begin * 1e9), round(end * 1e9) + 1, 5000000):
                    seconds = nanoseconds / 1e9
                    if any(abs(seconds - epoch) < 1e-8 for epoch in scan_times):
                        cloud(seconds)
                    message = Imu()
                    message.header.stamp = stamp(seconds)
                    message.linear_acceleration.z = 9.81
                    message.orientation_covariance[0] = -1.0
                    imu.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.001)

            def drain():
                end = time.monotonic() + 0.4
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)

            samples(1.0, 2.0, (1.0, 1.2, 1.4, 1.6, 1.8))
            drain()
            first_count = len(outputs)
            samples(2.005, 2.8)
            drain()
            outage_count = len(outputs)
            cloud(2.55)
            samples(2.805, 3.2, (3.0,))
            drain()
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in outputs]
            print({"scan": first_count, "outage": outage_count,
                   "total": len(outputs), "stamps": stamps[-5:]}, flush=True)
            assert first_count > 0 and outage_count > first_count and len(outputs) > outage_count
            # Startup may publish the same epoch; initialized output must not go backwards.
            assert all(a <= b for a, b in zip(stamps, stamps[1:]))
            assert stamps[-1] >= 3100000000
            report = {"command": command, "scan_phase_outputs": first_count,
                      "outage_outputs": outage_count - first_count,
                      "recovery_outputs": len(outputs) - outage_count,
                      "first_stamp_ns": stamps[0], "final_stamp_ns": stamps[-1]}
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            code = process.wait(timeout=10)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
    assert code == 0
    report["exit_code"] = code
    (args.output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
