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
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--early-recovery", action="store_true",
                        help="Deliver the recovery scan before its IMU watermark")
    parser.add_argument("--informative", action="store_true",
                        help="Keep a dense planar scan instead of filtering all returns")
    parser.add_argument("--scan-delay-seconds", type=float, default=0.0,
                        help="Delay delivery of informative scans after startup")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = "230"
    rclpy.init()
    node = rclpy.create_node("sensor_timeline_fixture")
    imu = node.create_publisher(Imu, "/fixture/imu", 1000)
    sonar = node.create_publisher(PointCloud2, "/fixture/sonar", 100)
    outputs = []
    health = []
    subscription = node.create_subscription(Odometry, "/Odometry", outputs.append, 1000)
    health_subscription = node.create_subscription(
        DiagnosticArray, "/uwfl2/navigation_health", health.append, 10)
    command = [args.executable, "--ros-args", "-p", "imu.topic:=/fixture/imu",
               "-p", "sonar.topic:=/fixture/sonar", "-p", "imu.frequency:=200.0",
               "-p", "sonar.max_range:=10.0",
               "-p", f"sonar.min_range:={0.01 if args.informative else 1000.0}"]
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
                acquisition = seconds - args.scan_delay_seconds if seconds >= 2.0 else seconds
                points = ([(1.0 + x * 0.3, y * 0.3, -1.0)
                           for x in range(10) for y in range(-5, 5)]
                          if args.informative else
                          [(float(x), float(y), -1.0) for x in (1, 2) for y in (-1, 0, 1)])
                sonar.publish(create_cloud_xyz32(
                    Header(stamp=stamp(acquisition), frame_id="fixture_sonar"), points))

            def samples(begin, end, scan_times=()):
                next_sample = time.monotonic()
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
                    next_sample += 0.005
                    time.sleep(max(0.0, next_sample - time.monotonic()))

            def drain():
                end = time.monotonic() + 0.4
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)

            # Initialize through several real-time scans before testing fallback.
            samples(1.0, 3.0, tuple(1.0 + 0.2 * i for i in range(10)))
            drain()
            first_count = len(outputs)
            # Leave enough silence for the 1 Hz health observer to sample it,
            # even if an early recovery callback arrives before its IMU epoch.
            samples(3.005, 5.3)
            if args.early_recovery:
                cloud(5.55)
            samples(5.305, 5.8)
            drain()
            outage_count = len(outputs)
            if not args.early_recovery:
                cloud(5.55)
            samples(5.805, 6.2, (6.0,))
            drain()
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in outputs]
            print({"scan": first_count, "outage": outage_count,
                   "total": len(outputs), "stamps": stamps[-5:]}, flush=True)
            assert first_count > 0 and outage_count > first_count and len(outputs) > outage_count
            # Startup may publish the same epoch; initialized output must not go backwards.
            assert all(a <= b for a, b in zip(stamps, stamps[1:]))
            assert stamps[-1] >= 6100000000
            assert health, "Navigation health was not published"
            initialized_health = [message.status[0] for message in health
                                  if message.status and message.status[0].message != "initializing"]
            assert initialized_health
            assert all(status.message != "invalid estimator covariance" for status in initialized_health)
            if args.informative:
                assert any(dict((item.key, item.value) for item in status.values).get(
                    "sonar_condition") == "accepted_geometry_rank_unassessed"
                           for status in initialized_health), "No informative scan was accepted"
                # The explicitly late recovery message is intentionally stale.
                # Check normal delayed delivery before this recovery injection.
                during_scans = [message.status[0] for message in health[:2]
                                if message.status]
                assert all(dict((item.key, item.value) for item in status.values).get(
                    "sonar_late_dropped", "0") == "0" for status in during_scans)
            assert any(status.message == "degraded: no usable sonar or fused DVL"
                       for status in initialized_health), [
                {"level": status.level, "message": status.message,
                 "values": {item.key: item.value for item in status.values}}
                for status in initialized_health]
            report = {"command": command, "scan_phase_outputs": first_count,
                      "informative": args.informative,
                      "scan_delay_seconds": args.scan_delay_seconds,
                      "early_recovery": args.early_recovery,
                      "health_messages": len(health),
                      "health_conditions": sorted({dict((item.key, item.value) for item in status.values).get("sonar_condition", "")
                                                   for status in initialized_health}),
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
