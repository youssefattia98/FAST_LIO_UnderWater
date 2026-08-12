#!/usr/bin/env python3
"""Create a full-SE(3) manual loop request from simulation ground truth."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--keyframes", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--from-id", type=int)
    parser.add_argument("--to-id", type=int)
    parser.add_argument("--select-return-pair", action="store_true")
    parser.add_argument("--minimum-keyframe-separation", type=int, default=20)
    parser.add_argument("--maximum-return-distance", type=float, default=5.0)
    parser.add_argument("--ground-truth-parent", default="World")
    parser.add_argument("--ground-truth-child", default="BROV_low")
    parser.add_argument("--vehicle-frame", default="imu_link")
    parser.add_argument("--translation-sigma", type=float, default=0.05)
    parser.add_argument("--rotation-sigma-deg", type=float, default=1.0)
    return parser.parse_args()


def quaternion_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    quaternion = np.asarray([w, x, y, z], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion in ground-truth transform")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform(message) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = quaternion_rotation(
        message.rotation.x,
        message.rotation.y,
        message.rotation.z,
        message.rotation.w,
    )
    result[:3, 3] = (
        message.translation.x,
        message.translation.y,
        message.translation.z,
    )
    return result


def rotation_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def storage_id(bag: Path) -> str:
    document = yaml.safe_load((bag / "metadata.yaml").read_text())
    return str(document["rosbag2_bagfile_information"]["storage_identifier"])


def read_ground_truth(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=storage_id(args.bag)),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/tf_static"]))
    samples: list[tuple[float, np.ndarray]] = []
    T_ground_truth_vehicle: np.ndarray | None = (
        np.eye(4) if args.ground_truth_child == args.vehicle_frame else None
    )
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, TFMessage)
        for stamped in message.transforms:
            if (
                topic == "/tf_static"
                and stamped.header.frame_id == args.ground_truth_child
                and stamped.child_frame_id == args.vehicle_frame
            ):
                T_ground_truth_vehicle = transform(stamped.transform)
            if (
                topic == "/tf"
                and stamped.header.frame_id == args.ground_truth_parent
                and stamped.child_frame_id == args.ground_truth_child
            ):
                samples.append((stamp_seconds(stamped.header.stamp), transform(stamped.transform)))
    if T_ground_truth_vehicle is None:
        raise RuntimeError(
            f"missing fixed {args.ground_truth_child}->{args.vehicle_frame} transform"
        )
    if len(samples) < 2:
        raise RuntimeError("ground-truth trajectory was not found in the input bag")
    samples.sort(key=lambda item: item[0])
    times = np.asarray([item[0] for item in samples])
    poses = np.asarray([item[1] @ T_ground_truth_vehicle for item in samples])
    return times, poses


def read_keyframes(path: Path) -> list[dict[str, float | int]]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("keyframe CSV is empty")
    return [
        {
            "id": int(row["id"]),
            "timestamp": float(row["timestamp"]),
            "tx": float(row["tx"]),
            "ty": float(row["ty"]),
            "tz": float(row["tz"]),
        }
        for row in rows
    ]


def nearest_poses(
    source_times: np.ndarray, source_poses: np.ndarray, target_times: np.ndarray
) -> np.ndarray:
    right = np.clip(np.searchsorted(source_times, target_times), 0, len(source_times) - 1)
    left = np.clip(right - 1, 0, len(source_times) - 1)
    indices = np.where(
        np.abs(target_times - source_times[left])
        <= np.abs(target_times - source_times[right]),
        left,
        right,
    )
    return source_poses[indices]


def select_pair(
    rows: list[dict[str, float | int]],
    poses: np.ndarray,
    minimum_separation: int,
    maximum_distance: float,
) -> tuple[int, int]:
    best: tuple[float, int, int] | None = None
    for first in range(len(rows)):
        for second in range(first + minimum_separation, len(rows)):
            distance = float(
                np.linalg.norm(poses[first, :3, 3] - poses[second, :3, 3])
            )
            if distance > maximum_distance:
                continue
            elapsed = float(rows[second]["timestamp"]) - float(rows[first]["timestamp"])
            score = elapsed - 2.0 * distance
            if best is None or score > best[0]:
                best = (score, first, second)
    if best is None:
        raise RuntimeError(
            "no ground-truth return pair satisfies the separation and distance limits"
        )
    return best[1], best[2]


def main() -> int:
    args = parse_args()
    rows = read_keyframes(args.keyframes)
    truth_times, truth_poses = read_ground_truth(args)
    keyframe_times = np.asarray([float(row["timestamp"]) for row in rows])
    poses = nearest_poses(truth_times, truth_poses, keyframe_times)

    if args.from_id is not None or args.to_id is not None:
        if args.from_id is None or args.to_id is None:
            raise ValueError("--from-id and --to-id must be supplied together")
        indices = {int(row["id"]): index for index, row in enumerate(rows)}
        first, second = indices[args.from_id], indices[args.to_id]
    elif args.select_return_pair:
        first, second = select_pair(
            rows,
            poses,
            args.minimum_keyframe_separation,
            args.maximum_return_distance,
        )
    else:
        raise ValueError("select a pair explicitly or pass --select-return-pair")

    if first >= second:
        raise ValueError("manual loop IDs must be chronological")
    relative = np.linalg.inv(poses[first]) @ poses[second]
    qx, qy, qz, qw = rotation_quaternion(relative[:3, :3])
    covariance = np.zeros((6, 6))
    covariance[:3, :3] = np.eye(3) * math.radians(args.rotation_sigma_deg) ** 2
    covariance[3:, 3:] = np.eye(3) * args.translation_sigma**2
    request = {
        "from_id": int(rows[first]["id"]),
        "to_id": int(rows[second]["id"]),
        "relative_pose": {
            "position": {
                "x": float(relative[0, 3]),
                "y": float(relative[1, 3]),
                "z": float(relative[2, 3]),
            },
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        },
        "covariance": [float(value) for value in covariance.reshape(-1)],
        "test_override": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(request, sort_keys=False))
    distance = float(np.linalg.norm(relative[:3, 3]))
    print(
        f"Selected keyframes {request['from_id']}->{request['to_id']} "
        f"({keyframe_times[second] - keyframe_times[first]:.1f} s apart, "
        f"GT displacement {distance:.3f} m)"
    )
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
