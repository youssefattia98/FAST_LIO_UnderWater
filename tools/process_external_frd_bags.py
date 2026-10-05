#!/usr/bin/env python3
"""Convert external BlueROV FRD bags to the topics/frames consumed by UWFL2."""

from __future__ import annotations

import argparse
import shutil
import struct
import sys
from collections import Counter
from pathlib import Path
from typing import Callable

import rosbag2_py
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, MagneticField, PointCloud2, PointField


# MAVLink/ArduPilot body vectors are forward-right-down. ROS REP-103 body
# vectors are forward-left-up, a pi rotation about the common forward X axis.
FRD_TO_FLU_SIGNS = (1.0, -1.0, -1.0)
BODY_FRAME = "base_link"
DVL_FRAME = "dvl_link"
SONAR_FRAME = "sonar_link"

SOURCE_TOPICS = {
    "/bluerov2/imu": ("/auv/imu/data_raw", "sensor_msgs/msg/Imu"),
    "/bluerov2/magnetometer": (
        "/auv/imu/magnetic_field",
        "sensor_msgs/msg/MagneticField",
    ),
    "/sensor/DVLA50/velocity_twist": (
        "/auv/dvl",
        "geometry_msgs/msg/TwistWithCovarianceStamped",
    ),
    "/sonar3d/point_cloud": ("/sonar_point_cloud", "sensor_msgs/msg/PointCloud2"),
}


def transform_vector(vector, signs: tuple[float, float, float]) -> None:
    vector.x *= signs[0]
    vector.y *= signs[1]
    vector.z *= signs[2]


def convert_imu(message: Imu) -> Imu:
    message.header.frame_id = BODY_FRAME
    transform_vector(message.linear_acceleration, FRD_TO_FLU_SIGNS)
    transform_vector(message.angular_velocity, FRD_TO_FLU_SIGNS)

    # Match bridge.cpp: orientation and all IMU covariances are unknown. UWFL2
    # then uses the configured estimator covariances instead of invalid data.
    message.orientation.x = 0.0
    message.orientation.y = 0.0
    message.orientation.z = 0.0
    message.orientation.w = 1.0
    message.orientation_covariance = [-1.0] + [0.0] * 8
    message.angular_velocity_covariance = [-1.0] + [0.0] * 8
    message.linear_acceleration_covariance = [-1.0] + [0.0] * 8
    return message


def convert_magnetometer(message: MagneticField) -> MagneticField:
    message.header.frame_id = BODY_FRAME
    transform_vector(message.magnetic_field, FRD_TO_FLU_SIGNS)
    message.magnetic_field_covariance = [-1.0] + [0.0] * 8
    return message


def convert_dvl(message) -> TwistWithCovarianceStamped:
    converted = TwistWithCovarianceStamped()
    converted.header = message.header
    converted.header.frame_id = DVL_FRAME
    converted.twist.twist = message.twist
    transform_vector(converted.twist.twist.linear, FRD_TO_FLU_SIGNS)
    transform_vector(converted.twist.twist.angular, FRD_TO_FLU_SIGNS)
    # TwistStamped has no covariance. Zeros deliberately make UWFL2 use the
    # configured dvl.velocity_cov rather than inventing sensor uncertainty.
    converted.twist.covariance = [0.0] * 36
    return converted


def _xyz_fields(message: PointCloud2) -> dict[str, PointField]:
    fields = {field.name: field for field in message.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError(f"PointCloud2 has no {name!r} field")
        if fields[name].count != 1 or fields[name].datatype not in (
            PointField.FLOAT32,
            PointField.FLOAT64,
        ):
            raise ValueError(f"PointCloud2 {name!r} field is not a scalar float")
    return fields


def convert_sonar(message: PointCloud2) -> PointCloud2:
    fields = _xyz_fields(message)
    data = bytearray(message.data)
    endian = ">" if message.is_bigendian else "<"
    formats = {
        PointField.FLOAT32: endian + "f",
        PointField.FLOAT64: endian + "d",
    }

    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            point_offset = row_offset + column * message.point_step
            for axis, sign in (("y", -1.0), ("z", -1.0)):
                field = fields[axis]
                fmt = formats[field.datatype]
                offset = point_offset + field.offset
                value = struct.unpack_from(fmt, data, offset)[0]
                struct.pack_into(fmt, data, offset, sign * value)

    message.data = bytes(data)
    message.header.frame_id = SONAR_FRAME
    return message


CONVERTERS: dict[str, Callable] = {
    "/bluerov2/imu": convert_imu,
    "/bluerov2/magnetometer": convert_magnetometer,
    "/sensor/DVLA50/velocity_twist": convert_dvl,
    "/sonar3d/point_cloud": convert_sonar,
}


def topic_metadata(source, name: str, message_type: str):
    type_hash = source.type_description_hash if message_type == source.type else ""
    return rosbag2_py.TopicMetadata(
        id=0,
        name=name,
        type=message_type,
        serialization_format=source.serialization_format,
        offered_qos_profiles=source.offered_qos_profiles,
        type_description_hash=type_hash,
    )


def convert_bag(input_bag: Path, output_bag: Path, force: bool = False) -> Counter:
    if not (input_bag / "metadata.yaml").is_file():
        raise ValueError(f"Not a ROS 2 bag directory: {input_bag}")
    if output_bag.exists():
        if not force:
            raise FileExistsError(f"Output exists (use --force): {output_bag}")
        shutil.rmtree(output_bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    source_topics = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    active_sources = set(source_topics).intersection(SOURCE_TOPICS)
    if not active_sources:
        raise ValueError(f"No supported external-driver topics in {input_bag}")

    destination_names = {SOURCE_TOPICS[name][0] for name in active_sources}
    collisions = destination_names.intersection(set(source_topics) - active_sources)
    if collisions:
        raise ValueError(f"Destination topics already exist: {sorted(collisions)}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    for source in source_topics.values():
        if source.name in active_sources:
            destination, message_type = SOURCE_TOPICS[source.name]
            writer.create_topic(topic_metadata(source, destination, message_type))
        else:
            writer.create_topic(
                topic_metadata(source, source.name, source.type)
            )

    counts: Counter = Counter()
    message_classes = {
        name: get_message(source_topics[name].type) for name in active_sources
    }
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic not in active_sources:
            writer.write(topic, serialized, timestamp)
            counts["copied"] += 1
            continue

        message = deserialize_message(serialized, message_classes[topic])
        converted = CONVERTERS[topic](message)
        destination = SOURCE_TOPICS[topic][0]
        writer.write(destination, serialize_message(converted), timestamp)
        counts[destination] += 1

    writer.close()
    return counts


def discover_external_bags(root: Path) -> list[Path]:
    bags = []
    for candidate in sorted(root.iterdir()):
        if not candidate.is_dir() or not (candidate / "metadata.yaml").is_file():
            continue
        reader = rosbag2_py.Info()
        metadata = reader.read_metadata(str(candidate), "mcap")
        names = {entry.topic_metadata.name for entry in metadata.topics_with_message_count}
        if names.intersection(SOURCE_TOPICS):
            bags.append(candidate)
    return bags


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="*", type=Path, help="Input ROS 2 bag directories")
    parser.add_argument(
        "--all",
        action="store_true",
        help="Process all external-driver bags immediately below --input-root",
    )
    parser.add_argument(
        "--input-root",
        type=Path,
        default=Path("/home/attia/ros2_ws/bags2"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("/home/attia/ros2_ws/bags2/processed"),
    )
    parser.add_argument("--force", action="store_true", help="Replace existing outputs")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = list(args.bags)
    if args.all:
        inputs.extend(discover_external_bags(args.input_root))
    inputs = list(dict.fromkeys(path.resolve() for path in inputs))
    if not inputs:
        print("No bags selected. Pass bag paths or use --all.", file=sys.stderr)
        return 2

    args.output_root.mkdir(parents=True, exist_ok=True)
    failed = False
    for input_bag in inputs:
        output_bag = args.output_root / input_bag.name
        print(f"Converting {input_bag} -> {output_bag}")
        try:
            counts = convert_bag(input_bag, output_bag, args.force)
        except Exception as error:  # Keep batch processing useful after one bad bag.
            failed = True
            print(f"ERROR: {error}", file=sys.stderr)
            continue
        details = ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))
        print(f"Done: {details}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
