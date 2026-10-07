#!/usr/bin/env python3
"""Copy a ROS 2 bag while releasing the sonar frame for UWFL2 ownership."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def open_reader(path: Path, storage_id: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def topic_metadata(source) -> rosbag2_py.TopicMetadata:
    return rosbag2_py.TopicMetadata(
        id=0,
        name=source.name,
        type=source.type,
        serialization_format=source.serialization_format,
        offered_qos_profiles=source.offered_qos_profiles,
        type_description_hash=source.type_description_hash,
    )


def find_cloud_frame(
    bag: Path, storage_id: str, sonar_topic: str
) -> str:
    reader = open_reader(bag, storage_id)
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == sonar_topic:
            frame_id = deserialize_message(serialized, PointCloud2).header.frame_id
            if not frame_id:
                raise ValueError(f"{sonar_topic} has an empty frame_id")
            return frame_id
    raise ValueError(f"Bag has no messages on {sonar_topic}")


def repair_bag(
    input_bag: Path,
    output_bag: Path,
    sonar_topic: str,
    body_frame: str,
    force: bool,
) -> tuple[str, int, int]:
    if not (input_bag / "metadata.yaml").is_file():
        raise ValueError(f"Not a ROS 2 bag directory: {input_bag}")
    if output_bag.resolve() == input_bag.resolve():
        raise ValueError("Input and output bags must differ")
    if output_bag.exists():
        if not force:
            raise FileExistsError(f"Output exists (use --force): {output_bag}")
        shutil.rmtree(output_bag)

    metadata = rosbag2_py.Info().read_metadata(str(input_bag), "")
    storage_id = metadata.storage_identifier
    sonar_frame = find_cloud_frame(input_bag, storage_id, sonar_topic)

    reader = open_reader(input_bag, storage_id)
    source_topics = reader.get_all_topics_and_types()
    source_names = {topic.name for topic in source_topics}
    if sonar_topic not in source_names:
        raise ValueError(f"Bag has no {sonar_topic} topic")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in source_topics:
        writer.create_topic(topic_metadata(topic))

    removed = 0
    copied = 0
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic in ("/tf", "/tf_static"):
            message = deserialize_message(serialized, TFMessage)
            retained = []
            for transform in message.transforms:
                if (
                    transform.child_frame_id == sonar_frame
                    and transform.header.frame_id != body_frame
                ):
                    removed += 1
                else:
                    retained.append(transform)
            if len(retained) != len(message.transforms):
                if not retained:
                    continue
                message.transforms = retained
                serialized = serialize_message(message)

        writer.write(topic, serialized, timestamp)
        copied += 1

    writer.close()
    return sonar_frame, removed, copied


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--sonar-topic", default="/sonar_point_cloud")
    parser.add_argument("--body-frame", default="body")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    frame, removed, copied = repair_bag(
        args.input_bag.resolve(),
        args.output_bag.resolve(),
        args.sonar_topic,
        args.body_frame,
        args.force,
    )
    print(f"Sonar frame: {frame}")
    print(f"Removed conflicting TF transforms: {removed}")
    print(f"Copied messages: {copied}")
    print(f"Output: {args.output_bag.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
