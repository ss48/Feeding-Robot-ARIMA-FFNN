#!/usr/bin/env python3
import csv
import sys

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message


def main():
    if len(sys.argv) < 3:
        print("Usage: export_joint_states_csv.py <bag_path.db3> <out.csv>")
        sys.exit(1)

    bag_path = sys.argv[1]
    out_csv = sys.argv[2]

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topic = "/joint_states"
    if topic not in topic_types:
        raise RuntimeError(f"{topic} not found in bag. Found: {list(topic_types.keys())}")

    msg_type = get_message(topic_types[topic])

    # We’ll write wide CSV: one row per message, with joint names as columns.
    # First pass: discover joint names order from first message.
    first = True
    joint_names = []

    with open(out_csv, "w", newline="") as f:
        writer = None

        while reader.has_next():
            tname, data, t = reader.read_next()
            if tname != topic:
                continue

            msg = deserialize_message(data, msg_type)

            if first:
                joint_names = list(msg.name)
                header = ["time_ns"]
                header += [f"pos_{n}" for n in joint_names]
                header += [f"vel_{n}" for n in joint_names]
                header += [f"eff_{n}" for n in joint_names]
                writer = csv.writer(f)
                writer.writerow(header)
                first = False

            # Some fields might be empty; pad safely
            pos = list(msg.position) if msg.position else [float("nan")] * len(joint_names)
            vel = list(msg.velocity) if msg.velocity else [float("nan")] * len(joint_names)
            eff = list(msg.effort) if msg.effort else [float("nan")] * len(joint_names)

            # Ensure lengths match
            def fit(arr):
                if len(arr) < len(joint_names):
                    return arr + [float("nan")] * (len(joint_names) - len(arr))
                return arr[: len(joint_names)]

            row = [t] + fit(pos) + fit(vel) + fit(eff)
            writer.writerow(row)

    print(f"Saved: {out_csv}")


if __name__ == "__main__":
    rclpy.init()
    main()
    rclpy.shutdown()
