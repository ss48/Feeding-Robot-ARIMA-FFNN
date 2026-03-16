#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message

STATE_TOPIC = "/joint_states"
REF_TOPIC = "/joint1_controller/reference"


def main():

    if len(sys.argv) < 2:
        print("Usage: python3 plot_bag.py <path_to_db3>")
        return

    db3_path = sys.argv[1]

    reader = SequentialReader()
    storage_options = StorageOptions(uri=db3_path, storage_id="sqlite3")
    converter_options = ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(typ) for name, typ in topic_types.items()}

    t_state = []
    pos = []

    t_ref = []
    ref = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])

        if topic == STATE_TOPIC:
            if len(msg.position) > 0:
                t_state.append(t)
                pos.append(msg.position[0])

        if topic == REF_TOPIC:
            if len(msg.values) > 0:
                t_ref.append(t)
                ref.append(msg.values[0])

    if not t_state:
        print("No joint_states found.")
        return

    t0 = t_state[0]
    t_state = (np.array(t_state) - t0) / 1e9
    pos = np.array(pos)

    if t_ref:
        t_ref = (np.array(t_ref) - t0) / 1e9
        ref = np.array(ref)
    else:
        print("No reference topic found.")
        ref = None

    plt.figure()
    plt.plot(t_state, pos, label="Actual Position")

    if ref is not None:
        plt.step(t_ref, ref, where="post", label="Reference")

    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.title("Joint1 Position vs Reference")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
