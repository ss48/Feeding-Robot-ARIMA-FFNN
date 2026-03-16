from rosbags.rosbag2 import Reader
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.serde import deserialize_cdr
import pandas as pd

def bag_to_csv(bag_path, output_name):
    data = []

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/joint_states':
                msg = deserialize_cdr(rawdata, connection.msgtype)

                for i, name in enumerate(msg.name):
                    data.append({
                        'time': timestamp * 1e-9,
                        'joint': name,
                        'position': msg.position[i],
                        'velocity': msg.velocity[i],
                        'effort': msg.effort[i]
                    })

    df = pd.DataFrame(data)
    df.to_csv(output_name, index=False)
    print(f"Saved {output_name}")

bags = [
    "bag_pid_joint1",
    "bag_pid_joint2",
    "bag_pid_joint3",
    "bag_pid_joint4"
]

for bag in bags:
    bag_to_csv(bag, bag + ".csv")

