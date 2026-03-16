import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import rclpy.serialization
from control_msgs.msg import MultiDOFStateStamped

def read_bag(db_path, topic_name):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("""
        SELECT timestamp, data FROM messages
        WHERE topic_id = (
            SELECT id FROM topics WHERE name=?
        )
    """, (topic_name,))

    rows = cursor.fetchall()
    conn.close()
    return rows

def extract_joint_data(rows):
    times = []
    ref = []
    pos = []

    for t, raw in rows:
        msg = rclpy.serialization.deserialize_message(raw, MultiDOFStateStamped)

        # Access dof_states (list)
        state = msg.dof_states[0]

        times.append(t * 1e-9)
        ref.append(state.reference)
        pos.append(state.feedback)

    times = np.array(times)
    times = times - times[0]

    return times, np.array(ref), np.array(pos)

def process_bag(folder, joint):
    db_file = [f for f in os.listdir(folder) if f.endswith(".db3")][0]
    db_path = os.path.join(folder, db_file)

    topic = f"/joint{joint}_controller/controller_state"
    rows = read_bag(db_path, topic)

    return extract_joint_data(rows)

untuned_folder = "bags/untuned_all"
tuned_folder   = "bags/tuned_all"

for joint in range(1, 5):

    t_u, r_u, p_u = process_bag(untuned_folder, joint)
    t_t, r_t, p_t = process_bag(tuned_folder, joint)

    plt.figure(figsize=(10,6))

    plt.plot(t_u, r_u, '--', linewidth=2, label="Desired (Untuned)")
    plt.plot(t_u, p_u, linewidth=2, label="Actual (Untuned)")

    plt.plot(t_t, r_t, '--', linewidth=2, label="Desired (Tuned)")
    plt.plot(t_t, p_t, linewidth=2, label="Actual (Tuned)")

    plt.title(f"PID Tuning Comparison - Joint {joint}", fontsize=14)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f"joint{joint}_comparison.png")
    plt.close()   # IMPORTANT: prevents blocking

