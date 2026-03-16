import sqlite3
import pandas as pd
import os

def extract_joint_states(bag_folder, output_csv):
    db_file = None
    
    # find db3 file
    for file in os.listdir(bag_folder):
        if file.endswith(".db3"):
            db_file = os.path.join(bag_folder, file)
            break

    if db_file is None:
        print(f"No db3 file found in {bag_folder}")
        return

    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    cursor.execute("SELECT timestamp, data FROM messages")
    rows = cursor.fetchall()

    data_list = []

    for row in rows:
        timestamp = row[0]
        # we only save timestamp now (raw data is binary)
        data_list.append({
            "time": timestamp * 1e-9
        })

    df = pd.DataFrame(data_list)
    df.to_csv(output_csv, index=False)
    print(f"Saved {output_csv}")

bags = [
    "bag_pid_joint1",
    "bag_pid_joint2",
    "bag_pid_joint3",
    "bag_pid_joint4"
]

for bag in bags:
    extract_joint_states(bag, bag + "_time.csv")
