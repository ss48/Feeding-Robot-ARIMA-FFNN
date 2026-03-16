import pandas as pd
import matplotlib.pyplot as plt

plt.style.use('ggplot')

joints = ['joint1', 'joint2', 'joint3', 'joint4']

for joint in joints:
    file_name = f'bag_pid_{joint}.csv'
    df = pd.read_csv(file_name)

    df_joint = df[df['joint'] == joint].copy()

    # Normalize time to start at 0
    df_joint['time'] -= df_joint['time'].iloc[0]

    plt.figure(figsize=(8,5))
    plt.plot(df_joint['time'], df_joint['position'], label='PID Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title(f'{joint} PID Step Response')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f'{joint}_pid_response.png', dpi=300)
    plt.show()

print("All PID graphs generated.")
