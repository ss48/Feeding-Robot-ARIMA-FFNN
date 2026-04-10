"""
Benchmark Node — Records arm tracking performance for PID vs PID+ARIMA comparison.

Subscribes to joint states, trajectory goals, and ARIMA predictions.
Computes and logs: RMSE, overshoot, settling time, smoothness (jerk),
prediction error, and cycle time.

Saves timestamped CSV to ~/feedbot_benchmark/ for offline analysis.

Usage:
  ros2 run feedbot_fusion benchmark_node --ros-args -p mode:=pid_only
  ros2 run feedbot_fusion benchmark_node --ros-args -p mode:=pid_arima

Topics subscribed:
  /joint_states           (JointState)
  /predicted_state        (Float64MultiArray) — ARIMA predictions
  /feeding_state          (String) — FSM state for cycle timing

Topics published:
  /benchmark/metrics      (String) — JSON metrics summary
"""

import os
import time
import math
import json
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

JOINT_NAMES = ['base_y_joint', 'lower_z_joint', 'upper_z_joint', 'feeder_joint']
SETTLE_THRESHOLD = 0.02  # rad — within this = "settled"
LOG_DIR = os.path.expanduser('~/feedbot_benchmark')


class BenchmarkNode(Node):

    def __init__(self):
        super().__init__('benchmark_node')

        self.declare_parameter('mode', 'pid_only')  # 'pid_only' or 'pid_arima'

        self.mode = self.get_parameter('mode').value

        os.makedirs(LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(LOG_DIR, f'{self.mode}_{timestamp}.csv')

        self.csv_file = open(self.csv_path, 'w')
        self.csv_file.write(
            'time,'
            'j1_target,j1_actual,j1_error,j1_predicted,j1_pred_error,'
            'j2_target,j2_actual,j2_error,j2_predicted,j2_pred_error,'
            'j3_target,j3_actual,j3_error,j3_predicted,j3_pred_error,'
            'j4_target,j4_actual,j4_error,j4_predicted,j4_pred_error,'
            'fsm_state\n')

        # State
        self.actual = [0.0] * 4
        self.target = [0.0] * 4
        self.predicted = [0.0] * 4
        self.fsm_state = 'WAITING'
        self.start_time = time.monotonic()

        # Metrics accumulators
        self.errors = [deque(maxlen=5000) for _ in range(4)]
        self.positions = [deque(maxlen=5000) for _ in range(4)]
        self.velocities = [deque(maxlen=5000) for _ in range(4)]
        self.overshoots = [0.0] * 4
        self.settle_times = [0.0] * 4
        self._target_changed_time = [time.monotonic()] * 4
        self._settled = [False] * 4
        self._prev_vel = [0.0] * 4
        self._jerk_sum = [0.0] * 4
        self._jerk_count = [0] * 4

        # Cycle tracking
        self._cycle_start = None
        self._cycle_times = []

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(Float64MultiArray, '/predicted_state', self._pred_cb, 10)
        self.create_subscription(String, '/feeding_state', self._fsm_cb, 10)

        # Metrics publisher
        self.metrics_pub = self.create_publisher(String, '/benchmark/metrics', 10)

        # Record at 50 Hz
        self.create_timer(0.02, self._record)

        # Publish summary at 1 Hz
        self.create_timer(1.0, self._publish_metrics)

        self.get_logger().info(
            f'Benchmark started — mode: {self.mode}, logging to {self.csv_path}')

    def _joint_cb(self, msg):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                new_pos = msg.position[idx]
                old_pos = self.actual[i]
                self.actual[i] = new_pos

                # Track velocity for jerk calculation
                vel = (new_pos - old_pos) * 50.0  # approx 50Hz
                self.velocities[i].append(vel)

                # Jerk = derivative of acceleration
                if len(self.velocities[i]) >= 3:
                    v = list(self.velocities[i])
                    acc_now = (v[-1] - v[-2]) * 50.0
                    acc_prev = (v[-2] - v[-3]) * 50.0
                    jerk = abs(acc_now - acc_prev) * 50.0
                    self._jerk_sum[i] += jerk
                    self._jerk_count[i] += 1

    def _pred_cb(self, msg):
        if len(msg.data) >= 4:
            self.predicted = list(msg.data[:4])

    def _fsm_cb(self, msg):
        new_state = msg.data
        if new_state != self.fsm_state:
            # Cycle timing
            if new_state == 'IDLE' and self.fsm_state == 'WAITING':
                self._cycle_start = time.monotonic()
            elif new_state == 'WAITING' and self._cycle_start is not None:
                cycle_time = time.monotonic() - self._cycle_start
                self._cycle_times.append(cycle_time)
                self.get_logger().info(f'Cycle completed in {cycle_time:.1f}s')
                self._cycle_start = None
        self.fsm_state = new_state

    def _record(self):
        t = time.monotonic() - self.start_time

        row = [f'{t:.3f}']
        for i in range(4):
            target = self.target[i]
            actual = self.actual[i]
            predicted = self.predicted[i]
            error = actual - target
            pred_error = predicted - actual

            self.errors[i].append(error)
            self.positions[i].append(actual)

            # Track overshoot
            if abs(error) > abs(self.overshoots[i]):
                self.overshoots[i] = error

            # Track settling time
            if not self._settled[i]:
                if abs(error) < SETTLE_THRESHOLD:
                    self._settled[i] = True
                    self.settle_times[i] = t - self._target_changed_time[i]

            row.extend([
                f'{target:.4f}', f'{actual:.4f}', f'{error:.4f}',
                f'{predicted:.4f}', f'{pred_error:.4f}'])

        row.append(self.fsm_state)
        self.csv_file.write(','.join(row) + '\n')

    def _publish_metrics(self):
        metrics = {'mode': self.mode}

        for i, name in enumerate(JOINT_NAMES):
            errs = list(self.errors[i])
            if errs:
                rmse = math.sqrt(sum(e ** 2 for e in errs) / len(errs))
            else:
                rmse = 0.0

            avg_jerk = (self._jerk_sum[i] / max(self._jerk_count[i], 1))

            metrics[name] = {
                'rmse': round(rmse, 5),
                'overshoot': round(abs(self.overshoots[i]), 5),
                'settle_time': round(self.settle_times[i], 3),
                'avg_jerk': round(avg_jerk, 2),
            }

        if self._cycle_times:
            metrics['avg_cycle_time'] = round(
                sum(self._cycle_times) / len(self._cycle_times), 1)
            metrics['cycles'] = len(self._cycle_times)

        msg = String()
        msg.data = json.dumps(metrics, indent=2)
        self.metrics_pub.publish(msg)

    def set_target(self, positions):
        """Called when a new trajectory goal is detected."""
        now = time.monotonic()
        for i in range(min(len(positions), 4)):
            if abs(positions[i] - self.target[i]) > 0.01:
                self.target[i] = positions[i]
                self.overshoots[i] = 0.0
                self._settled[i] = False
                self._target_changed_time[i] = now

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Benchmark data saved to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
