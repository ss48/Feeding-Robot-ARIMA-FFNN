import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class AutoPIDTuner(Node):

    def __init__(self):
        super().__init__('auto_pid_tuner')

        self.joint_name = "joint1"
        self.step_target = 0.5
        self.duration = 4.0

        self.actual_position = 0.0
        self.time_data = []
        self.error_data = []

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint1_controller/reference',
            10
        )

    def joint_callback(self, msg):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.actual_position = msg.position[idx]

    def send_step(self):
        msg = Float64MultiArray()
        msg.data = [self.step_target]
        self.pub.publish(msg)

    def run_test(self):
        self.time_data = []
        self.error_data = []

        start = time.time()

        while time.time() - start < self.duration:
            t = time.time() - start
            error = abs(self.step_target - self.actual_position)

            self.time_data.append(t)
            self.error_data.append(error)

            self.send_step()
            rclpy.spin_once(self, timeout_sec=0.01)

        return self.compute_itae()

    def compute_itae(self):
        t = np.array(self.time_data)
        e = np.array(self.error_data)
        return np.sum(t * e)

def main():
    rclpy.init()
    tuner = AutoPIDTuner()

    best = [80.0, 0.5, 10.0]
    dp = [20.0, 0.2, 3.0]

    best_score = tuner.run_test()

    for _ in range(10):

        for i in range(3):

            trial = best.copy()
            trial[i] += dp[i]

            tuner.get_logger().info(f"Testing PID {trial}")

            score = tuner.run_test()

            if score < best_score:
                best_score = score
                best = trial
                dp[i] *= 1.2
            else:
                trial[i] -= 2 * dp[i]
                score = tuner.run_test()

                if score < best_score:
                    best_score = score
                    best = trial
                    dp[i] *= 1.1
                else:
                    dp[i] *= 0.5

    tuner.get_logger().info(f"Best PID found: {best}")

    tuner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
