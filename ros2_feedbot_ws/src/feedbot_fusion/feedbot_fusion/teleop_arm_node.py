"""
Keyboard Teleop Node — manual joint control for the feeding robot arm.

Controls:
  W/S — Joint 1 (base yaw) +/-
  E/D — Joint 2 (shoulder pitch) +/-
  R/F — Joint 3 (elbow pitch) +/-
  T/G — Joint 4 (feeder tilt) +/-
  H   — Home position (0, 0, 0, 0)
  +/- — Increase/decrease step size
  Q   — Quit

Publishes:
  /arm_controller/follow_joint_trajectory (FollowJointTrajectory action)

Usage:
  ros2 run feedbot_fusion teleop_arm
"""

import sys
import termios
import tty
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['base_y_joint', 'lower_z_joint', 'upper_z_joint', 'feeder_joint']

JOINT_LIMITS = {
    'base_y_joint':  (-3.05, 3.05),
    'lower_z_joint': (-1.57, 1.57),
    'upper_z_joint': (-2.09, 1.31),
    'feeder_joint':  (-1.57, 2.01),
}

KEY_MAP = {
    'w': (0, +1), 's': (0, -1),   # Joint 1
    'e': (1, +1), 'd': (1, -1),   # Joint 2
    'r': (2, +1), 'f': (2, -1),   # Joint 3
    't': (3, +1), 'g': (3, -1),   # Joint 4
}

HELP_TEXT = """
─────────────────────────────────────
  Feeding Robot Arm Teleop
─────────────────────────────────────
  W/S  —  Joint 1 (base yaw)    +/-
  E/D  —  Joint 2 (shoulder)    +/-
  R/F  —  Joint 3 (elbow)       +/-
  T/G  —  Joint 4 (feeder tilt) +/-
  H    —  Home position (0,0,0,0)
  +/-  —  Step size up/down
  [/]  —  Speed slower/faster
  Q    —  Quit
─────────────────────────────────────
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopArmNode(Node):

    def __init__(self):
        super().__init__('teleop_arm_node')

        self.step_size = 0.05  # radians per keypress
        self.trajectory_duration = 1.0  # seconds per move (smooth transition)

        self.current_positions = [0.0] * 4
        self.target_positions = [0.0] * 4

        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)

        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for arm_controller action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Connected to arm_controller')

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                idx = JOINT_NAMES.index(name)
                self.current_positions[idx] = pos

    def send_goal(self, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        # Start point at current position with zero velocity
        start = JointTrajectoryPoint()
        start.positions = [float(p) for p in self.current_positions]
        start.velocities = [0.0] * 4
        start.time_from_start = Duration(sec=0, nanosec=0)

        # End point with zero velocity (smooth stop)
        end = JointTrajectoryPoint()
        end.positions = [float(p) for p in positions]
        end.velocities = [0.0] * 4
        end.time_from_start = Duration(
            sec=int(self.trajectory_duration),
            nanosec=int((self.trajectory_duration % 1) * 1e9))

        goal.trajectory.points = [start, end]
        self.action_client.send_goal_async(goal)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        self.target_positions = list(self.current_positions)

        print(HELP_TEXT)
        print(f'  Step size: {math.degrees(self.step_size):.1f}°'
              f' ({self.step_size:.3f} rad)')
        self._print_positions()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = get_key(settings)

                if key == 'q' or key == '\x03':  # q or Ctrl+C
                    break

                elif key == 'h':
                    self.target_positions = [0.0, 0.0, 0.0, 0.0]
                    self.send_goal(self.target_positions)
                    print('\n  >> HOME (0, 0, 0, 0)')
                    self._print_positions()

                elif key == '+' or key == '=':
                    self.step_size = min(self.step_size + 0.01, 0.5)
                    print(f'\n  Step size: {math.degrees(self.step_size):.1f}°')

                elif key == '-':
                    self.step_size = max(self.step_size - 0.01, 0.01)
                    print(f'\n  Step size: {math.degrees(self.step_size):.1f}°')

                elif key == '[':
                    self.trajectory_duration = min(self.trajectory_duration + 0.25, 5.0)
                    print(f'\n  Speed: {self.trajectory_duration:.2f}s per move (slower)')

                elif key == ']':
                    self.trajectory_duration = max(self.trajectory_duration - 0.25, 0.25)
                    print(f'\n  Speed: {self.trajectory_duration:.2f}s per move (faster)')

                elif key.lower() in KEY_MAP:
                    joint_idx, direction = KEY_MAP[key.lower()]
                    name = JOINT_NAMES[joint_idx]
                    lo, hi = JOINT_LIMITS[name]

                    new_pos = self.target_positions[joint_idx] + direction * self.step_size
                    new_pos = max(lo, min(hi, new_pos))
                    self.target_positions[joint_idx] = new_pos

                    self.send_goal(self.target_positions)
                    self._print_positions()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _print_positions(self):
        labels = ['Base', 'Shoulder', 'Elbow', 'Feeder']
        parts = [f'{l}: {math.degrees(p):+6.1f}°'
                 for l, p in zip(labels, self.target_positions)]
        print(f'  {" | ".join(parts)}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArmNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
