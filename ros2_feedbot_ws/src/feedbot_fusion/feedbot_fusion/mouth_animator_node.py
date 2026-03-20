"""
Mouth Animator Node — cycles the patient head's jaw joint in Gazebo.

Publishes a sinusoidal position command to /patient_head/jaw_cmd so the
jaw opens and closes periodically.  Also publishes /mouth_open (Bool)
and /mouth_ready_prediction (Bool) for the fusion and FSM nodes.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool


class MouthAnimatorNode(Node):

    def __init__(self):
        super().__init__('mouth_animator_node')

        # Parameters
        self.declare_parameter('period', 4.0)          # seconds per cycle
        self.declare_parameter('max_opening', -0.4)     # radians (negative = open)
        self.declare_parameter('open_threshold', 0.5)   # openness ratio to count as "open"

        self.period = self.get_parameter('period').value
        self.max_opening = self.get_parameter('max_opening').value
        self.open_threshold = self.get_parameter('open_threshold').value

        # Publisher for Gazebo JointPositionController
        self.jaw_pub = self.create_publisher(Float64, '/patient_head/jaw_cmd', 10)

        # Publishers for ROS feeding system
        self.mouth_open_pub = self.create_publisher(Bool, '/mouth_open', 10)
        # Note: /mouth_ready_prediction is published by arima_ffnn_node.
        # This node publishes the ground-truth jaw state on /mouth_open only.

        # 20 Hz update
        self.timer = self.create_timer(0.05, self.tick)
        self.t = 0.0

        self.get_logger().info(
            f'Mouth animator started (period={self.period}s, '
            f'max_opening={self.max_opening} rad)')

    def tick(self):
        self.t += 0.05

        # Sinusoidal openness: 0 = closed, 1 = fully open
        openness = 0.5 + 0.5 * math.sin(2.0 * math.pi * self.t / self.period)

        # Jaw angle: 0 = closed, max_opening = fully open
        jaw_angle = self.max_opening * openness

        # Publish to Gazebo
        jaw_msg = Float64()
        jaw_msg.data = jaw_angle
        self.jaw_pub.publish(jaw_msg)

        # Publish mouth state
        is_open = openness > self.open_threshold
        open_msg = Bool()
        open_msg.data = is_open
        self.mouth_open_pub.publish(open_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MouthAnimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
