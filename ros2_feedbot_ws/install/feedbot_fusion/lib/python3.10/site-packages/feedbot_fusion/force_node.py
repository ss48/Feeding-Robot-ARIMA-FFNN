import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import numpy as np


class ForceNode(Node):

    def __init__(self):
        super().__init__('force_node')

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/spoon/wrench',
            self.force_callback,
            10)

        self.publisher = self.create_publisher(
            Float64,
            '/spoon_force',
            10)

        self.force_buffer = []

        self.get_logger().info("Force node started")

    def force_callback(self, msg):

        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        force_norm = np.sqrt(fx**2 + fy**2 + fz**2)

        self.force_buffer.append(force_norm)

        if len(self.force_buffer) > 10:
            self.force_buffer.pop(0)

        filtered_force = np.mean(self.force_buffer)

        force_msg = Float64()
        force_msg.data = filtered_force

        self.publisher.publish(force_msg)

        self.get_logger().info(f"Filtered force: {filtered_force:.3f}")


def main(args=None):

    rclpy.init(args=args)

    node = ForceNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
