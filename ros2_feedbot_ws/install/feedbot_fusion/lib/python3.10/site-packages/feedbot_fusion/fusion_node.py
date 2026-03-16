import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState


class FusionNode(Node):

    def __init__(self):

        super().__init__('fusion_node')

        self.food_visible = False
        self.force = 0.0

        # Subscribers
        self.create_subscription(
            Bool,
            '/food_visible',
            self.food_callback,
            10)

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        self.create_subscription(
            Float64,
            '/spoon_force',
            self.force_callback,
            10)

        # Publisher
        self.error_pub = self.create_publisher(
            Float64,
            '/food_error_x',
            10)

        # Run fusion at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_error)

        self.get_logger().info("Fusion node started")

    def food_callback(self, msg):
        self.food_visible = msg.data

    def joint_callback(self, msg):
        pass  # not needed yet

    def force_callback(self, msg):
        self.force = msg.data

    def publish_error(self):

        msg = Float64()

        if self.food_visible:
            msg.data = 0.1
        else:
            msg.data = 0.0

        self.error_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = FusionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
