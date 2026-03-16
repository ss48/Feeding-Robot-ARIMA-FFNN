import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState


class SpeedController(Node):

    def __init__(self):
        super().__init__('speed_controller')

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            '/joint3_controller/reference',
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.current_pos = 0.0
        self.target = 1.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Speed controller started")

    def joint_callback(self, msg):

        for name, pos in zip(msg.name, msg.position):
            if name == "joint3":
                self.current_pos = pos

    def control_loop(self):

        error = self.target - self.current_pos

        step = 0.02

        if abs(error) < step:
            command = self.target
        elif error > 0:
            command = self.current_pos + step
        else:
            command = self.current_pos - step

        msg = MultiDOFCommand()
        msg.dof_names = ['joint3']
        msg.values = [command]
        msg.values_dot = []

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SpeedController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
