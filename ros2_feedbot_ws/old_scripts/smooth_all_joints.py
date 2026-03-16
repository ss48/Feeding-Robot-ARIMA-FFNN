import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
import math
import time

class SmoothAllJoints(Node):

    def __init__(self):
        super().__init__('smooth_all_joints')

        self.pub1 = self.create_publisher(MultiDOFCommand, '/joint1_controller/reference', 10)
        self.pub2 = self.create_publisher(MultiDOFCommand, '/joint2_controller/reference', 10)
        self.pub3 = self.create_publisher(MultiDOFCommand, '/joint3_controller/reference', 10)
        self.pub4 = self.create_publisher(MultiDOFCommand, '/joint4_controller/reference', 10)

        self.start_time = time.time()

        self.timer = self.create_timer(0.01, self.update)  # 100 Hz

    def update(self):
        t = time.time() - self.start_time

        # Smooth sinusoidal motion
        j1 = 0.4 * math.sin(0.5 * t)
        j2 = 0.3 * math.sin(0.6 * t)
        j3 = 0.25 * math.sin(0.7 * t)
        j4 = 0.2 * math.sin(0.8 * t)

        self.publish_joint(self.pub1, 'joint1', j1)
        self.publish_joint(self.pub2, 'joint2', j2)
        self.publish_joint(self.pub3, 'joint3', j3)
        self.publish_joint(self.pub4, 'joint4', j4)

    def publish_joint(self, publisher, name, value):
        msg = MultiDOFCommand()
        msg.dof_names = [name]
        msg.values = [value]
        publisher.publish(msg)


def main():
    rclpy.init()
    node = SmoothAllJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
