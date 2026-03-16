#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand

class JointStepTest(Node):

    def __init__(self):
        super().__init__('joint1_step_test')

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            '/joint1_controller/reference',
            10
        )

        self.current_value = 0.0

        # Continuous publishing (100Hz)
        self.publish_timer = self.create_timer(0.01, self.publish_reference)

        # Toggle step every 5 sec
        self.toggle_timer = self.create_timer(5.0, self.toggle_step)

        self.get_logger().info("Joint1 step test running...")

    def publish_reference(self):
        cmd = MultiDOFCommand()
        cmd.dof_names = ['joint1']
        cmd.values = [self.current_value]
        cmd.values_dot = []
        self.publisher.publish(cmd)

    def toggle_step(self):
        self.current_value = 0.5 if self.current_value == 0.0 else 0.0
        self.get_logger().info(f"Step switched to: {self.current_value}")

def main():
    rclpy.init()
    node = JointStepTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
