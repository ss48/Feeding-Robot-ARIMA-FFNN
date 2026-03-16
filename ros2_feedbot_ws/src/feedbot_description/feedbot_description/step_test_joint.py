#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand

class JointStepTest(Node):

    def __init__(self, joint_name):

        super().__init__('joint_step_test')

        topic = f'/{joint_name}_controller/reference'

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            topic,
            10
        )

        self.joint_name = joint_name
        self.current_value = 0.0

        self.publish_timer = self.create_timer(0.01, self.publish_reference)
        self.toggle_timer = self.create_timer(5.0, self.toggle_step)

        self.get_logger().info(f"Step test running for {joint_name}")

    def publish_reference(self):

        cmd = MultiDOFCommand()
        cmd.dof_names = [self.joint_name]
        cmd.values = [self.current_value]
        cmd.values_dot = []
        self.publisher.publish(cmd)

    def toggle_step(self):

        self.current_value = 0.5 if self.current_value == 0.0 else 0.0
        self.get_logger().info(f"Step switched to: {self.current_value}")

def main():

    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: ros2 run feedbot_description step_test_joint jointX")
        return

    joint_name = sys.argv[1]

    node = JointStepTest(joint_name)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
