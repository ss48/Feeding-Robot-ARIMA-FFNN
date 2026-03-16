#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand


class StepTest(Node):

    def __init__(self):
        super().__init__('step_test_all_joints')

        self.get_logger().info("Stable continuous step test running...")

        self.joint_publishers = []
        self.joint_names = []

        for i in range(1, 5):
            joint_name = f'joint{i}'
            topic = f'/{joint_name}_controller/reference'

            pub = self.create_publisher(MultiDOFCommand, topic, 10)
            self.joint_publishers.append(pub)
            self.joint_names.append(joint_name)

        self.current_value = 0.0

        # Fast publish timer (100 Hz)
        self.publish_timer = self.create_timer(0.01, self.publish_reference)

        # Slow toggle timer (every 3 seconds)
        self.toggle_timer = self.create_timer(3.0, self.toggle_step)

    def publish_reference(self):

        for pub, joint_name in zip(self.joint_publishers, self.joint_names):
            cmd = MultiDOFCommand()
            cmd.dof_names = [joint_name]
            cmd.values = [self.current_value]
            cmd.values_dot = []
            pub.publish(cmd)

    def toggle_step(self):

        if self.current_value == 0.0:
            self.current_value = 0.5
        else:
            self.current_value = 0.0

        self.get_logger().info(f"Switched step to: {self.current_value}")


def main():
    rclpy.init()
    node = StepTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
