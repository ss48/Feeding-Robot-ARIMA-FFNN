#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand

class HoldArm(Node):

    def __init__(self):
        super().__init__('hold_arm')

        self.pub1 = self.create_publisher(MultiDOFCommand, '/joint1_controller/reference', 10)
        self.pub2 = self.create_publisher(MultiDOFCommand, '/joint2_controller/reference', 10)
        self.pub3 = self.create_publisher(MultiDOFCommand, '/joint3_controller/reference', 10)
        self.pub4 = self.create_publisher(MultiDOFCommand, '/joint4_controller/reference', 10)

        self.timer = self.create_timer(0.01, self.publish_reference)  # 100 Hz

    def publish_reference(self):

        msg = MultiDOFCommand()
        msg.dof_names = ['joint']
        msg.values = [0.0]
        msg.values_dot = [0.0]

        self.pub1.publish(msg)
        self.pub2.publish(msg)
        self.pub3.publish(msg)
        self.pub4.publish(msg)

def main():
    rclpy.init()
    node = HoldArm()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
