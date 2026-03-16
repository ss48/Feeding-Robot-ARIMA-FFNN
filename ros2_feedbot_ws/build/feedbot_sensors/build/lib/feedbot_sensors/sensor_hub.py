#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from tf2_msgs.msg import TFMessage

class SensorHub(Node):
    def __init__(self):
        super().__init__('sensor_hub')

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)

        self.get_logger().info("Sensor hub started")

    def joint_cb(self, msg):
        # joint1 is index 2 in your setup
        self.get_logger().info(f"Joint1 position = {msg.position[2]:.3f}")

    def image_cb(self, msg):
        self.get_logger().info("Camera frame received")

    def tf_cb(self, msg):
        pass  # too noisy to print

def main():
    rclpy.init()
    node = SensorHub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

