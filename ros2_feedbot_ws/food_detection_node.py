#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FoodDetectionNode(Node):

    def __init__(self):
        super().__init__('food_detection_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info("Food detection node started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red color range (strawberry)
        lower_red1 = np.array([0,120,70])
        upper_red1 = np.array([10,255,255])

        lower_red2 = np.array([170,120,70])
        upper_red2 = np.array([180,255,255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:

            largest = max(contours, key=cv2.contourArea)

            x,y,w,h = cv2.boundingRect(largest)

            cx = int(x + w/2)
            cy = int(y + h/2)

            self.get_logger().info(f"Food detected at pixel: {cx},{cy}")

            cv2.circle(frame,(cx,cy),5,(0,255,0),-1)

        cv2.imshow("Food Detection", frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = FoodDetectionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
