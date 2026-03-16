import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

import cv2
import numpy as np


class VisionNode(Node):

    def __init__(self):

        super().__init__('vision_node')

        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.visible_pub = self.create_publisher(
            Bool,
            '/food_visible',
            10
        )

        # Publish food center + size
        self.center_pub = self.create_publisher(
            Point,
            '/food_center',
            10
        )

        self.get_logger().info('Vision node started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red color detection (strawberry/apple)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        # Remove noise
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.erode(mask,kernel,iterations=1)
        mask = cv2.dilate(mask,kernel,iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        visible_msg = Bool()
        center_msg = Point()

        if contours:

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 40:

                M = cv2.moments(largest)

                if M["m00"] != 0:

                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    visible_msg.data = True

                    center_msg.x = float(cx)
                    center_msg.y = float(cy)
                    center_msg.z = float(area)

                    self.visible_pub.publish(visible_msg)
                    self.center_pub.publish(center_msg)

                    # Draw detection
                    cv2.circle(frame,(cx,cy),6,(0,255,0),-1)

                    x,y,w,h = cv2.boundingRect(largest)
                    cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)

                    self.get_logger().info(
                        f"Food detected x={cx}, y={cy}, area={area:.1f}"
                    )

                    cv2.imshow("camera_view",frame)
                    cv2.imshow("mask",mask)
                    cv2.waitKey(1)

                    return

        # If no food detected
        visible_msg.data = False

        center_msg.x = -1.0
        center_msg.y = -1.0
        center_msg.z = 0.0

        self.visible_pub.publish(visible_msg)
        self.center_pub.publish(center_msg)

        self.get_logger().info("Food not detected")

        cv2.imshow("camera_view",frame)
        cv2.imshow("mask",mask)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
