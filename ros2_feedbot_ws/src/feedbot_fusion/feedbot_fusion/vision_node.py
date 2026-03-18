import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float64MultiArray
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

import cv2
import numpy as np


# ───────────────────────────────────────────────────────────────────────
# HSV colour ranges per fruit type.
# Each entry is a list of (lower, upper) HSV tuples.  Red wraps around
# hue 0/180 so it needs two ranges.
# ───────────────────────────────────────────────────────────────────────
FRUIT_HSV_RANGES = {
    'apple':      [((0, 120, 70),  (10, 255, 255)),
                   ((170, 120, 70), (180, 255, 255))],
    'strawberry': [((0, 150, 100), (8, 255, 255)),
                   ((172, 150, 100), (180, 255, 255))],
    'banana':     [((20, 100, 100), (35, 255, 255))],
    'grape':      [((125, 50, 50),  (155, 255, 255))],
    'orange':     [((10, 150, 150), (22, 255, 255))],
    'kiwi':       [((35, 50, 50),   (85, 255, 200))],
}

# Fruit name → display colour (BGR) for bounding boxes / labels
FRUIT_COLORS = {
    'apple':      (30,  30, 220),
    'strawberry': (50,  30, 180),
    'banana':     (0,  230, 255),
    'grape':      (180, 50, 130),
    'orange':     (0,  140, 255),
    'kiwi':       (30, 140,  50),
}

# Integer IDs for the /detected_fruits flat array
FRUIT_IDS = {name: idx for idx, name in enumerate(FRUIT_HSV_RANGES)}

MIN_CONTOUR_AREA = 40


class VisionNode(Node):

    def __init__(self):

        super().__init__('vision_node')

        self.bridge = CvBridge()
        self._kernel = np.ones((5, 5), np.uint8)

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/image_raw',
            self.image_callback,
            10
        )

        # --- Backward-compatible publishers (single primary food) ---
        self.visible_pub = self.create_publisher(Bool, '/food_visible', 10)
        self.center_pub = self.create_publisher(Point, '/food_center', 10)

        # --- New publishers ---
        self.type_pub = self.create_publisher(String, '/food_type', 10)
        self.fruits_pub = self.create_publisher(
            Float64MultiArray, '/detected_fruits', 10)

        self.get_logger().info('Vision node started (multi-fruit detection)')

    # ----------------------------------------------------------------
    # Detection helper
    # ----------------------------------------------------------------
    def _detect_fruit(self, hsv, fruit_name):
        """Return (cx, cy, area, x, y, w, h) or None for *fruit_name*."""
        ranges = FRUIT_HSV_RANGES[fruit_name]
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

        mask = cv2.erode(mask, self._kernel, iterations=1)
        mask = cv2.dilate(mask, self._kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < MIN_CONTOUR_AREA:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(largest)
        return (cx, cy, area, x, y, w, h)

    # ----------------------------------------------------------------
    # Main callback
    # ----------------------------------------------------------------
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = {}  # name → (cx, cy, area, x, y, w, h)

        for fruit_name in FRUIT_HSV_RANGES:
            result = self._detect_fruit(hsv, fruit_name)
            if result is not None:
                detections[fruit_name] = result

        # --- Draw all detections on frame ---
        for name, (cx, cy, area, x, y, w, h) in detections.items():
            color = FRUIT_COLORS.get(name, (255, 255, 255))
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"{name} ({area:.0f})",
                        (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        color, 1, cv2.LINE_AA)

        # --- Publish all detections as flat array ---
        # Layout: [cx, cy, area, type_id, cx, cy, area, type_id, ...]
        flat = []
        for name, (cx, cy, area, *_) in detections.items():
            flat.extend([float(cx), float(cy), float(area),
                         float(FRUIT_IDS[name])])
        fruits_msg = Float64MultiArray()
        fruits_msg.data = flat
        self.fruits_pub.publish(fruits_msg)

        # --- Primary food = largest detection (backward compat) ---
        visible_msg = Bool()
        center_msg = Point()
        type_msg = String()

        if detections:
            primary_name = max(detections, key=lambda n: detections[n][2])
            cx, cy, area = detections[primary_name][:3]

            visible_msg.data = True
            center_msg.x = float(cx)
            center_msg.y = float(cy)
            center_msg.z = float(area)
            type_msg.data = primary_name

            self.visible_pub.publish(visible_msg)
            self.center_pub.publish(center_msg)
            self.type_pub.publish(type_msg)

            self.get_logger().info(
                f"Detected {len(detections)} fruit(s), "
                f"primary={primary_name} x={cx} y={cy} area={area:.0f}"
            )
        else:
            visible_msg.data = False
            center_msg.x = -1.0
            center_msg.y = -1.0
            center_msg.z = 0.0
            type_msg.data = ''

            self.visible_pub.publish(visible_msg)
            self.center_pub.publish(center_msg)
            self.type_pub.publish(type_msg)

            self.get_logger().info("No food detected")

        cv2.imshow("camera_view", frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
