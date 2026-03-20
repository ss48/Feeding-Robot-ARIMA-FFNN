"""
Vision Node — Multi-fruit detection with 3D position estimation

Detects food on the plate using HSV colour segmentation, then estimates
the food's 3D bearing angles using a pinhole camera model.  The ultrasonic
sensor (via the fusion node) provides the depth to complete the 3D position.

Camera intrinsics are derived from the URDF:
  horizontal_fov = 1.047 rad (60 deg), image 640x480

Publishes:
  /food_visible       (Bool)               — detection flag
  /food_center        (Point)              — cx, cy, area
  /food_type          (String)             — primary fruit name
  /detected_fruits    (Float64MultiArray)  — flat [cx, cy, area, type_id, ...]
  /food_bearing       (Point)              — bearing_h (rad), bearing_v (rad),
                                             estimated_depth (m) from apparent size
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float64MultiArray
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

import cv2
import numpy as np
import math


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

# ───────────────────────────────────────────────────────────────────────
# Camera intrinsics (from URDF: horizontal_fov=1.047 rad, 640x480)
# ───────────────────────────────────────────────────────────────────────
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
H_FOV = 1.047  # radians (60 degrees)
V_FOV = H_FOV * (IMAGE_HEIGHT / IMAGE_WIDTH)  # ~0.785 rad (45 deg)

# Focal length in pixels (pinhole model)
FX = (IMAGE_WIDTH / 2.0) / math.tan(H_FOV / 2.0)
FY = (IMAGE_HEIGHT / 2.0) / math.tan(V_FOV / 2.0)
CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0

# Known approximate food diameters (metres) for monocular depth estimation
FRUIT_DIAMETERS = {
    'apple':      0.08,
    'strawberry': 0.035,
    'banana':     0.04,   # width
    'grape':      0.02,
    'orange':     0.075,
    'kiwi':       0.06,
}
DEFAULT_FRUIT_DIAMETER = 0.06


class VisionNode(Node):

    def __init__(self):

        super().__init__('vision_node')

        self.bridge = CvBridge()
        self._kernel = np.ones((5, 5), np.uint8)

        # Subscribe to camera (Gazebo bridge topic)
        self.declare_parameter('camera_topic', '/feeding_robot/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
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

        # Camera bearing + monocular depth estimate for fusion with ultrasonic
        self.bearing_pub = self.create_publisher(Point, '/food_bearing', 10)

        self.get_logger().info(
            'Vision node started (multi-fruit detection + 3D bearing estimation)')

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
    # 3D bearing estimation from pinhole camera model
    # ----------------------------------------------------------------
    def _estimate_bearing_and_depth(self, cx, cy, bbox_w, fruit_name):
        """Compute bearing angles and monocular depth estimate.

        Returns (bearing_h, bearing_v, estimated_depth_m).
        bearing_h: horizontal angle from camera optical axis (rad, +right)
        bearing_v: vertical angle from camera optical axis (rad, +down)
        estimated_depth_m: depth from apparent size (metres)
        """
        # Bearing angles via pinhole model
        bearing_h = math.atan2(cx - CX, FX)
        bearing_v = math.atan2(cy - CY, FY)

        # Monocular depth from known fruit diameter
        known_diameter = FRUIT_DIAMETERS.get(fruit_name, DEFAULT_FRUIT_DIAMETER)
        if bbox_w > 5:
            # depth = (real_size * focal_length) / pixel_size
            estimated_depth = (known_diameter * FX) / bbox_w
        else:
            estimated_depth = 1.0  # fallback far distance

        return bearing_h, bearing_v, estimated_depth

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
            bh, bv, depth = self._estimate_bearing_and_depth(cx, cy, w, name)
            cv2.putText(frame, f"{name} d={depth:.2f}m",
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
        bearing_msg = Point()

        if detections:
            primary_name = max(detections, key=lambda n: detections[n][2])
            cx, cy, area, x, y, w, h = detections[primary_name]

            visible_msg.data = True
            center_msg.x = float(cx)
            center_msg.y = float(cy)
            center_msg.z = float(area)
            type_msg.data = primary_name

            # 3D bearing + monocular depth
            bh, bv, depth = self._estimate_bearing_and_depth(
                cx, cy, w, primary_name)
            bearing_msg.x = bh       # horizontal bearing (rad)
            bearing_msg.y = bv       # vertical bearing (rad)
            bearing_msg.z = depth    # monocular depth estimate (m)

            self.visible_pub.publish(visible_msg)
            self.center_pub.publish(center_msg)
            self.type_pub.publish(type_msg)
            self.bearing_pub.publish(bearing_msg)

            self.get_logger().info(
                f"Detected {len(detections)} fruit(s), "
                f"primary={primary_name} x={cx} y={cy} area={area:.0f} "
                f"bearing=({math.degrees(bh):.1f}°, {math.degrees(bv):.1f}°) "
                f"depth={depth:.3f}m"
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
