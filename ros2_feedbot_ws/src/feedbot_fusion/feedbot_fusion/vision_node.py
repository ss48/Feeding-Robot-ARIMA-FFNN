"""
Vision Node — ML-based food detection with 3D position estimation

Uses NVIDIA jetson-inference (detectNet / SSD-Mobilenet) for object detection
and segmentation when available (Jetson hardware).  Falls back to HSV colour
segmentation when jetson-inference is not installed (e.g. dev PC / Gazebo).

jetson-inference repo: https://github.com/dusty-nv/jetson-inference
  - detectNet uses SSD-Mobilenet-v2 trained on COCO (91 classes including food)
  - COCO food classes: apple (53), banana (52), orange (55), sandwich (48),
    broccoli (56), carrot (57), cake (61), pizza (59), donut (60), etc.

Camera intrinsics from URDF: horizontal_fov=1.047 rad (60 deg), 640x480

Publishes:
  /food_visible       (Bool)               — detection flag
  /food_center        (Point)              — cx, cy, area
  /food_type          (String)             — detected food class name
  /detected_fruits    (Float64MultiArray)  — flat [cx, cy, area, class_id, ...]
  /food_bearing       (Point)              — bearing_h (rad), bearing_v (rad),
                                             estimated_depth (m)
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
# Try to import jetson-inference for ML detection
# ───────────────────────────────────────────────────────────────────────
try:
    import jetson_inference
    import jetson_utils
    HAS_JETSON_INFERENCE = True
except ImportError:
    HAS_JETSON_INFERENCE = False

# ───────────────────────────────────────────────────────────────────────
# COCO class IDs for food items (SSD-Mobilenet-v2)
# ───────────────────────────────────────────────────────────────────────
FOOD_CLASS_IDS = {
    47: 'cup',
    48: 'fork',
    49: 'knife',
    50: 'spoon',
    51: 'bowl',
    52: 'banana',
    53: 'apple',
    54: 'sandwich',
    55: 'orange',
    56: 'broccoli',
    57: 'carrot',
    58: 'hot_dog',
    59: 'pizza',
    60: 'donut',
    61: 'cake',
}

# Food names we care about for the feeding robot
FOOD_NAMES = {
    'apple', 'banana', 'orange', 'sandwich', 'broccoli', 'carrot',
    'hot_dog', 'pizza', 'donut', 'cake', 'strawberry',
    'grape', 'kiwi',
}

# ───────────────────────────────────────────────────────────────────────
# HSV fallback (for Gazebo / when jetson-inference unavailable)
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

FRUIT_COLORS = {
    'apple':      (30,  30, 220),
    'strawberry': (50,  30, 180),
    'banana':     (0,  230, 255),
    'grape':      (180, 50, 130),
    'orange':     (0,  140, 255),
    'kiwi':       (30, 140,  50),
}

FRUIT_IDS = {name: idx for idx, name in enumerate(FRUIT_HSV_RANGES)}
MIN_CONTOUR_AREA = 40

# ───────────────────────────────────────────────────────────────────────
# Camera intrinsics (from URDF: horizontal_fov=1.047 rad, 640x480)
# ───────────────────────────────────────────────────────────────────────
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
H_FOV = 1.047
V_FOV = H_FOV * (IMAGE_HEIGHT / IMAGE_WIDTH)

FX = (IMAGE_WIDTH / 2.0) / math.tan(H_FOV / 2.0)
FY = (IMAGE_HEIGHT / 2.0) / math.tan(V_FOV / 2.0)
CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0

# Known approximate food diameters (metres) for monocular depth
FOOD_DIAMETERS = {
    'apple':      0.08,
    'strawberry': 0.035,
    'banana':     0.04,
    'grape':      0.02,
    'orange':     0.075,
    'kiwi':       0.06,
    'sandwich':   0.12,
    'broccoli':   0.10,
    'carrot':     0.03,
    'pizza':      0.15,
    'donut':      0.10,
    'cake':       0.12,
    'hot_dog':    0.04,
}
DEFAULT_FOOD_DIAMETER = 0.06

# ML detection confidence threshold
ML_CONFIDENCE_THRESHOLD = 0.35


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self._kernel = np.ones((5, 5), np.uint8)

        # Parameters
        self.declare_parameter('camera_topic', '/feeding_robot/camera/image_raw')
        self.declare_parameter('detection_method', 'auto')  # 'auto', 'ml', 'hsv'
        self.declare_parameter('ml_model', 'ssd-mobilenet-v2')
        self.declare_parameter('ml_confidence', ML_CONFIDENCE_THRESHOLD)

        camera_topic = self.get_parameter('camera_topic').value
        self._detection_method = self.get_parameter('detection_method').value
        ml_model = self.get_parameter('ml_model').value
        self._ml_confidence = self.get_parameter('ml_confidence').value

        # Decide detection method
        if self._detection_method == 'auto':
            self._use_ml = HAS_JETSON_INFERENCE
        elif self._detection_method == 'ml':
            self._use_ml = HAS_JETSON_INFERENCE
            if not HAS_JETSON_INFERENCE:
                self.get_logger().warn(
                    'ML detection requested but jetson-inference not installed! '
                    'Falling back to HSV.')
        else:
            self._use_ml = False

        # Initialize ML detector if available
        self._net = None
        if self._use_ml:
            try:
                # Try ONNX model first (required for TensorRT 10+)
                import os
                onnx_path = os.path.expanduser(
                    '~/jetson-inference/data/networks/ssd_mobilenet_v2.onnx')
                labels_path = os.path.expanduser(
                    '~/jetson-inference/data/networks/ssd_coco_labels.txt')

                if os.path.exists(onnx_path):
                    self._net = jetson_inference.detectNet(
                        argv=[
                            '--model=' + onnx_path,
                            '--labels=' + labels_path,
                            '--input-blob=input_0',
                            '--output-cvg=scores',
                            '--output-bbox=boxes',
                            '--threshold=' + str(self._ml_confidence),
                        ])
                    self.get_logger().info(
                        f'ML detection initialized: ONNX model '
                        f'(confidence threshold: {self._ml_confidence})')
                else:
                    # Fallback to built-in model name
                    self._net = jetson_inference.detectNet(
                        ml_model, threshold=self._ml_confidence)
                    self.get_logger().info(
                        f'ML detection initialized: {ml_model} '
                        f'(confidence threshold: {self._ml_confidence})')
            except Exception as e:
                self.get_logger().error(
                    f'Failed to load ML model: {e} — falling back to HSV')
                self._use_ml = False

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # Publishers
        self.visible_pub = self.create_publisher(Bool, '/food_visible', 10)
        self.center_pub = self.create_publisher(Point, '/food_center', 10)
        self.type_pub = self.create_publisher(String, '/food_type', 10)
        self.fruits_pub = self.create_publisher(
            Float64MultiArray, '/detected_fruits', 10)
        self.bearing_pub = self.create_publisher(Point, '/food_bearing', 10)
        self.plate_pub = self.create_publisher(Bool, '/plate_detected', 10)

        method = 'jetson-inference ML (detectNet)' if self._use_ml else 'HSV colour'
        self.get_logger().info(
            f'Vision node started — detection method: {method}')

    # ----------------------------------------------------------------
    # ML detection using jetson-inference detectNet
    # ----------------------------------------------------------------
    def _detect_ml(self, frame):
        """Run ML object detection. Returns dict of name → (cx, cy, area, x, y, w, h)."""
        detections = {}

        # Convert OpenCV BGR to CUDA image for jetson-inference
        img_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        cuda_img = jetson_utils.cudaFromNumpy(img_rgba)

        # Run detection
        det_list = self._net.Detect(cuda_img, overlay='none')

        for det in det_list:
            class_id = det.ClassID
            class_name = self._net.GetClassDesc(class_id).lower()

            # Check if it's a food item
            is_food = (class_name in FOOD_NAMES or
                       class_id in FOOD_CLASS_IDS)
            if not is_food:
                continue

            # Use the COCO class name
            if class_id in FOOD_CLASS_IDS:
                name = FOOD_CLASS_IDS[class_id]
            else:
                name = class_name

            cx = int(det.Center[0])
            cy = int(det.Center[1])
            x = int(det.Left)
            y = int(det.Top)
            w = int(det.Width)
            h = int(det.Height)
            area = float(w * h)

            # Keep largest detection per class
            if name not in detections or area > detections[name][2]:
                detections[name] = (cx, cy, area, x, y, w, h)

            # Draw on frame
            color = FRUIT_COLORS.get(name, (0, 255, 0))
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            conf = det.Confidence
            bh, bv, depth = self._estimate_bearing_and_depth(cx, cy, w, name)
            cv2.putText(frame, f"{name} {conf:.0%} d={depth:.2f}m",
                        (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        color, 1, cv2.LINE_AA)

        return detections

    # ----------------------------------------------------------------
    # HSV fallback detection
    # ----------------------------------------------------------------
    def _detect_hsv(self, frame):
        """Run HSV colour-based detection. Returns dict of name → (cx, cy, area, x, y, w, h)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = {}

        for fruit_name in FRUIT_HSV_RANGES:
            result = self._detect_fruit_hsv(hsv, fruit_name)
            if result is not None:
                detections[fruit_name] = result

        # Draw on frame
        for name, (cx, cy, area, x, y, w, h) in detections.items():
            color = FRUIT_COLORS.get(name, (255, 255, 255))
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            bh, bv, depth = self._estimate_bearing_and_depth(cx, cy, w, name)
            cv2.putText(frame, f"{name} d={depth:.2f}m",
                        (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        color, 1, cv2.LINE_AA)

        return detections

    def _detect_fruit_hsv(self, hsv, fruit_name):
        """HSV detection for a single fruit type."""
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
    # Plate detection via shape analysis
    # ----------------------------------------------------------------
    def _detect_plate(self, frame):
        """Detect plate/bowl using contour circularity in the frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5000:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter < 1:
                continue
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity > 0.4:
                return True
        return False

    # ----------------------------------------------------------------
    # 3D bearing estimation from pinhole camera model
    # ----------------------------------------------------------------
    def _estimate_bearing_and_depth(self, cx, cy, bbox_w, food_name):
        """Compute bearing angles and monocular depth estimate."""
        bearing_h = math.atan2(cx - CX, FX)
        bearing_v = math.atan2(cy - CY, FY)

        known_diameter = FOOD_DIAMETERS.get(food_name, DEFAULT_FOOD_DIAMETER)
        if bbox_w > 5:
            estimated_depth = (known_diameter * FX) / bbox_w
        else:
            estimated_depth = 1.0

        return bearing_h, bearing_v, estimated_depth

    # ----------------------------------------------------------------
    # Main callback
    # ----------------------------------------------------------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Plate detection
        plate_found = self._detect_plate(frame)
        self.plate_pub.publish(Bool(data=plate_found))

        # Food detection (ML or HSV)
        if self._use_ml and self._net is not None:
            detections = self._detect_ml(frame)
        else:
            detections = self._detect_hsv(frame)

        # --- Publish all detections as flat array ---
        flat = []
        for name, (cx, cy, area, *_) in detections.items():
            type_id = float(FRUIT_IDS.get(name, hash(name) % 100))
            flat.extend([float(cx), float(cy), float(area), type_id])
        fruits_msg = Float64MultiArray()
        fruits_msg.data = flat
        self.fruits_pub.publish(fruits_msg)

        # --- Primary food = largest detection ---
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

            bh, bv, depth = self._estimate_bearing_and_depth(
                cx, cy, w, primary_name)
            bearing_msg.x = bh
            bearing_msg.y = bv
            bearing_msg.z = depth

            self.visible_pub.publish(visible_msg)
            self.center_pub.publish(center_msg)
            self.type_pub.publish(type_msg)
            self.bearing_pub.publish(bearing_msg)

            method = 'ML' if self._use_ml else 'HSV'
            self.get_logger().info(
                f"[{method}] Detected {len(detections)} food(s), "
                f"primary={primary_name} x={cx} y={cy} area={area:.0f} "
                f"bearing=({math.degrees(bh):.1f}, {math.degrees(bv):.1f}) "
                f"depth={depth:.3f}m")
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
