"""
Vision Node — Object and food detection with 3D position estimation.

Detection backends (selected automatically or via parameter):
  1. jetson-inference (Jetson Orin/Nano with NVIDIA GPU)
  2. MediaPipe Object Detection (Pi 4/5 CPU — EfficientDet-Lite)
  3. HSV colour segmentation (fallback for Gazebo / basic)

All backends detect COCO objects including food items, plates, utensils.
MediaPipe runs at ~5-8fps on Pi 4 with EfficientDet-Lite0.

Publishes:
  /food_visible       (Bool)               — food detection flag
  /food_center        (Point)              — cx, cy, area of primary food
  /food_type          (String)             — detected food class name
  /detected_fruits    (Float64MultiArray)  — flat [cx, cy, area, class_id, ...]
  /food_bearing       (Point)              — bearing_h, bearing_v, depth (m)
  /plate_detected     (Bool)               — plate/bowl found
  /objects_detected   (String)             — comma-separated list of all detected objects
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
import os

# ───────────────────────────────────────────────────────────────────────
# Try to import detection backends
# ───────────────────────────────────────────────────────────────────────
try:
    import jetson_inference
    import jetson_utils
    HAS_JETSON = True
except ImportError:
    HAS_JETSON = False

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False

# ───────────────────────────────────────────────────────────────────────
# COCO class definitions
# ───────────────────────────────────────────────────────────────────────
FOOD_NAMES = {
    'apple', 'banana', 'orange', 'sandwich', 'broccoli', 'carrot',
    'hot_dog', 'pizza', 'donut', 'cake', 'strawberry', 'grape', 'kiwi',
}

PLATE_NAMES = {'bowl', 'cup', 'plate', 'dining table'}

UTENSIL_NAMES = {'fork', 'knife', 'spoon'}

# All objects we care about for the feeding robot
ALL_OBJECT_NAMES = FOOD_NAMES | PLATE_NAMES | UTENSIL_NAMES

# COCO class IDs (for jetson-inference SSD-Mobilenet)
FOOD_CLASS_IDS = {
    47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon', 51: 'bowl',
    52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
    56: 'broccoli', 57: 'carrot', 58: 'hot_dog', 59: 'pizza',
    60: 'donut', 61: 'cake',
}

# ───────────────────────────────────────────────────────────────────────
# HSV fallback ranges
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
    'apple': (30, 30, 220), 'strawberry': (50, 30, 180),
    'banana': (0, 230, 255), 'grape': (180, 50, 130),
    'orange': (0, 140, 255), 'kiwi': (30, 140, 50),
}

FRUIT_IDS = {name: idx for idx, name in enumerate(FRUIT_HSV_RANGES)}
MIN_CONTOUR_AREA = 40

# ───────────────────────────────────────────────────────────────────────
# Camera intrinsics (Pi Camera V2.1, 640x480, HFOV=60°)
# ───────────────────────────────────────────────────────────────────────
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
H_FOV = 1.047
V_FOV = H_FOV * (IMAGE_HEIGHT / IMAGE_WIDTH)
FX = (IMAGE_WIDTH / 2.0) / math.tan(H_FOV / 2.0)
FY = (IMAGE_HEIGHT / 2.0) / math.tan(V_FOV / 2.0)
CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0

# Known food diameters (metres) for monocular depth estimation
FOOD_DIAMETERS = {
    'apple': 0.08, 'strawberry': 0.035, 'banana': 0.04, 'grape': 0.02,
    'orange': 0.075, 'kiwi': 0.06, 'sandwich': 0.12, 'broccoli': 0.10,
    'carrot': 0.03, 'pizza': 0.15, 'donut': 0.10, 'cake': 0.12,
    'hot_dog': 0.04,
}
DEFAULT_FOOD_DIAMETER = 0.06
ML_CONFIDENCE_THRESHOLD = 0.35


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self._kernel = np.ones((5, 5), np.uint8)

        # Parameters
        self.declare_parameter('camera_topic', '/feeding_robot/camera/image_raw')
        self.declare_parameter('detection_method', 'auto')  # 'auto', 'mediapipe', 'ml', 'hsv'
        self.declare_parameter('ml_confidence', ML_CONFIDENCE_THRESHOLD)
        self.declare_parameter('process_every_n', 2)

        camera_topic = self.get_parameter('camera_topic').value
        detection_method = self.get_parameter('detection_method').value
        self._ml_confidence = self.get_parameter('ml_confidence').value
        self._process_every_n = self.get_parameter('process_every_n').value

        self._frame_count = 0
        self._backend = 'hsv'  # default fallback
        self._net = None
        self._mp_detector = None

        # Select detection backend
        if detection_method == 'auto':
            if HAS_JETSON:
                self._init_jetson()
            elif HAS_MEDIAPIPE:
                self._init_mediapipe()
            else:
                self._backend = 'hsv'
        elif detection_method == 'ml' and HAS_JETSON:
            self._init_jetson()
        elif detection_method == 'mediapipe' and HAS_MEDIAPIPE:
            self._init_mediapipe()
        else:
            self._backend = 'hsv'

        # Subscriber
        self.create_subscription(Image, camera_topic, self.image_callback, 10)

        # Publishers
        self.visible_pub = self.create_publisher(Bool, '/food_visible', 10)
        self.center_pub = self.create_publisher(Point, '/food_center', 10)
        self.type_pub = self.create_publisher(String, '/food_type', 10)
        self.fruits_pub = self.create_publisher(Float64MultiArray, '/detected_fruits', 10)
        self.bearing_pub = self.create_publisher(Point, '/food_bearing', 10)
        self.plate_pub = self.create_publisher(Bool, '/plate_detected', 10)
        self.objects_pub = self.create_publisher(String, '/objects_detected', 10)

        self.get_logger().info(
            f'Vision node started — backend: {self._backend}, '
            f'confidence: {self._ml_confidence}, '
            f'process every {self._process_every_n} frames')

    # ────────────────────────────────────────────────────────────────
    # Backend initialization
    # ────────────────────────────────────────────────────────────────
    def _init_jetson(self):
        try:
            self._net = jetson_inference.detectNet(
                'ssd-mobilenet-v2', threshold=self._ml_confidence)
            self._backend = 'jetson'
            self.get_logger().info('Detection backend: jetson-inference (SSD-Mobilenet-v2)')
        except Exception as e:
            self.get_logger().error(f'Jetson init failed: {e}')
            if HAS_MEDIAPIPE:
                self._init_mediapipe()
            else:
                self._backend = 'hsv'

    def _init_mediapipe(self):
        try:
            # Use EfficientDet-Lite0 float32 for Pi 4 CPU
            model_path = os.path.expanduser(
                '~/.mediapipe/efficientdet_lite0_float32.tflite')
            if not os.path.exists(model_path):
                self.get_logger().info('Downloading EfficientDet-Lite0 float32 model...')
                os.makedirs(os.path.dirname(model_path), exist_ok=True)
                import urllib.request
                url = ('https://storage.googleapis.com/mediapipe-models/'
                       'object_detector/efficientdet_lite0/float32/latest/'
                       'efficientdet_lite0_float32.tflite')
                urllib.request.urlretrieve(url, model_path)
                self.get_logger().info(f'Model downloaded to {model_path}')

            base_options = mp_python.BaseOptions(model_asset_path=model_path)
            options = mp_vision.ObjectDetectorOptions(
                base_options=base_options,
                max_results=10,
                score_threshold=self._ml_confidence,
            )
            self._mp_detector = mp_vision.ObjectDetector.create_from_options(options)
            self._backend = 'mediapipe'
            self.get_logger().info(
                'Detection backend: MediaPipe EfficientDet-Lite0 float32 (CPU)')
        except Exception as e:
            self.get_logger().error(f'MediaPipe init failed: {e}')
            self._backend = 'hsv'

    # ────────────────────────────────────────────────────────────────
    # MediaPipe detection
    # ────────────────────────────────────────────────────────────────
    def _detect_mediapipe(self, frame):
        """Run MediaPipe object detection. Returns (food_detections, all_detections)."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = self._mp_detector.detect(mp_image)

        food_detections = {}
        all_names = []

        for det in results.detections:
            name = det.categories[0].category_name.lower()
            score = det.categories[0].score
            all_names.append(f'{name}({score:.2f})')

            bbox = det.bounding_box
            x, y, w, h = bbox.origin_x, bbox.origin_y, bbox.width, bbox.height
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            area = w * h

            # Draw on frame
            color = (0, 255, 0) if name in FOOD_NAMES else (255, 200, 0)
            cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
            cv2.putText(frame, f'{name} {score:.2f}',
                        (int(x), int(y) - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, color, 1, cv2.LINE_AA)

            if name in FOOD_NAMES:
                food_detections[name] = (cx, cy, area, int(x), int(y), int(w), int(h))

        return food_detections, all_names

    # ────────────────────────────────────────────────────────────────
    # Jetson-inference detection
    # ────────────────────────────────────────────────────────────────
    def _detect_jetson(self, frame):
        """Run jetson-inference detection. Returns (food_detections, all_detections)."""
        img_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        cuda_img = jetson_utils.cudaFromNumpy(img_rgba)
        det_list = self._net.Detect(cuda_img, overlay='none')

        food_detections = {}
        all_names = []

        for det in det_list:
            class_id = det.ClassID
            class_name = self._net.GetClassDesc(class_id).lower()
            all_names.append(f'{class_name}({det.Confidence:.2f})')

            if class_id in FOOD_CLASS_IDS:
                name = FOOD_CLASS_IDS[class_id]
            else:
                name = class_name

            cx = int(det.Center[0])
            cy = int(det.Center[1])
            x, y = int(det.Left), int(det.Top)
            w, h = int(det.Width), int(det.Height)
            area = w * h

            if name in FOOD_NAMES:
                food_detections[name] = (cx, cy, area, x, y, w, h)

        return food_detections, all_names

    # ────────────────────────────────────────────────────────────────
    # HSV fallback detection
    # ────────────────────────────────────────────────────────────────
    def _detect_hsv(self, frame):
        """Run HSV colour-based detection. Returns (food_detections, all_names)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        food_detections = {}
        all_names = []

        for fruit_name in FRUIT_HSV_RANGES:
            result = self._detect_fruit_hsv(hsv, fruit_name)
            if result is not None:
                food_detections[fruit_name] = result
                all_names.append(fruit_name)

        for name, (cx, cy, area, x, y, w, h) in food_detections.items():
            color = FRUIT_COLORS.get(name, (255, 255, 255))
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            bh, bv, depth = self._estimate_bearing_and_depth(cx, cy, w, name)
            cv2.putText(frame, f"{name} d={depth:.2f}m",
                        (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        color, 1, cv2.LINE_AA)

        return food_detections, all_names

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

    # ────────────────────────────────────────────────────────────────
    # Plate detection via shape analysis
    # ────────────────────────────────────────────────────────────────
    def _detect_plate(self, frame, all_names):
        """Detect plate — from ML detections or shape analysis fallback."""
        # Check if ML already found a plate/bowl
        for name in all_names:
            obj = name.split('(')[0]  # strip confidence
            if obj in PLATE_NAMES:
                return True

        # Shape-based fallback
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

    # ────────────────────────────────────────────────────────────────
    # 3D bearing estimation from pinhole camera model
    # ────────────────────────────────────────────────────────────────
    def _estimate_bearing_and_depth(self, cx, cy, bbox_w, food_name):
        bearing_h = math.atan2(cx - CX, FX)
        bearing_v = math.atan2(cy - CY, FY)
        known_diameter = FOOD_DIAMETERS.get(food_name, DEFAULT_FOOD_DIAMETER)
        if bbox_w > 5:
            estimated_depth = (known_diameter * FX) / bbox_w
        else:
            estimated_depth = 1.0
        return bearing_h, bearing_v, estimated_depth

    # ────────────────────────────────────────────────────────────────
    # Main callback
    # ────────────────────────────────────────────────────────────────
    def image_callback(self, msg):
        self._frame_count += 1
        if self._frame_count % self._process_every_n != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run detection based on backend
        if self._backend == 'jetson':
            food_detections, all_names = self._detect_jetson(frame)
        elif self._backend == 'mediapipe':
            food_detections, all_names = self._detect_mediapipe(frame)
        else:
            food_detections, all_names = self._detect_hsv(frame)

        # Plate detection
        plate_found = self._detect_plate(frame, all_names)
        self.plate_pub.publish(Bool(data=plate_found))

        # Publish all detected object names
        self.objects_pub.publish(String(data=', '.join(all_names) if all_names else ''))

        # Publish food detections
        flat = []
        for name, (cx, cy, area, *_) in food_detections.items():
            type_id = float(FRUIT_IDS.get(name, hash(name) % 100))
            flat.extend([float(cx), float(cy), float(area), type_id])
        fruits_msg = Float64MultiArray()
        fruits_msg.data = flat
        self.fruits_pub.publish(fruits_msg)

        # Primary food = largest detection
        visible_msg = Bool()
        center_msg = Point()
        type_msg = String()
        bearing_msg = Point()

        if food_detections:
            primary_name = max(food_detections, key=lambda n: food_detections[n][2])
            cx, cy, area, x, y, w, h = food_detections[primary_name]

            visible_msg.data = True
            center_msg.x = float(cx)
            center_msg.y = float(cy)
            center_msg.z = float(area)
            type_msg.data = primary_name

            bh, bv, depth = self._estimate_bearing_and_depth(cx, cy, w, primary_name)
            bearing_msg.x = bh
            bearing_msg.y = bv
            bearing_msg.z = depth

            self.visible_pub.publish(visible_msg)
            self.center_pub.publish(center_msg)
            self.type_pub.publish(type_msg)
            self.bearing_pub.publish(bearing_msg)

            self.get_logger().info(
                f'[{self._backend}] {len(food_detections)} food(s), '
                f'primary={primary_name} ({cx},{cy}) '
                f'depth={depth:.2f}m | all: {", ".join(all_names)}',
                throttle_duration_sec=2.0)
        else:
            visible_msg.data = False
            center_msg.x = -1.0
            center_msg.y = -1.0
            center_msg.z = 0.0
            type_msg.data = ''

            self.visible_pub.publish(visible_msg)
            self.center_pub.publish(center_msg)
            self.type_pub.publish(type_msg)

            if all_names:
                self.get_logger().info(
                    f'[{self._backend}] No food — objects: {", ".join(all_names)}',
                    throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
