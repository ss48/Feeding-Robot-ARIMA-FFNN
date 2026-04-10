"""
Face Detection Node — MediaPipe Face Mesh for mouth state and face tracking.

Detects the user's face, determines if their mouth is open/closed,
estimates face position in 3D, and publishes feeding-readiness.

Runs on Raspberry Pi 5 CPU at ~8-10fps (processes every Nth frame).

Publishes:
  /face_detected          (Bool)    — face found in frame
  /face_center            (Point)   — cx, cy (pixels), z=face_area
  /face_bearing           (Point)   — bearing_h, bearing_v (rad), depth (m)
  /mouth_open             (Bool)    — MAR > threshold
  /mouth_aspect_ratio     (Float64) — raw MAR value
  /face_expression        (String)  — "ready" / "not_ready" / "no_face"
  /mouth_ready_prediction (Bool)    — face + mouth open + ready

Subscribes:
  /feeding_robot/camera/image_raw (Image)
"""

import math

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import mediapipe as mp

# Camera intrinsics (Pi Camera V2.1 at 640x480) — same as vision_node
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
H_FOV = 1.047  # 60 degrees
FX = (IMAGE_WIDTH / 2.0) / math.tan(H_FOV / 2.0)
FY = FX  # square pixels
CX = IMAGE_WIDTH / 2.0
CY = IMAGE_HEIGHT / 2.0

# Average adult inter-pupillary distance for depth estimation
AVERAGE_IPD_METERS = 0.063

# MediaPipe Face Mesh landmark indices
LM_UPPER_LIP_INNER = 13
LM_LOWER_LIP_INNER = 14
LM_MOUTH_LEFT = 78
LM_MOUTH_RIGHT = 308
LM_NOSE_TIP = 1

# Eye landmarks for Eye Aspect Ratio (alertness)
LM_LEFT_EYE = [33, 160, 158, 133, 153, 144]
LM_RIGHT_EYE = [362, 385, 387, 263, 373, 380]

# Iris landmarks for depth estimation (refine_landmarks=True required)
LM_LEFT_IRIS = 468
LM_RIGHT_IRIS = 473


class FaceNode(Node):

    def __init__(self):
        super().__init__('face_node')

        # Parameters
        self.declare_parameter('camera_topic', '/feeding_robot/camera/image_raw')
        self.declare_parameter('mar_threshold', 0.3)
        self.declare_parameter('ear_threshold', 0.2)
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('max_num_faces', 1)
        self.declare_parameter('process_every_n', 3)

        camera_topic = self.get_parameter('camera_topic').value
        self.mar_threshold = self.get_parameter('mar_threshold').value
        self.ear_threshold = self.get_parameter('ear_threshold').value
        det_conf = self.get_parameter('detection_confidence').value
        track_conf = self.get_parameter('tracking_confidence').value
        max_faces = self.get_parameter('max_num_faces').value
        self.process_every_n = self.get_parameter('process_every_n').value

        # MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=max_faces,
            refine_landmarks=True,  # enables iris landmarks for depth
            min_detection_confidence=det_conf,
            min_tracking_confidence=track_conf,
        )

        self.bridge = CvBridge()
        self.frame_count = 0

        # Subscribers
        self.create_subscription(Image, camera_topic, self.image_callback, 10)

        # Publishers
        self.face_detected_pub = self.create_publisher(Bool, '/face_detected', 10)
        self.face_center_pub = self.create_publisher(Point, '/face_center', 10)
        self.face_bearing_pub = self.create_publisher(Point, '/face_bearing', 10)
        self.mouth_open_pub = self.create_publisher(Bool, '/mouth_open', 10)
        self.mar_pub = self.create_publisher(Float64, '/mouth_aspect_ratio', 10)
        self.expression_pub = self.create_publisher(String, '/face_expression', 10)
        self.mouth_ready_pub = self.create_publisher(Bool, '/mouth_ready_prediction', 10)

        self.get_logger().info(
            f'Face node started (MAR threshold={self.mar_threshold}, '
            f'process every {self.process_every_n} frames)')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb)

        if not results.multi_face_landmarks:
            self._publish_no_face()
            return

        # Use first (closest/largest) face
        face_landmarks = results.multi_face_landmarks[0]
        landmarks = face_landmarks.landmark
        h, w = frame.shape[:2]

        # Face center from nose tip
        nose = landmarks[LM_NOSE_TIP]
        face_cx = nose.x * w
        face_cy = nose.y * h

        # Face bounding area (approximate from landmark spread)
        xs = [lm.x * w for lm in landmarks]
        ys = [lm.y * h for lm in landmarks]
        face_w = max(xs) - min(xs)
        face_h = max(ys) - min(ys)
        face_area = face_w * face_h

        # Mouth Aspect Ratio
        mar = self._compute_mar(landmarks, w, h)

        # Eye Aspect Ratio (alertness)
        ear = self._compute_ear(landmarks, w, h)

        # Depth from inter-pupillary distance
        depth = self._estimate_depth(landmarks, w)

        # Bearing angles
        bearing_h = math.atan2(face_cx - CX, FX)
        bearing_v = math.atan2(face_cy - CY, FY)

        # Expression classification
        mouth_is_open = mar > self.mar_threshold
        eyes_open = ear > self.ear_threshold
        expression = 'ready' if eyes_open else 'not_ready'
        mouth_ready = mouth_is_open and expression == 'ready'

        # Publish all topics
        self.face_detected_pub.publish(Bool(data=True))

        center_msg = Point()
        center_msg.x = face_cx
        center_msg.y = face_cy
        center_msg.z = face_area
        self.face_center_pub.publish(center_msg)

        bearing_msg = Point()
        bearing_msg.x = bearing_h
        bearing_msg.y = bearing_v
        bearing_msg.z = depth
        self.face_bearing_pub.publish(bearing_msg)

        self.mouth_open_pub.publish(Bool(data=mouth_is_open))
        self.mar_pub.publish(Float64(data=float(mar)))
        self.expression_pub.publish(String(data=expression))
        self.mouth_ready_pub.publish(Bool(data=mouth_ready))

    def _compute_mar(self, landmarks, w, h):
        """Mouth Aspect Ratio from inner lip landmarks."""
        upper = landmarks[LM_UPPER_LIP_INNER]
        lower = landmarks[LM_LOWER_LIP_INNER]
        left = landmarks[LM_MOUTH_LEFT]
        right = landmarks[LM_MOUTH_RIGHT]

        vertical = math.sqrt(
            ((upper.x - lower.x) * w) ** 2 +
            ((upper.y - lower.y) * h) ** 2)
        horizontal = math.sqrt(
            ((left.x - right.x) * w) ** 2 +
            ((left.y - right.y) * h) ** 2)

        return vertical / max(horizontal, 1e-6)

    def _compute_ear(self, landmarks, w, h):
        """Eye Aspect Ratio — average of both eyes for alertness detection."""
        left_ear = self._eye_ratio(landmarks, LM_LEFT_EYE, w, h)
        right_ear = self._eye_ratio(landmarks, LM_RIGHT_EYE, w, h)
        return (left_ear + right_ear) / 2.0

    def _eye_ratio(self, landmarks, eye_indices, w, h):
        """EAR for a single eye: vertical / horizontal distance ratio."""
        pts = [(landmarks[i].x * w, landmarks[i].y * h) for i in eye_indices]
        # pts: [outer_corner, upper1, upper2, inner_corner, lower1, lower2]
        vertical_1 = math.sqrt((pts[1][0] - pts[5][0]) ** 2 + (pts[1][1] - pts[5][1]) ** 2)
        vertical_2 = math.sqrt((pts[2][0] - pts[4][0]) ** 2 + (pts[2][1] - pts[4][1]) ** 2)
        horizontal = math.sqrt((pts[0][0] - pts[3][0]) ** 2 + (pts[0][1] - pts[3][1]) ** 2)
        return (vertical_1 + vertical_2) / (2.0 * max(horizontal, 1e-6))

    def _estimate_depth(self, landmarks, w):
        """Estimate face depth from inter-pupillary distance using iris landmarks."""
        left_iris = landmarks[LM_LEFT_IRIS]
        right_iris = landmarks[LM_RIGHT_IRIS]

        pixel_ipd = math.sqrt(
            ((left_iris.x - right_iris.x) * w) ** 2 +
            ((left_iris.y - right_iris.y) * w) ** 2)

        if pixel_ipd < 5.0:
            return 0.5  # fallback default

        depth = (AVERAGE_IPD_METERS * FX) / pixel_ipd
        return max(0.1, min(depth, 2.0))  # clamp to reasonable range

    def _publish_no_face(self):
        """Publish default values when no face detected."""
        self.face_detected_pub.publish(Bool(data=False))
        self.mouth_open_pub.publish(Bool(data=False))
        self.mar_pub.publish(Float64(data=0.0))
        self.expression_pub.publish(String(data='no_face'))
        self.mouth_ready_pub.publish(Bool(data=False))

    def destroy_node(self):
        self.face_mesh.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
