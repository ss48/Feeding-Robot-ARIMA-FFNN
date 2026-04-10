"""
Sensor Fusion Node — EKF-based multi-sensor fusion with 3D food localisation

Fuses vision (camera bearing + monocular depth), ultrasonic range, force,
joint states, and ARIMA-FFNN predictions using an Extended Kalman Filter
with confidence weighting, outlier rejection, and sensor health monitoring.

The key innovation is camera+ultrasonic fusion for precise food positioning:
  - Camera provides bearing angles (horizontal, vertical) to food
  - Ultrasonic provides range measurement to nearest object
  - Monocular depth provides a second depth estimate from apparent food size
  - EKF fuses these into a stable 3D food position in the robot frame

State vector  x = [food_x, food_y, food_z, plate_distance, mouth_distance, force]
  food_x:         lateral offset of food from camera centre (metres)
  food_y:         vertical offset of food from camera centre (metres)
  food_z:         depth/distance to food along camera axis (metres)
  plate_distance: fused distance to plate surface (cm)
  mouth_distance: fused distance to patient mouth (cm)
  force:          contact force on spoon (N)

Measurement sources:
  - Camera bearing (/food_bearing)            → food_x, food_y, food_z
  - Ultrasonic range (/sonar_plate_distance)  → food_z, plate_distance
  - Monocular depth (from camera)             → food_z (redundant)
  - Vision area (/food_center)                → plate_distance
  - Force (/spoon_force)                      → force
  - Joints (/joint_states)                    → mouth_distance
  - ARIMA (/predicted_state)                  → process model assist
  - Sonar mouth (/sonar_mouth_distance)       → mouth_distance

Publishes:
  /food_error_x       (Float64) — visual servoing error (normalised)
  /plate_distance     (Float64) — EKF-fused distance to plate (cm)
  /mouth_distance     (Float64) — EKF-fused distance to mouth (cm)
  /fusion_confidence  (Float64) — overall fusion confidence [0-1]
  /sensor_health      (Float64MultiArray) — per-sensor health
                        [vision, force, joints, arima, sonar, bearing]
  /food_position_3d   (Point)   — fused 3D food position in camera frame (m)
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Float64MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
IMAGE_CENTER_X = 320.0
AREA_TO_DISTANCE_K = 5000.0

# EKF dimensions
STATE_DIM = 6   # [food_x, food_y, food_z, plate_dist, mouth_dist, force]

# Robot link lengths (metres) — from feeding_robot_core.xacro
L1_HEIGHT = 0.043   # base_y to shoulder (0.131 - 0.088)
L2_LENGTH = 0.200   # vertical arm: shoulder to elbow (0.331 - 0.131)
L3_LENGTH = 0.199   # horizontal arm: elbow to wrist (upper_arm_length)

# Sensor timeout thresholds (seconds)
VISION_TIMEOUT = 1.0
FORCE_TIMEOUT = 1.0
JOINT_TIMEOUT = 1.0
ARIMA_TIMEOUT = 2.0
SONAR_TIMEOUT = 1.0
BEARING_TIMEOUT = 1.0

# Mahalanobis distance gate for outlier rejection
OUTLIER_GATE = 5.0   # chi-squared 95% with 1 DOF ≈ 3.84; use 5 for margin

# Exponential moving average alpha for temporal smoothing
EMA_ALPHA = 0.3


# ===================================================================
# Extended Kalman Filter — 6-state with camera+ultrasonic fusion
# ===================================================================
class EKF:
    """6-state Extended Kalman Filter for multi-sensor food localisation.

    State: [food_x, food_y, food_z, plate_dist, mouth_dist, force]
    """

    def __init__(self):
        # State: food_x(m), food_y(m), food_z(m), plate_dist(cm), mouth_dist(cm), force(N)
        self.x = np.array([0.0, 0.0, 0.5, 50.0, 40.0, 0.0])

        # State covariance — start with high uncertainty
        self.P = np.diag([0.1, 0.1, 0.5, 100.0, 100.0, 1.0])

        # Process noise (how much the state can change per step)
        self.Q = np.diag([0.001, 0.001, 0.005, 1.0, 0.5, 0.05])

        # Measurement noise per channel (default; overridden by confidence)
        self.R_base = np.diag([0.01, 0.01, 0.05, 25.0, 10.0, 0.1])

    def predict(self, dt, arima_hint=None):
        """Predict step — constant-state model with optional ARIMA hint."""
        F = np.eye(STATE_DIM)

        # Optionally use ARIMA predicted joints to refine mouth_distance
        if arima_hint is not None:
            j2_pred, j3_pred = arima_hint
            reach = _forward_reach(j2_pred, j3_pred)
            predicted_mouth = max(5.0, 40.0 - reach * 100.0)
            self.x[4] += 0.2 * (predicted_mouth - self.x[4])

        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z, confidence, mask):
        """Update step with confidence-weighted measurement noise.

        Args:
            z:          np.array(STATE_DIM) measurement vector
            confidence: np.array(STATE_DIM) per-channel confidence in [0, 1]
            mask:       np.array(STATE_DIM, bool) which channels have valid data
        """
        if not np.any(mask):
            return

        active = np.where(mask)[0]
        H = np.zeros((len(active), STATE_DIM))
        z_active = np.zeros(len(active))
        R_active = np.zeros((len(active), len(active)))

        for i, ch in enumerate(active):
            H[i, ch] = 1.0
            z_active[i] = z[ch]
            conf = np.clip(confidence[ch], 0.05, 1.0)
            R_active[i, i] = self.R_base[ch, ch] / conf

        # Innovation
        y = z_active - H @ self.x
        S = H @ self.P @ H.T + R_active

        # Mahalanobis outlier gating (per channel)
        keep = []
        for i in range(len(active)):
            d2 = y[i] ** 2 / S[i, i]
            if d2 <= OUTLIER_GATE:
                keep.append(i)

        if not keep:
            return

        H = H[keep]
        z_active = z_active[keep]
        R_active = R_active[np.ix_(keep, keep)]
        y = z_active - H @ self.x
        S = H @ self.P @ H.T + R_active

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ S_inv

        self.x = self.x + K @ y
        I = np.eye(STATE_DIM)
        self.P = (I - K @ H) @ self.P

        # Enforce constraints
        self.x[2] = max(self.x[2], 0.01)   # food_z > 0 (must be in front)
        self.x[3] = max(self.x[3], 1.0)    # plate_distance > 0
        self.x[4] = max(self.x[4], 1.0)    # mouth_distance > 0
        self.x[5] = max(self.x[5], 0.0)    # force >= 0


# ===================================================================
# Sensor Health Monitor
# ===================================================================
class SensorHealthMonitor:
    """Tracks freshness and consistency of each sensor stream."""

    def __init__(self, n_sensors=6):
        self.n = n_sensors
        self.last_update = [0.0] * n_sensors
        self.timeouts = [VISION_TIMEOUT, FORCE_TIMEOUT, JOINT_TIMEOUT,
                         ARIMA_TIMEOUT, SONAR_TIMEOUT, BEARING_TIMEOUT]
        self.consistency_buf = [[] for _ in range(n_sensors)]
        self.buf_size = 20

    def touch(self, idx, value, now):
        self.last_update[idx] = now
        buf = self.consistency_buf[idx]
        buf.append(value)
        if len(buf) > self.buf_size:
            buf.pop(0)

    def health(self, now):
        h = np.zeros(self.n)
        for i in range(self.n):
            age = now - self.last_update[i]
            freshness = max(0.0, 1.0 - age / self.timeouts[i])
            buf = self.consistency_buf[i]
            if len(buf) >= 3:
                std = float(np.std(buf))
                consistency = 1.0 / (1.0 + std)
            else:
                consistency = 0.5
            h[i] = freshness * (0.6 + 0.4 * consistency)
        return h


# ===================================================================
# Helper: forward kinematics reach
# ===================================================================
def _forward_reach(j2, j3):
    """Approximate forward reach of the arm tip in metres."""
    x = L2_LENGTH * math.sin(j2) + L3_LENGTH * math.sin(j2 + j3)
    z = L2_LENGTH * math.cos(j2) + L3_LENGTH * math.cos(j2 + j3)
    return math.sqrt(x ** 2 + z ** 2)


# ===================================================================
# ROS 2 Node
# ===================================================================
class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        # ----- Core components -----
        self.ekf = EKF()
        # 6 sensors: vision, force, joints, arima, sonar, bearing
        self.health_mon = SensorHealthMonitor(n_sensors=6)

        # ----- Smoothed outputs (EMA) -----
        self.ema_food_error = 0.0
        self.ema_plate_dist = 50.0
        self.ema_mouth_dist = 40.0
        self.ema_food_x = 0.0
        self.ema_food_y = 0.0
        self.ema_food_z = 0.5

        # ----- Raw sensor state -----
        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)  # x, y, area
        self.food_bearing = (0.0, 0.0, 0.5)  # bearing_h, bearing_v, mono_depth
        self.force = 0.0
        self.joint_positions = {}
        self.predicted_state = [0.0] * 4
        self.prediction_error = 0.0
        self.sonar_plate_dist = None
        self.sonar_mouth_dist = None

        self._last_fuse_time = time.monotonic()

        # ----- Subscribers -----
        self.create_subscription(
            Bool, '/food_visible', self._cb_food_visible, 10)
        self.create_subscription(
            Point, '/food_center', self._cb_food_center, 10)
        self.create_subscription(
            Point, '/food_bearing', self._cb_food_bearing, 10)
        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 10)
        self.create_subscription(
            Float64, '/spoon_force', self._cb_force, 10)
        self.create_subscription(
            Float64MultiArray, '/predicted_state', self._cb_pred, 10)
        self.create_subscription(
            Float64, '/prediction_error', self._cb_pred_err, 10)
        self.create_subscription(
            Float64, '/sonar_plate_distance', self._cb_sonar_plate, 10)
        self.create_subscription(
            Float64, '/sonar_mouth_distance', self._cb_sonar_mouth, 10)

        # ----- Publishers -----
        self.error_pub = self.create_publisher(Float64, '/food_error_x', 10)
        self.plate_dist_pub = self.create_publisher(
            Float64, '/plate_distance', 10)
        self.mouth_dist_pub = self.create_publisher(
            Float64, '/mouth_distance', 10)
        self.confidence_pub = self.create_publisher(
            Float64, '/fusion_confidence', 10)
        self.health_pub = self.create_publisher(
            Float64MultiArray, '/sensor_health', 10)
        # NEW: fused 3D food position in camera frame
        self.food_pos_pub = self.create_publisher(
            Point, '/food_position_3d', 10)
        # Face position pass-through (EMA-smoothed from face_node bearing)
        self.face_pos_pub = self.create_publisher(
            Point, '/face_position_3d', 10)

        # Face detection data
        self.face_bearing = None
        self.face_detected = False
        self._face_ema = [0.0, 0.0, 0.5]
        self.create_subscription(
            Point, '/face_bearing', self._cb_face_bearing, 10)
        self.create_subscription(
            Bool, '/face_detected', self._cb_face_detected, 10)

        # ----- Fusion at 10 Hz -----
        self.timer = self.create_timer(0.1, self._fuse_and_publish)

        self.get_logger().info(
            "Fusion node started (6-state EKF: camera+ultrasonic 3D food "
            "localisation + confidence weighting + outlier rejection)")

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------
    def _cb_food_visible(self, msg):
        self.food_visible = msg.data

    def _cb_food_center(self, msg):
        self.food_center = (msg.x, msg.y, msg.z)
        now = time.monotonic()
        area = msg.z
        dist = AREA_TO_DISTANCE_K / math.sqrt(area) if area > 1.0 else 50.0
        self.health_mon.touch(0, dist, now)

    def _cb_food_bearing(self, msg):
        self.food_bearing = (msg.x, msg.y, msg.z)
        self.health_mon.touch(5, msg.z, time.monotonic())

    def _cb_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos
        now = time.monotonic()
        j2 = self.joint_positions.get('joint2', 0.0)
        reach = _forward_reach(j2, self.joint_positions.get('joint3', 0.0))
        self.health_mon.touch(2, reach, now)

    def _cb_force(self, msg):
        self.force = msg.data
        self.health_mon.touch(1, msg.data, time.monotonic())

    def _cb_pred(self, msg):
        self.predicted_state = list(msg.data)
        now = time.monotonic()
        self.health_mon.touch(3, msg.data[0] if msg.data else 0.0, now)

    def _cb_pred_err(self, msg):
        self.prediction_error = msg.data

    def _cb_sonar_plate(self, msg):
        self.sonar_plate_dist = msg.data
        self.health_mon.touch(4, msg.data, time.monotonic())

    def _cb_sonar_mouth(self, msg):
        self.sonar_mouth_dist = msg.data
        self.health_mon.touch(4, msg.data, time.monotonic())

    def _cb_face_bearing(self, msg):
        self.face_bearing = (msg.x, msg.y, msg.z)

    def _cb_face_detected(self, msg):
        self.face_detected = msg.data

    # ----------------------------------------------------------------
    # Main fusion loop
    # ----------------------------------------------------------------
    def _fuse_and_publish(self):
        now = time.monotonic()
        dt = now - self._last_fuse_time
        self._last_fuse_time = now

        # health indices: 0=vision, 1=force, 2=joints, 3=arima, 4=sonar, 5=bearing
        health = self.health_mon.health(now)

        # ---- Build measurement vector ----
        # State: [food_x, food_y, food_z, plate_dist, mouth_dist, force]
        z = np.zeros(STATE_DIM)
        confidence = np.zeros(STATE_DIM)
        mask = np.zeros(STATE_DIM, dtype=bool)

        # --- Camera bearing → food_x, food_y, food_z ---
        # Camera+ultrasonic fusion: bearing gives direction, depth from
        # monocular estimate (fused with ultrasonic below)
        if self.food_visible and health[5] > 0.1:
            bearing_h, bearing_v, mono_depth = self.food_bearing
            # Convert bearing + depth to 3D position in camera frame
            # food_x = depth * tan(bearing_h)  (lateral offset)
            # food_y = depth * tan(bearing_v)  (vertical offset)
            # food_z = depth                    (forward distance)
            z[0] = mono_depth * math.tan(bearing_h)
            z[1] = mono_depth * math.tan(bearing_v)
            z[2] = mono_depth

            bearing_conf = health[5]
            confidence[0] = bearing_conf * 0.8   # lateral has good accuracy
            confidence[1] = bearing_conf * 0.8   # vertical has good accuracy
            confidence[2] = bearing_conf * 0.4   # monocular depth is less reliable
            mask[0] = True
            mask[1] = True
            mask[2] = True

        # --- Vision area → plate_distance ---
        if self.food_visible and self.food_center[2] > 1.0:
            area = self.food_center[2]
            z[3] = AREA_TO_DISTANCE_K / math.sqrt(area)
            area_conf = min(1.0, area / 500.0)
            confidence[3] = health[0] * area_conf
            mask[3] = True

        # --- Joints → mouth_distance ---
        j2 = self.joint_positions.get('joint2', None)
        j3 = self.joint_positions.get('joint3', None)
        if j2 is not None and j3 is not None:
            reach_m = _forward_reach(j2, j3)
            z[4] = max(5.0, 40.0 - reach_m * 100.0)
            confidence[4] = health[2]
            mask[4] = True

        # --- Force ---
        if health[1] > 0.1:
            z[5] = self.force
            confidence[5] = health[1]
            mask[5] = True

        # ---- EKF predict ----
        arima_hint = None
        if health[3] > 0.3 and len(self.predicted_state) >= 4:
            arima_hint = (self.predicted_state[1], self.predicted_state[2])
        self.ekf.predict(dt, arima_hint=arima_hint)

        # ---- EKF update 1: camera + force + joints ----
        self.ekf.update(z, confidence, mask)

        # ---- EKF update 2: ultrasonic (independent redundant measurement) ----
        # Ultrasonic range fuses with food_z (depth) and plate_distance
        z_sonar = np.zeros(STATE_DIM)
        conf_sonar = np.zeros(STATE_DIM)
        mask_sonar = np.zeros(STATE_DIM, dtype=bool)

        if self.sonar_plate_dist is not None and health[4] > 0.1:
            sonar_range_m = self.sonar_plate_dist / 100.0  # cm → m

            # Ultrasonic range → food_z (depth in metres)
            # This is the key camera+ultrasonic fusion: ultrasonic provides
            # a more reliable depth than monocular estimation
            z_sonar[2] = sonar_range_m
            conf_sonar[2] = health[4] * 0.9  # ultrasonic depth is very reliable
            mask_sonar[2] = True

            # Also update plate_distance (in cm)
            z_sonar[3] = self.sonar_plate_dist
            conf_sonar[3] = health[4]
            mask_sonar[3] = True

        if self.sonar_mouth_dist is not None and health[4] > 0.1:
            z_sonar[4] = self.sonar_mouth_dist
            conf_sonar[4] = health[4]
            mask_sonar[4] = True

        if np.any(mask_sonar):
            self.ekf.update(z_sonar, conf_sonar, mask_sonar)

        # ---- EKF update 3: refine food_x, food_y using fused depth ----
        # Now that food_z is fused (camera+ultrasonic), recompute food_x/y
        # using bearing angles with the improved depth estimate
        if self.food_visible and health[5] > 0.1:
            bearing_h, bearing_v, _ = self.food_bearing
            fused_depth = self.ekf.x[2]  # use EKF-fused depth
            z_refine = np.zeros(STATE_DIM)
            conf_refine = np.zeros(STATE_DIM)
            mask_refine = np.zeros(STATE_DIM, dtype=bool)

            z_refine[0] = fused_depth * math.tan(bearing_h)
            z_refine[1] = fused_depth * math.tan(bearing_v)
            conf_refine[0] = health[5] * 0.9
            conf_refine[1] = health[5] * 0.9
            mask_refine[0] = True
            mask_refine[1] = True

            self.ekf.update(z_refine, conf_refine, mask_refine)

        # ---- Temporal smoothing (EMA) ----
        self.ema_food_x = _ema(self.ema_food_x, self.ekf.x[0], EMA_ALPHA)
        self.ema_food_y = _ema(self.ema_food_y, self.ekf.x[1], EMA_ALPHA)
        self.ema_food_z = _ema(self.ema_food_z, self.ekf.x[2], EMA_ALPHA)
        self.ema_food_error = _ema(
            self.ema_food_error,
            self.ekf.x[0] / max(self.ekf.x[2], 0.01),  # normalised lateral error
            EMA_ALPHA)
        self.ema_plate_dist = _ema(
            self.ema_plate_dist, self.ekf.x[3], EMA_ALPHA)
        self.ema_mouth_dist = _ema(
            self.ema_mouth_dist, self.ekf.x[4], EMA_ALPHA)

        # ---- Publish fused outputs ----
        self.error_pub.publish(Float64(data=float(self.ema_food_error)))
        self.plate_dist_pub.publish(
            Float64(data=float(self.ema_plate_dist)))
        self.mouth_dist_pub.publish(
            Float64(data=float(self.ema_mouth_dist)))

        # 3D food position (camera frame, metres)
        food_pos = Point()
        food_pos.x = float(self.ema_food_x)
        food_pos.y = float(self.ema_food_y)
        food_pos.z = float(self.ema_food_z)
        self.food_pos_pub.publish(food_pos)

        # Overall confidence
        active_health = health[mask] if np.any(mask) else health
        overall = float(np.mean(active_health))
        self.confidence_pub.publish(Float64(data=overall))

        # Sensor health array [vision, force, joints, arima, sonar, bearing]
        health_msg = Float64MultiArray()
        health_msg.data = health.tolist()
        self.health_pub.publish(health_msg)

        # Face position pass-through (EMA-smoothed)
        if self.face_detected and self.face_bearing is not None:
            import math as _math
            bh, bv, depth = self.face_bearing
            face_x = depth * _math.tan(bh)
            face_y = depth * _math.tan(bv)
            face_z = depth
            self._face_ema[0] = _ema(self._face_ema[0], face_x, self.EMA_ALPHA)
            self._face_ema[1] = _ema(self._face_ema[1], face_y, self.EMA_ALPHA)
            self._face_ema[2] = _ema(self._face_ema[2], face_z, self.EMA_ALPHA)
            face_pos = Point()
            face_pos.x = float(self._face_ema[0])
            face_pos.y = float(self._face_ema[1])
            face_pos.z = float(self._face_ema[2])
            self.face_pos_pub.publish(face_pos)

            # Update mouth_distance from face depth
            face_dist_cm = depth * 100.0
            mouth_out = _ema(self.ema_mouth_dist, face_dist_cm, self.EMA_ALPHA)
            self.ema_mouth_dist = mouth_out
            self.mouth_dist_pub.publish(Float64(data=mouth_out))


# ===================================================================
# Utility
# ===================================================================
def _ema(prev, new, alpha):
    return alpha * new + (1.0 - alpha) * prev


# ===================================================================
# Entry point
# ===================================================================
def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
