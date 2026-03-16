"""
Sensor Fusion Node — EKF-based multi-sensor fusion

Fuses vision, force, joint state, and ARIMA-FFNN prediction data using an
Extended Kalman Filter with confidence weighting, outlier rejection, and
sensor health monitoring.

State vector  x = [food_error_x, plate_distance, mouth_distance, force]
Measurement sources:
  - Vision  (food_center, food_visible)  → food_error_x, plate_distance
  - Force   (spoon_force)                → force
  - Joints  (joint_states)               → mouth_distance
  - ARIMA   (predicted_state)            → used as process model assist

Publishes:
  /food_error_x    (Float64) — visual servoing error
  /plate_distance  (Float64) — EKF-fused distance to plate (cm)
  /mouth_distance  (Float64) — EKF-fused distance to mouth (cm)
  /fusion_confidence (Float64) — overall fusion confidence [0-1]
  /sensor_health     (Float64MultiArray) — per-sensor health [vision, force, joints, arima]
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
STATE_DIM = 4   # [food_error_x, plate_distance, mouth_distance, force]
MEAS_DIM = 4    # same layout — each sensor contributes one element

# Robot link lengths (metres) for forward kinematics
L1_HEIGHT = 0.042   # joint1→joint2 vertical offset
L2_LENGTH = 0.181   # link2 length (joint2→joint3)
L3_LENGTH = 0.164   # link3 length (joint3→joint4/feeder_head)

# Sensor timeout thresholds (seconds)
VISION_TIMEOUT = 1.0
FORCE_TIMEOUT = 1.0
JOINT_TIMEOUT = 1.0
ARIMA_TIMEOUT = 2.0

# Mahalanobis distance gate for outlier rejection
OUTLIER_GATE = 5.0   # chi-squared 95% with 1 DOF ≈ 3.84; use 5 for margin

# Exponential moving average alpha for temporal smoothing
EMA_ALPHA = 0.3


# ===================================================================
# Extended Kalman Filter
# ===================================================================
class EKF:
    """Lightweight 4-state Extended Kalman Filter."""

    def __init__(self):
        # State: [food_error_x, plate_dist, mouth_dist, force]
        self.x = np.array([0.0, 50.0, 40.0, 0.0])

        # State covariance — start with high uncertainty
        self.P = np.diag([1.0, 100.0, 100.0, 1.0])

        # Process noise (how much the state can change per step)
        self.Q = np.diag([0.01, 1.0, 0.5, 0.05])

        # Measurement noise per sensor (default; overridden by confidence)
        self.R_base = np.diag([0.05, 25.0, 10.0, 0.1])

    def predict(self, dt, arima_hint=None):
        """Predict step — simple constant-state model.

        If an ARIMA hint is available it nudges the mouth_distance
        prediction via the predicted joint extension.
        """
        # F = identity (constant model)
        F = np.eye(STATE_DIM)

        # Optionally use ARIMA predicted joints to refine mouth_distance
        if arima_hint is not None:
            j2_pred, j3_pred = arima_hint
            reach = _forward_reach(j2_pred, j3_pred)
            predicted_mouth = max(5.0, 40.0 - reach * 100.0)
            # Blend: state drifts towards ARIMA prediction
            self.x[2] += 0.2 * (predicted_mouth - self.x[2])

        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z, confidence, mask):
        """Update step with confidence-weighted measurement noise and
        per-channel enable mask.

        Args:
            z:          np.array(4) measurement vector
            confidence: np.array(4) per-sensor confidence in [0, 1]
            mask:       np.array(4, bool) which channels have valid data
        """
        if not np.any(mask):
            return

        # Build observation matrix H and vectors for active channels only
        active = np.where(mask)[0]
        H = np.zeros((len(active), STATE_DIM))
        z_active = np.zeros(len(active))
        R_active = np.zeros((len(active), len(active)))

        for i, ch in enumerate(active):
            H[i, ch] = 1.0
            z_active[i] = z[ch]
            # Scale measurement noise inversely with confidence
            conf = np.clip(confidence[ch], 0.05, 1.0)
            R_active[i, i] = self.R_base[ch, ch] / conf

        # Innovation
        y = z_active - H @ self.x
        S = H @ self.P @ H.T + R_active

        # --- Mahalanobis outlier gating (per channel) ---
        keep = []
        for i in range(len(active)):
            d2 = y[i] ** 2 / S[i, i]
            if d2 <= OUTLIER_GATE:
                keep.append(i)

        if not keep:
            return

        # Rebuild with only inlier channels
        H = H[keep]
        z_active = z_active[keep]
        R_active = R_active[np.ix_(keep, keep)]
        y = z_active - H @ self.x
        S = H @ self.P @ H.T + R_active

        # Kalman gain
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ S_inv

        # State and covariance update
        self.x = self.x + K @ y
        I = np.eye(STATE_DIM)
        self.P = (I - K @ H) @ self.P

        # Enforce non-negative distances
        self.x[1] = max(self.x[1], 1.0)
        self.x[2] = max(self.x[2], 1.0)
        self.x[3] = max(self.x[3], 0.0)


# ===================================================================
# Sensor Health Monitor
# ===================================================================
class SensorHealthMonitor:
    """Tracks freshness and consistency of each sensor stream."""

    def __init__(self, n_sensors=4):
        self.n = n_sensors
        self.last_update = [0.0] * n_sensors
        self.timeouts = [VISION_TIMEOUT, FORCE_TIMEOUT,
                         JOINT_TIMEOUT, ARIMA_TIMEOUT]
        self.consistency_buf = [[] for _ in range(n_sensors)]
        self.buf_size = 20

    def touch(self, idx, value, now):
        """Record a new reading for sensor *idx*."""
        self.last_update[idx] = now
        buf = self.consistency_buf[idx]
        buf.append(value)
        if len(buf) > self.buf_size:
            buf.pop(0)

    def health(self, now):
        """Return np.array(n) of health scores in [0, 1]."""
        h = np.zeros(self.n)
        for i in range(self.n):
            # Freshness component (1 if recent, decays to 0)
            age = now - self.last_update[i]
            freshness = max(0.0, 1.0 - age / self.timeouts[i])

            # Consistency component (low variance → high consistency)
            buf = self.consistency_buf[i]
            if len(buf) >= 3:
                std = float(np.std(buf))
                # Normalise: std < 1 → consistency ~ 1
                consistency = 1.0 / (1.0 + std)
            else:
                consistency = 0.5  # not enough data yet

            h[i] = freshness * (0.6 + 0.4 * consistency)
        return h


# ===================================================================
# Helper: forward kinematics reach
# ===================================================================
def _forward_reach(j2, j3):
    """Approximate forward reach of the arm tip in metres,
    given joint2 and joint3 angles (radians)."""
    # Planar 2-link approximation in the vertical plane
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
        self.health_mon = SensorHealthMonitor(n_sensors=4)

        # ----- Smoothed outputs (EMA) -----
        self.ema_food_error = 0.0
        self.ema_plate_dist = 50.0
        self.ema_mouth_dist = 40.0

        # ----- Raw sensor state -----
        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)  # x, y, area
        self.force = 0.0
        self.joint_positions = {}
        self.predicted_state = [0.0] * 4
        self.prediction_error = 0.0

        self._last_fuse_time = time.monotonic()

        # ----- Subscribers -----
        self.create_subscription(
            Bool, '/food_visible', self._cb_food_visible, 10)
        self.create_subscription(
            Point, '/food_center', self._cb_food_center, 10)
        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 10)
        self.create_subscription(
            Float64, '/spoon_force', self._cb_force, 10)
        self.create_subscription(
            Float64MultiArray, '/predicted_state', self._cb_pred, 10)
        self.create_subscription(
            Float64, '/prediction_error', self._cb_pred_err, 10)

        # ----- Publishers (same API as before + new diagnostics) -----
        self.error_pub = self.create_publisher(Float64, '/food_error_x', 10)
        self.plate_dist_pub = self.create_publisher(Float64, '/plate_distance', 10)
        self.mouth_dist_pub = self.create_publisher(Float64, '/mouth_distance', 10)
        self.confidence_pub = self.create_publisher(Float64, '/fusion_confidence', 10)
        self.health_pub = self.create_publisher(
            Float64MultiArray, '/sensor_health', 10)

        # ----- Fusion at 10 Hz -----
        self.timer = self.create_timer(0.1, self._fuse_and_publish)

        self.get_logger().info(
            "Fusion node started (EKF + confidence weighting + "
            "outlier rejection + sensor health)")

    # ----------------------------------------------------------------
    # Callbacks — store raw data and touch health monitor
    # ----------------------------------------------------------------
    def _cb_food_visible(self, msg):
        self.food_visible = msg.data

    def _cb_food_center(self, msg):
        self.food_center = (msg.x, msg.y, msg.z)
        now = time.monotonic()
        # Use the plate distance derived from area as the health value
        area = msg.z
        dist = AREA_TO_DISTANCE_K / math.sqrt(area) if area > 1.0 else 50.0
        self.health_mon.touch(0, dist, now)

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

    # ----------------------------------------------------------------
    # Main fusion loop
    # ----------------------------------------------------------------
    def _fuse_and_publish(self):
        now = time.monotonic()
        dt = now - self._last_fuse_time
        self._last_fuse_time = now

        # ---- 1. Sensor health ----
        health = self.health_mon.health(now)
        # health indices: 0=vision, 1=force, 2=joints, 3=arima

        # ---- 2. Build raw measurement vector ----
        z = np.zeros(MEAS_DIM)
        confidence = np.zeros(MEAS_DIM)
        mask = np.zeros(MEAS_DIM, dtype=bool)

        # --- Vision → food_error_x and plate_distance ---
        if self.food_visible and self.food_center[2] > 1.0:
            z[0] = (self.food_center[0] - IMAGE_CENTER_X) / IMAGE_CENTER_X
            z[1] = AREA_TO_DISTANCE_K / math.sqrt(self.food_center[2])
            # Confidence: higher area → more reliable detection
            area_conf = min(1.0, self.food_center[2] / 500.0)
            confidence[0] = health[0] * area_conf
            confidence[1] = health[0] * area_conf
            mask[0] = True
            mask[1] = True

        # --- Joints → mouth_distance ---
        j2 = self.joint_positions.get('joint2', None)
        j3 = self.joint_positions.get('joint3', None)
        if j2 is not None and j3 is not None:
            reach_m = _forward_reach(j2, j3)
            z[2] = max(5.0, 40.0 - reach_m * 100.0)
            confidence[2] = health[2]
            mask[2] = True

        # --- Force ---
        if health[1] > 0.1:
            z[3] = self.force
            confidence[3] = health[1]
            mask[3] = True

        # ---- 3. EKF predict ----
        arima_hint = None
        if health[3] > 0.3 and len(self.predicted_state) >= 4:
            # Use ARIMA predicted joint2/joint3 as process-model hint
            arima_hint = (self.predicted_state[1], self.predicted_state[2])
        self.ekf.predict(dt, arima_hint=arima_hint)

        # ---- 4. EKF update (with outlier gating) ----
        self.ekf.update(z, confidence, mask)

        # ---- 5. Temporal smoothing (EMA) ----
        self.ema_food_error = _ema(self.ema_food_error,
                                   self.ekf.x[0], EMA_ALPHA)
        self.ema_plate_dist = _ema(self.ema_plate_dist,
                                   self.ekf.x[1], EMA_ALPHA)
        self.ema_mouth_dist = _ema(self.ema_mouth_dist,
                                   self.ekf.x[2], EMA_ALPHA)

        # ---- 6. Publish fused outputs ----
        self.error_pub.publish(Float64(data=float(self.ema_food_error)))
        self.plate_dist_pub.publish(Float64(data=float(self.ema_plate_dist)))
        self.mouth_dist_pub.publish(Float64(data=float(self.ema_mouth_dist)))

        # Overall confidence = mean of active-channel health scores
        active_health = health[mask] if np.any(mask) else health
        overall = float(np.mean(active_health))
        self.confidence_pub.publish(Float64(data=overall))

        # Sensor health array [vision, force, joints, arima]
        health_msg = Float64MultiArray()
        health_msg.data = health.tolist()
        self.health_pub.publish(health_msg)


# ===================================================================
# Utility
# ===================================================================
def _ema(prev, new, alpha):
    """Exponential moving average."""
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
