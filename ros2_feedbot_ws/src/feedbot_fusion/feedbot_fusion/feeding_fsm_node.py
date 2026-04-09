"""
Feeding Finite State Machine (FSM) Node

Coordinates the full autonomous feeding sequence:

  WAITING -> IDLE -> DETECT_FOOD -> LOCATE_FOOD -> COLLECT_FOOD ->
  DETECT_PATIENT -> PRE_FEED -> FEED -> RETRACT -> WAITING

Autonomous mode: auto-starts when plate and food are detected.
Face tracking: uses MediaPipe face detection for patient detection,
mouth-open verification before feeding, and face-approach IK.
Manual mode: spacebar still works as fallback.

Uses EKF-fused 3D food position (camera + ultrasonic Kalman filter) from
the fusion_node for precise fork positioning.  The arm computes joint angles
via analytical inverse kinematics to reach the exact food location on the
plate, rather than using fixed hardcoded poses.

The fuzzy controller provides target_force, target_angle, and feeding_safe.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

import math


class FeedingState:
    WAITING = 'WAITING'
    IDLE = 'IDLE'
    DETECT_FOOD = 'DETECT_FOOD'
    LOCATE_FOOD = 'LOCATE_FOOD'
    COLLECT_FOOD = 'COLLECT_FOOD'
    DETECT_PATIENT = 'DETECT_PATIENT'
    PRE_FEED = 'PRE_FEED'
    FEED = 'FEED'
    RETRACT = 'RETRACT'


# Joint names matching the feedbot_description URDF
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# Robot link lengths (metres) — must match fusion_node
L1_HEIGHT = 0.042   # joint1→joint2 vertical offset
L2_LENGTH = 0.181   # link2 length
L3_LENGTH = 0.164   # link3 length

# Predefined joint poses (radians) for non-IK phases
# feedbot_description URDF: joint2/3/4 rotate around Y axis
# At joints=0: arm extends UP vertically then OUT horizontally
# Camera is on the feeder_head, facing along the feeder direction
# joint4 = -1.5 (~90° down) tilts the camera to look at the plate below
POSES = {
    'home':         [0.0,   0.0,   0.0,  -1.5],   # arm default, camera tilted down at plate
    'plate_above':  [0.0,   0.0,  -0.3,  -1.2],   # slight elbow bend, camera down
    'pre_feed':     [1.2,   0.0,   0.0,   0.0],   # rotate to patient, camera forward
    'feed':         [1.2,   0.0,  -0.3,   0.3],   # extend toward mouth
    'retract':      [0.6,   0.0,   0.0,   0.0],
}

# Joint limits (from URDF)
JOINT_LIMITS = {
    'joint1': (-3.05, 3.05),
    'joint2': (-1.57, 1.57),
    'joint3': (-2.09, 1.31),
    'joint4': (-1.57, 2.01),
}

POSITION_TOLERANCE = 0.08  # rad
STATE_TIMEOUT = 15.0
TRAJECTORY_DURATION_SEC = 2
FACE_APPROACH_DURATION_SEC = 4  # slower approach near face
CONFIDENCE_THRESHOLD = 0.3
PLATE_NEAR_CM = 15.0
MOUTH_NEAR_CM = 10.0

# Number of consecutive frames with stable 3D position before forking
LOCATE_STABLE_COUNT = 5
LOCATE_POSITION_TOLERANCE = 0.02  # metres — position must be stable within 2cm

# Face/mouth safety thresholds
MOUTH_OPEN_CONFIRM_FRAMES = 5   # consecutive open frames before feeding (0.5s at 10Hz)
FACE_LOST_RETRACT_FRAMES = 30   # retract if face lost for 3s (30 frames at 10Hz)
FACE_APPROACH_OFFSET = 0.05     # stop 5cm before face (metres)
FORCE_COLLISION_THRESHOLD = 5.0  # emergency retract if force exceeds this during approach


class FeedingFSMNode(Node):

    def __init__(self):
        super().__init__('feeding_fsm_node')

        self.state = FeedingState.WAITING
        self.state_start_time = self.get_clock().now()

        # Start signal — set by /feeding_start topic (spacebar press)
        self.start_requested = False

        # Sensor data
        self.current_positions = {name: 0.0 for name in JOINT_NAMES}
        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)
        self.feeding_safe = False
        self.mouth_ready = False
        self.predicted_state = [0.0] * 4
        self.current_force = 0.0
        self.override = False

        # EKF fusion outputs
        self.plate_distance = 50.0
        self.mouth_distance = 40.0
        self.fusion_confidence = 0.0
        self.sensor_health = [0.0] * 6

        # Fused 3D food position from camera+ultrasonic Kalman filter
        self.food_position_3d = (0.0, 0.0, 0.5)  # x, y, z in camera frame (m)
        self._food_pos_history = []  # for stability check

        # Face detection data (from face_node)
        self.face_detected = False
        self.face_bearing = (0.0, 0.0, 0.5)  # bearing_h, bearing_v, depth
        self.mouth_open = False
        self.face_expression = 'no_face'
        self.plate_detected = False

        # Face safety counters
        self.consecutive_mouth_open = 0
        self.face_lost_count = 0

        # Fuzzy controller outputs
        self.target_force = 0.0
        self.target_angle = 0.0

        # Target pose for current state
        self.target_pose = POSES['home']

        # IK-computed pickup pose (filled by LOCATE_FOOD state)
        self.pickup_pose = None

        # Feeding cycle counter
        self.feed_count = 0

        # ---- subscribers: raw sensors ----
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(
            Bool, '/food_visible', self.food_visible_cb, 10)
        self.create_subscription(
            Point, '/food_center', self.food_center_cb, 10)
        self.create_subscription(
            Float64, '/spoon_force', self.force_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/predicted_state', self.pred_cb, 10)
        self.create_subscription(
            Bool, '/mouth_ready_prediction', self.mouth_cb, 10)
        self.create_subscription(
            Bool, '/feeding_override', self.override_cb, 10)
        self.create_subscription(
            Bool, '/feeding_start', self.start_cb, 10)

        # ---- subscribers: face detection (from face_node) ----
        self.create_subscription(
            Bool, '/face_detected', self.face_detected_cb, 10)
        self.create_subscription(
            Point, '/face_bearing', self.face_bearing_cb, 10)
        self.create_subscription(
            Bool, '/mouth_open', self.mouth_open_cb, 10)
        self.create_subscription(
            String, '/face_expression', self.expression_cb, 10)
        self.create_subscription(
            Bool, '/plate_detected', self.plate_detected_cb, 10)

        # ---- subscribers: EKF fusion outputs ----
        self.create_subscription(
            Float64, '/plate_distance', self.plate_dist_cb, 10)
        self.create_subscription(
            Float64, '/mouth_distance', self.mouth_dist_cb, 10)
        self.create_subscription(
            Float64, '/fusion_confidence', self.confidence_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/sensor_health', self.health_cb, 10)
        self.create_subscription(
            Point, '/food_position_3d', self.food_pos_3d_cb, 10)

        # ---- subscribers: fuzzy controller outputs ----
        self.create_subscription(
            Bool, '/feeding_safe', self.safe_cb, 10)
        self.create_subscription(
            Float64, '/target_force', self.target_force_cb, 10)
        self.create_subscription(
            Float64, '/target_angle', self.target_angle_cb, 10)

        # ---- publishers ----
        # ForwardCommandController accepts Float64MultiArray on /arm_controller/commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)
        self.state_pub = self.create_publisher(String, '/feeding_state', 10)
        self.food_type_pub = self.create_publisher(String, '/food_type', 10)

        # 10 Hz FSM tick
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            'Feeding FSM started (EKF 3D food localisation + IK forking)')

    # ---- callbacks: raw sensors ----
    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos

    def food_visible_cb(self, msg):
        self.food_visible = msg.data

    def food_center_cb(self, msg):
        self.food_center = (msg.x, msg.y, msg.z)

    def safe_cb(self, msg):
        self.feeding_safe = msg.data

    def mouth_cb(self, msg):
        self.mouth_ready = msg.data

    def pred_cb(self, msg):
        self.predicted_state = list(msg.data)

    def force_cb(self, msg):
        self.current_force = msg.data

    def override_cb(self, msg):
        self.override = msg.data

    def start_cb(self, msg):
        if msg.data:
            self.start_requested = True
            self.get_logger().info('Start signal received (spacebar pressed)')

    # ---- callbacks: face detection ----
    def face_detected_cb(self, msg):
        self.face_detected = msg.data

    def face_bearing_cb(self, msg):
        self.face_bearing = (msg.x, msg.y, msg.z)

    def mouth_open_cb(self, msg):
        self.mouth_open = msg.data

    def expression_cb(self, msg):
        self.face_expression = msg.data

    def plate_detected_cb(self, msg):
        self.plate_detected = msg.data

    # ---- callbacks: EKF fusion ----
    def plate_dist_cb(self, msg):
        self.plate_distance = msg.data

    def mouth_dist_cb(self, msg):
        self.mouth_distance = msg.data

    def confidence_cb(self, msg):
        self.fusion_confidence = msg.data

    def health_cb(self, msg):
        self.sensor_health = list(msg.data)

    def food_pos_3d_cb(self, msg):
        self.food_position_3d = (msg.x, msg.y, msg.z)

    # ---- callbacks: fuzzy controller ----
    def target_force_cb(self, msg):
        self.target_force = msg.data

    def target_angle_cb(self, msg):
        self.target_angle = msg.data

    # ---- helpers ----
    def _set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(
                f'FSM: {self.state} -> {new_state} '
                f'[confidence={self.fusion_confidence:.2f}, '
                f'plate={self.plate_distance:.1f}cm, '
                f'mouth={self.mouth_distance:.1f}cm, '
                f'food_3d=({self.food_position_3d[0]:.3f}, '
                f'{self.food_position_3d[1]:.3f}, '
                f'{self.food_position_3d[2]:.3f})m]')
            self.state = new_state
            self.state_start_time = self.get_clock().now()

    def _state_elapsed(self):
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def _at_target(self):
        for i, jname in enumerate(JOINT_NAMES):
            if abs(self.current_positions[jname] - self.target_pose[i]) > POSITION_TOLERANCE:
                return False
        return True

    def _command_pose(self, pose, duration_sec=TRAJECTORY_DURATION_SEC):
        self.target_pose = pose
        msg = Float64MultiArray()
        msg.data = [float(p) for p in pose]
        self.cmd_pub.publish(msg)

    def _classify_food(self):
        area = self.food_center[2]
        food = 'apple' if area > 200 else 'strawberry'
        msg = String()
        msg.data = food
        self.food_type_pub.publish(msg)
        return food

    def _fusion_ok(self):
        return self.fusion_confidence >= CONFIDENCE_THRESHOLD

    # ---- Inverse Kinematics for food pickup ----
    def _compute_pickup_ik(self, food_x, food_y, food_z):
        """Compute joint angles to reach the fused 3D food position.

        The food position is in the camera frame (mounted on feeder_head).
        We transform it to the robot base frame, then solve the 4-DOF IK.

        Returns [j1, j2, j3, j4] or None if unreachable.
        """
        # The camera is on the feeder_head, which is at the end of the arm.
        # For the pickup approach, we need to position the feeder_head
        # above the food, so we compute base-frame coordinates.
        #
        # food_z is the forward distance from camera to food.
        # During DETECT_FOOD the arm is near home, so camera looks roughly
        # along the arm's forward direction.
        #
        # Approximate: food is at (food_z, food_x) in the arm's
        # horizontal plane (forward, lateral) from the camera position.

        # Current arm endpoint position (forward kinematics)
        j1_cur = self.current_positions.get('joint1', 0.0)
        j2_cur = self.current_positions.get('joint2', 0.0)
        j3_cur = self.current_positions.get('joint3', 0.0)

        # Arm tip position in base frame
        arm_x = L2_LENGTH * math.sin(j2_cur) + L3_LENGTH * math.sin(j2_cur + j3_cur)
        arm_z = L1_HEIGHT + L2_LENGTH * math.cos(j2_cur) + L3_LENGTH * math.cos(j2_cur + j3_cur)

        # Food position in base frame (approximate)
        # Camera is at arm tip, food_z is distance ahead, food_x is lateral
        target_r = arm_x + food_z   # radial distance from base
        target_z_height = arm_z - food_y  # vertical (food_y positive = below)

        # joint1: base rotation to align with food lateral offset
        j1 = j1_cur + math.atan2(food_x, max(food_z, 0.01))
        j1 = max(JOINT_LIMITS['joint1'][0], min(j1, JOINT_LIMITS['joint1'][1]))

        # 2-link IK in the vertical plane for joint2, joint3
        # Target: reach (target_r, target_z_height) relative to joint2 origin
        r = target_r
        z = target_z_height - L1_HEIGHT

        d_sq = r * r + z * z
        d = math.sqrt(d_sq)

        # Check reachability
        if d > (L2_LENGTH + L3_LENGTH) * 0.98 or d < abs(L2_LENGTH - L3_LENGTH) * 1.02:
            self.get_logger().warn(
                f'Food position unreachable: d={d:.3f}m '
                f'(range [{abs(L2_LENGTH - L3_LENGTH):.3f}, '
                f'{L2_LENGTH + L3_LENGTH:.3f}])')
            return None

        # Elbow angle (joint3) via cosine rule
        cos_j3 = (d_sq - L2_LENGTH**2 - L3_LENGTH**2) / (2 * L2_LENGTH * L3_LENGTH)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        j3 = -math.acos(cos_j3)  # elbow-down solution

        # Shoulder angle (joint2)
        alpha = math.atan2(r, z)
        beta = math.atan2(
            L3_LENGTH * math.sin(j3),
            L2_LENGTH + L3_LENGTH * math.cos(j3))
        j2 = alpha - beta

        # Clamp to joint limits
        j2 = max(JOINT_LIMITS['joint2'][0], min(j2, JOINT_LIMITS['joint2'][1]))
        j3 = max(JOINT_LIMITS['joint3'][0], min(j3, JOINT_LIMITS['joint3'][1]))

        # joint4: tilt feeder head downward to scoop food
        j4 = -0.3  # slight downward tilt for forking

        self.get_logger().info(
            f'IK solution: j1={math.degrees(j1):.1f}° j2={math.degrees(j2):.1f}° '
            f'j3={math.degrees(j3):.1f}° j4={math.degrees(j4):.1f}° '
            f'(target r={r:.3f}m z={z:.3f}m)')

        return [j1, j2, j3, j4]

    def _food_position_stable(self):
        """Check if the fused 3D food position has stabilised."""
        self._food_pos_history.append(self.food_position_3d)
        if len(self._food_pos_history) > LOCATE_STABLE_COUNT * 2:
            self._food_pos_history = self._food_pos_history[-LOCATE_STABLE_COUNT * 2:]

        if len(self._food_pos_history) < LOCATE_STABLE_COUNT:
            return False

        recent = self._food_pos_history[-LOCATE_STABLE_COUNT:]
        xs = [p[0] for p in recent]
        ys = [p[1] for p in recent]
        zs = [p[2] for p in recent]

        spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        return spread < LOCATE_POSITION_TOLERANCE

    # ---- Face approach IK ----
    def _compute_face_approach_ik(self, bearing_h, bearing_v, depth):
        """Compute joint angles to position the fork near the user's mouth.

        Uses face bearing angles and depth from face_node.
        Stops FACE_APPROACH_OFFSET metres before the face.
        Returns [j1, j2, j3, j4] or None if unreachable.
        """
        approach_depth = max(depth - FACE_APPROACH_OFFSET, 0.10)

        # j1: base rotation toward face
        j1 = bearing_h
        j1 = max(JOINT_LIMITS['joint1'][0], min(j1, JOINT_LIMITS['joint1'][1]))

        # Target position in arm plane
        target_x = approach_depth * math.tan(bearing_v)
        r = approach_depth
        z = L1_HEIGHT - target_x  # vertical from joint2

        d_sq = r * r + z * z
        d = math.sqrt(d_sq)

        if d > (L2_LENGTH + L3_LENGTH) * 0.98 or d < abs(L2_LENGTH - L3_LENGTH) * 1.02:
            return None

        cos_j3 = (d_sq - L2_LENGTH**2 - L3_LENGTH**2) / (2 * L2_LENGTH * L3_LENGTH)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        j3 = -math.acos(cos_j3)

        alpha = math.atan2(r, z)
        beta = math.atan2(
            L3_LENGTH * math.sin(j3),
            L2_LENGTH + L3_LENGTH * math.cos(j3))
        j2 = alpha - beta

        j2 = max(JOINT_LIMITS['joint2'][0], min(j2, JOINT_LIMITS['joint2'][1]))
        j3 = max(JOINT_LIMITS['joint3'][0], min(j3, JOINT_LIMITS['joint3'][1]))

        # Keep fork roughly horizontal for feeding
        j4 = 0.3

        return [j1, j2, j3, j4]

    # ---- FSM tick ----
    def tick(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        elapsed = self._state_elapsed()

        # ---- WAITING (auto-start on plate+food, or manual spacebar) ----
        if self.state == FeedingState.WAITING:
            self._command_pose(POSES['home'])
            if self.plate_detected and self.food_visible:
                self.get_logger().info(
                    'Plate and food detected — auto-starting feeding cycle')
                self._set_state(FeedingState.IDLE)
            elif self.start_requested:
                self.start_requested = False
                self.get_logger().info('Manual start (spacebar) — starting cycle')
                self._set_state(FeedingState.IDLE)

        # ---- IDLE ----
        elif self.state == FeedingState.IDLE:
            self._command_pose(POSES['home'])
            if self.food_visible:
                self._set_state(FeedingState.DETECT_FOOD)

        # ---- DETECT_FOOD ----
        elif self.state == FeedingState.DETECT_FOOD:
            if self.food_visible:
                food = self._classify_food()
                self.get_logger().info(
                    f'Food detected: {food} — entering LOCATE_FOOD '
                    f'for camera+ultrasonic 3D localisation')
                self._food_pos_history.clear()
                self._set_state(FeedingState.LOCATE_FOOD)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Food detection timed out')
                self._set_state(FeedingState.IDLE)

        # ---- LOCATE_FOOD (NEW: camera+ultrasonic fusion for precise positioning) ----
        elif self.state == FeedingState.LOCATE_FOOD:
            # Move to plate_above to get a good view
            self._command_pose(POSES['plate_above'])

            if self._fusion_ok() and self._food_position_stable():
                fx, fy, fz = self.food_position_3d
                self.get_logger().info(
                    f'Food localised via EKF (camera+ultrasonic): '
                    f'x={fx:.3f}m y={fy:.3f}m z={fz:.3f}m '
                    f'confidence={self.fusion_confidence:.2f}')

                # Compute IK for the fused food position
                self.pickup_pose = self._compute_pickup_ik(fx, fy, fz)
                if self.pickup_pose is not None:
                    self.get_logger().info(
                        f'IK pickup pose computed from fused position')
                else:
                    self.get_logger().warn(
                        'IK failed — falling back to predefined pose')

                self._set_state(FeedingState.COLLECT_FOOD)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn(
                    'Food localisation timed out — using predefined pose')
                self.pickup_pose = None
                self._set_state(FeedingState.COLLECT_FOOD)

        # ---- COLLECT_FOOD ----
        elif self.state == FeedingState.COLLECT_FOOD:
            if elapsed < 2.0:
                self._command_pose(POSES['plate_above'])
            else:
                # Use IK-computed pickup pose if available, else fallback
                if self.pickup_pose is not None:
                    self._command_pose(self.pickup_pose)
                else:
                    # Fallback to hardcoded plate_pickup
                    self._command_pose([0.0, -0.5, 0.6, -0.3])

            # Use fused force to detect food pickup
            if self.current_force > 2.0 and elapsed > 3.0:
                self.get_logger().info(
                    f'Food collected (force={self.current_force:.1f}N, '
                    f'plate_dist={self.plate_distance:.1f}cm, '
                    f'food_3d=({self.food_position_3d[0]:.3f}, '
                    f'{self.food_position_3d[1]:.3f}, '
                    f'{self.food_position_3d[2]:.3f})m, '
                    f'confidence={self.fusion_confidence:.2f})')
                self._set_state(FeedingState.DETECT_PATIENT)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Food collection timed out')
                self._set_state(FeedingState.RETRACT)

        # ---- DETECT_PATIENT (face detection via camera) ----
        elif self.state == FeedingState.DETECT_PATIENT:
            # Rotate toward expected patient position to scan for face
            self._command_pose(POSES['pre_feed'])

            if self.face_detected and self.face_expression == 'ready':
                self.get_logger().info(
                    f'Patient detected: bearing=('
                    f'{math.degrees(self.face_bearing[0]):.1f}°, '
                    f'{math.degrees(self.face_bearing[1]):.1f}°), '
                    f'depth={self.face_bearing[2]:.2f}m')
                self.face_lost_count = 0
                self._set_state(FeedingState.PRE_FEED)
            elif self.override:
                self._set_state(FeedingState.PRE_FEED)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Patient detection timed out — no face found')
                self._set_state(FeedingState.RETRACT)

        # ---- PRE_FEED (dynamic face-approach IK) ----
        elif self.state == FeedingState.PRE_FEED:
            # Force collision safety during approach
            if self.current_force > FORCE_COLLISION_THRESHOLD:
                self.get_logger().warn(
                    f'High force during approach ({self.current_force:.1f}N) '
                    f'— emergency retract')
                self._set_state(FeedingState.RETRACT)
                return

            if self.face_detected:
                self.face_lost_count = 0
                # Compute IK to approach the face
                approach_pose = self._compute_face_approach_ik(
                    self.face_bearing[0], self.face_bearing[1],
                    self.face_bearing[2])
                if approach_pose is not None:
                    self._command_pose(approach_pose,
                                       duration_sec=FACE_APPROACH_DURATION_SEC)
                else:
                    self._command_pose(POSES['pre_feed'])

                if self._at_target() or elapsed > 5.0:
                    if self.feeding_safe or self.override:
                        self.get_logger().info(
                            f'Pre-feed OK — approaching face at '
                            f'{self.face_bearing[2]:.2f}m')
                        self.consecutive_mouth_open = 0
                        self._set_state(FeedingState.FEED)
            else:
                self.face_lost_count += 1
                if self.face_lost_count > FACE_LOST_RETRACT_FRAMES:
                    self.get_logger().warn('Face lost during PRE_FEED — retracting')
                    self._set_state(FeedingState.RETRACT)

            if elapsed > STATE_TIMEOUT:
                self._set_state(FeedingState.RETRACT)

        # ---- FEED (wait for mouth open, then deliver) ----
        elif self.state == FeedingState.FEED:
            # Force collision safety
            if self.current_force > FORCE_COLLISION_THRESHOLD:
                self.get_logger().warn(
                    f'High force during feed ({self.current_force:.1f}N) '
                    f'— emergency retract')
                self._set_state(FeedingState.RETRACT)
                return

            # Face lost safety
            if not self.face_detected:
                self.face_lost_count += 1
                if self.face_lost_count > FACE_LOST_RETRACT_FRAMES // 2:
                    self.get_logger().warn('Face lost during FEED — retreating')
                    self._set_state(FeedingState.PRE_FEED)
                return
            self.face_lost_count = 0

            # Track mouth open/closed
            if self.mouth_open:
                self.consecutive_mouth_open += 1
            else:
                self.consecutive_mouth_open = 0

            # Require N consecutive mouth-open frames for safety
            if self.consecutive_mouth_open >= MOUTH_OPEN_CONFIRM_FRAMES or self.override:
                self._command_pose(POSES['feed'])
                if self._at_target() or elapsed > 5.0:
                    self.feed_count += 1
                    self.get_logger().info(
                        f'Feed #{self.feed_count} delivered '
                        f'(mouth_dist={self.mouth_distance:.1f}cm, '
                        f'force={self.current_force:.1f}N)')
                    self.consecutive_mouth_open = 0
                    self._set_state(FeedingState.RETRACT)
            elif elapsed % 2.0 < 0.15:
                self.get_logger().info(
                    f'Waiting for mouth open '
                    f'(consecutive={self.consecutive_mouth_open}/'
                    f'{MOUTH_OPEN_CONFIRM_FRAMES})')

        # ---- RETRACT (auto-restart) ----
        elif self.state == FeedingState.RETRACT:
            self._command_pose(POSES['retract'])
            if self._at_target() or elapsed > 5.0:
                self.get_logger().info(
                    f'Feeding cycle #{self.feed_count} complete — '
                    f'returning to WAITING (auto-restart when plate+food visible)')
                self._set_state(FeedingState.WAITING)


def main(args=None):
    rclpy.init(args=args)
    node = FeedingFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
