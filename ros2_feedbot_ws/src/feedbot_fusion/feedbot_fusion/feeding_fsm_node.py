"""
Feeding Finite State Machine (FSM) Node

Coordinates the full autonomous feeding sequence:

  WAITING -> IDLE -> DETECT_FOOD -> LOCATE_FOOD -> COLLECT_FOOD ->
  DETECT_PATIENT -> PRE_FEED -> FEED -> RETRACT -> WAITING

Autonomous mode: auto-starts when plate and food are detected.
Face tracking: uses MediaPipe face detection for patient detection,
mouth-open verification before feeding, and face-approach IK.
Patient sits to the RIGHT of the robot.
Manual mode: spacebar still works as fallback.

Uses FollowJointTrajectory action client for smooth arm control.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64, Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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
    EMERGENCY_STOP = 'EMERGENCY_STOP'


# Joint names matching the feeding_robot URDF
JOINT_NAMES = ['base_y_joint', 'lower_z_joint', 'upper_z_joint', 'feeder_joint']

# Robot link lengths (metres) — from feeding_robot_core.xacro
# base_top_z=0.088, shoulder_joint_z=0.131, elbow_joint_z=0.331
L1_HEIGHT = 0.043    # base_y to shoulder (0.131 - 0.088)
L2_LENGTH = 0.200    # vertical arm: shoulder to elbow (0.331 - 0.131)
L3_LENGTH = 0.199    # horizontal arm: elbow to wrist (upper_arm_length)
FEEDER_LENGTH = 0.168  # feeder + fork tip (0.148 + 0.02)

# Predefined joint poses [base_y, lower_z, upper_z, feeder]
# Calibrated on real hardware 2026-04-10
# Patient sits to the LEFT of the robot (negative base_y rotation)
POSES = {
    'home':        [ 0.121, -0.325,  0.227, -1.448],  # camera sees plate from above
    'plate_above': [ 0.077,  0.529,  0.026, -1.643],  # fork over plate, ready to stab
    'pre_feed':    [-1.342,  0.652, -1.727, -1.083],  # rotated to patient, fork raised
    'feed':        [-1.465,  0.284, -1.115, -0.716],  # fork at patient's mouth
    'retract':     [-1.342,  0.163, -1.724, -1.083],  # pulling back from patient
}

# Joint limits (from URDF)
JOINT_LIMITS = {
    'base_y_joint':  (-6.28, 6.28),  # continuous (360°)
    'lower_z_joint': (-1.57, 1.57),
    'upper_z_joint': (-2.09, 1.31),
    'feeder_joint':  (-1.57, 2.01),
}

POSITION_TOLERANCE = 0.08  # rad
STATE_TIMEOUT = 15.0
TRAJECTORY_DURATION_SEC = 3
FACE_APPROACH_DURATION_SEC = 4
CONFIDENCE_THRESHOLD = 0.3

LOCATE_STABLE_COUNT = 5
LOCATE_POSITION_TOLERANCE = 0.02  # metres

MOUTH_OPEN_CONFIRM_FRAMES = 5
FACE_LOST_RETRACT_FRAMES = 30
FORCE_COLLISION_THRESHOLD = 5.0


class FeedingFSMNode(Node):

    def __init__(self):
        super().__init__('feeding_fsm_node')

        self.declare_parameter('use_arima_feedforward', False)
        self.use_arima_ff = self.get_parameter('use_arima_feedforward').value

        self.state = FeedingState.WAITING
        self.state_start_time = self.get_clock().now()

        self.start_requested = False

        # Sensor data
        self.current_positions = {name: 0.0 for name in JOINT_NAMES}
        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)
        self.feeding_safe = False
        self.predicted_state = [0.0] * 4
        self.current_force = 0.0
        self.override = False

        # EKF fusion outputs
        self.plate_distance = 50.0
        self.mouth_distance = 40.0
        self.fusion_confidence = 0.0
        self.sensor_health = [0.0] * 6
        self.food_position_3d = (0.0, 0.0, 0.5)
        self._food_pos_history = []

        # Face detection data
        self.face_detected = False
        self.face_bearing = (0.0, 0.0, 0.5)
        self.mouth_open = False
        self.face_expression = 'no_face'
        self.plate_detected = False
        self.plate_center = (320.0, 240.0)  # plate center in pixels
        self.plate_radius = 0.0             # plate radius in pixels
        self.fork_tip = (320.0, 400.0)      # fork tip position in pixels
        self.fork_tip_detected = False
        self.consecutive_mouth_open = 0
        self.face_lost_count = 0

        # Fuzzy controller outputs
        self.target_force = 0.0
        self.target_angle = 0.0

        # Target pose and IK
        self.target_pose = POSES['home']
        self.pickup_pose = None
        self.feed_count = 0

        # Emergency stop
        self.emergency_stop = False
        self._pre_estop_state = None

        # ---- Action client for arm controller ----
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        # ---- Subscribers: raw sensors ----
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Bool, '/food_visible', self.food_visible_cb, 10)
        self.create_subscription(Point, '/food_center', self.food_center_cb, 10)
        self.create_subscription(Float64, '/spoon_force', self.force_cb, 10)
        self.create_subscription(Float64MultiArray, '/predicted_state', self.pred_cb, 10)
        self.create_subscription(Bool, '/feeding_override', self.override_cb, 10)
        self.create_subscription(Bool, '/feeding_start', self.start_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self.estop_cb, 10)

        # ---- Subscribers: face detection ----
        self.create_subscription(Bool, '/face_detected', self.face_detected_cb, 10)
        self.create_subscription(Point, '/face_bearing', self.face_bearing_cb, 10)
        self.create_subscription(Bool, '/mouth_open', self.mouth_open_cb, 10)
        self.create_subscription(String, '/face_expression', self.expression_cb, 10)
        self.create_subscription(Bool, '/plate_detected', self.plate_detected_cb, 10)
        self.create_subscription(Point, '/plate_map', self.plate_map_cb, 10)
        self.create_subscription(Point, '/fork_tip_position', self.fork_tip_cb, 10)

        # ---- Subscribers: EKF fusion ----
        self.create_subscription(Float64, '/plate_distance', self.plate_dist_cb, 10)
        self.create_subscription(Float64, '/mouth_distance', self.mouth_dist_cb, 10)
        self.create_subscription(Float64, '/fusion_confidence', self.confidence_cb, 10)
        self.create_subscription(Float64MultiArray, '/sensor_health', self.health_cb, 10)
        self.create_subscription(Point, '/food_position_3d', self.food_pos_3d_cb, 10)

        # ---- Subscribers: fuzzy controller ----
        self.create_subscription(Bool, '/feeding_safe', self.safe_cb, 10)
        self.create_subscription(Float64, '/target_force', self.target_force_cb, 10)
        self.create_subscription(Float64, '/target_angle', self.target_angle_cb, 10)

        # ---- Publishers ----
        self.state_pub = self.create_publisher(String, '/feeding_state', 10)
        self.food_type_pub = self.create_publisher(String, '/food_type', 10)

        # Send home pose on startup after a short delay
        self._homed = False
        self.create_timer(5.0, self._go_home_once)

        # 10 Hz FSM tick
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            'Feeding FSM started (real hardware — FollowJointTrajectory action)')

    def _go_home_once(self):
        """Startup: lift arm up, then position for top-down plate view."""
        if not self._homed:
            self._homed = True
            # Step 1: Lift arm straight up from any position (clear plate)
            cur = [self.current_positions[j] for j in JOINT_NAMES]
            lift_pose = list(cur)
            lift_pose[1] = -0.5   # shoulder back (lift up)
            lift_pose[3] = -0.3   # feeder less tilted (fork clears plate)
            self.get_logger().info('Step 1: Lifting arm to clear plate...')
            self._command_pose(lift_pose, duration_sec=4)

            # Step 2: Move to top-down plate view
            self.create_timer(5.0, self._go_to_top_view)

    def _go_to_top_view(self):
        """Position arm above plate with camera looking straight down."""
        # Arm up tall, feeder pointing down → camera has top-down view of plate
        top_view_pose = list(POSES['plate_above'])
        # Adjust for clear top-down view: shoulder less forward, feeder straight down
        top_view_pose[1] = 0.2    # shoulder slightly forward (arm over plate)
        top_view_pose[3] = -1.5   # feeder pointing straight down (top-down camera view)
        self.get_logger().info('Step 2: Moving to top-down plate view...')
        self._command_pose(top_view_pose, duration_sec=4)

        # Step 3: Settle at home after scanning
        self.create_timer(6.0, self._go_home_after_scan)

    def _go_home_after_scan(self):
        """Move to home position after initial plate scan."""
        self.get_logger().info('Step 3: Moving to HOME...')
        self._command_pose(POSES['home'], duration_sec=4)

    # ---- Callbacks ----
    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos

    def food_visible_cb(self, msg): self.food_visible = msg.data
    def food_center_cb(self, msg): self.food_center = (msg.x, msg.y, msg.z)
    def safe_cb(self, msg): self.feeding_safe = msg.data
    def pred_cb(self, msg): self.predicted_state = list(msg.data)
    def force_cb(self, msg): self.current_force = msg.data
    def override_cb(self, msg): self.override = msg.data

    def start_cb(self, msg):
        if msg.data:
            self.start_requested = True
            self.get_logger().info('Start signal received (spacebar)')

    def estop_cb(self, msg):
        if msg.data and not self.emergency_stop:
            self.emergency_stop = True
            self.get_logger().warn('EMERGENCY STOP — freezing arm')
            hold_pose = [self.current_positions[j] for j in JOINT_NAMES]
            self._command_pose(hold_pose)
            self._set_state(FeedingState.EMERGENCY_STOP)
        elif not msg.data and self.emergency_stop:
            self.emergency_stop = False
            self.get_logger().info('E-Stop released — returning to WAITING')
            self._set_state(FeedingState.WAITING)

    def face_detected_cb(self, msg): self.face_detected = msg.data
    def face_bearing_cb(self, msg): self.face_bearing = (msg.x, msg.y, msg.z)
    def mouth_open_cb(self, msg): self.mouth_open = msg.data
    def expression_cb(self, msg): self.face_expression = msg.data
    def plate_detected_cb(self, msg): self.plate_detected = msg.data
    def plate_map_cb(self, msg):
        self.plate_center = (msg.x, msg.y)
        self.plate_radius = msg.z

    def fork_tip_cb(self, msg):
        self.fork_tip = (msg.x, msg.y)
        self.fork_tip_detected = msg.z > 0.5

    def plate_dist_cb(self, msg): self.plate_distance = msg.data
    def mouth_dist_cb(self, msg): self.mouth_distance = msg.data
    def confidence_cb(self, msg): self.fusion_confidence = msg.data
    def health_cb(self, msg): self.sensor_health = list(msg.data)
    def food_pos_3d_cb(self, msg): self.food_position_3d = (msg.x, msg.y, msg.z)

    def target_force_cb(self, msg): self.target_force = msg.data
    def target_angle_cb(self, msg): self.target_angle = msg.data

    # ---- Helpers ----
    def _set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'FSM: {self.state} -> {new_state}')
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
        """Send trajectory goal to arm controller via action client."""
        self.target_pose = list(pose)

        if self.use_arima_ff and len(self.predicted_state) >= 4:
            ff_gain = 0.2
            adjusted = []
            for i in range(4):
                cur = list(self.current_positions.values())[i]
                pred_offset = ff_gain * (self.predicted_state[i] - cur)
                adjusted.append(float(pose[i]) + pred_offset)
            positions = adjusted
        else:
            positions = [float(p) for p in pose]

        # Clamp to joint limits
        for i, jname in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS[jname]
            positions[i] = max(lo, min(hi, positions[i]))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points = [point]

        if self._action_client.server_is_ready():
            self._action_client.send_goal_async(goal)
        else:
            self.get_logger().warn('Arm controller not ready', throttle_duration_sec=5.0)

    def _classify_food(self):
        area = self.food_center[2]
        food = 'apple' if area > 200 else 'strawberry'
        self.food_type_pub.publish(String(data=food))
        return food

    def _fusion_ok(self):
        return self.fusion_confidence >= CONFIDENCE_THRESHOLD

    # ---- IK for food pickup ----
    def _compute_pickup_ik(self, food_x, food_y, food_z):
        """Compute joint angles to reach food position (camera frame).

        Arm geometry: vertical segment (L2) then horizontal segment (L3).
        At j2=0, j3=0: arm points straight up then straight forward.
        """
        j1_cur = self.current_positions['base_y_joint']

        # Base rotation toward food lateral offset
        j1 = j1_cur + math.atan2(food_x, max(food_z, 0.01))
        j1 = max(JOINT_LIMITS['base_y_joint'][0], min(j1, JOINT_LIMITS['base_y_joint'][1]))

        # Target reach and height in arm plane
        arm_reach = L2_LENGTH * math.sin(
            self.current_positions['lower_z_joint']) + L3_LENGTH
        r = arm_reach + food_z
        z = L1_HEIGHT + L2_LENGTH - food_y

        d_sq = r * r + z * z
        d = math.sqrt(d_sq)

        if d > (L2_LENGTH + L3_LENGTH) * 0.98 or d < abs(L2_LENGTH - L3_LENGTH) * 1.02:
            self.get_logger().warn(f'Food unreachable: d={d:.3f}m')
            return None

        cos_j3 = (d_sq - L2_LENGTH**2 - L3_LENGTH**2) / (2 * L2_LENGTH * L3_LENGTH)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        j3 = -math.acos(cos_j3)

        alpha = math.atan2(r, z)
        beta = math.atan2(
            L3_LENGTH * math.sin(j3),
            L2_LENGTH + L3_LENGTH * math.cos(j3))
        j2 = alpha - beta

        j2 = max(JOINT_LIMITS['lower_z_joint'][0], min(j2, JOINT_LIMITS['lower_z_joint'][1]))
        j3 = max(JOINT_LIMITS['upper_z_joint'][0], min(j3, JOINT_LIMITS['upper_z_joint'][1]))
        j4 = -0.5  # fork tilted down for scooping

        self.get_logger().info(
            f'IK: j1={math.degrees(j1):.1f}° j2={math.degrees(j2):.1f}° '
            f'j3={math.degrees(j3):.1f}° j4={math.degrees(j4):.1f}°')
        return [j1, j2, j3, j4]

    # ---- IK for face approach ----
    def _compute_face_approach_ik(self, bearing_h, bearing_v, depth):
        """Compute joint angles to position fork near user's mouth.
        Patient is to the RIGHT (positive base_y rotation).
        """
        approach_depth = max(depth - 0.05, 0.10)

        j1 = bearing_h
        j1 = max(JOINT_LIMITS['base_y_joint'][0], min(j1, JOINT_LIMITS['base_y_joint'][1]))

        # Simple approach: use predefined arm pose but with dynamic base rotation
        j2 = 0.2   # slight shoulder forward
        j3 = -0.2  # slight elbow extend
        j4 = 0.3   # fork roughly horizontal

        return [j1, j2, j3, j4]

    def _cleanup_collect(self):
        """Clean up collect phase tracking variables."""
        for attr in ['_collect_phase', '_phase_start', '_stab_force_baseline',
                     '_force_before_lift', '_food_near_edge']:
            if hasattr(self, attr):
                delattr(self, attr)

    def _food_position_stable(self):
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

    # ---- FSM tick ----
    def tick(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        if self.emergency_stop and self.state != FeedingState.EMERGENCY_STOP:
            return

        elapsed = self._state_elapsed()

        # ---- WAITING ----
        if self.state == FeedingState.WAITING:
            if self.plate_detected and self.food_visible:
                self.get_logger().info('Plate + food detected — auto-starting')
                self._set_state(FeedingState.IDLE)
            elif self.start_requested:
                self.start_requested = False
                self.get_logger().info('Manual start — starting cycle')
                self._set_state(FeedingState.IDLE)

        # ---- IDLE ----
        elif self.state == FeedingState.IDLE:
            if self.food_visible:
                self._set_state(FeedingState.DETECT_FOOD)

        # ---- DETECT_FOOD ----
        elif self.state == FeedingState.DETECT_FOOD:
            if self.food_visible:
                food = self._classify_food()
                self.get_logger().info(f'Food: {food} — locating...')
                self._food_pos_history.clear()
                self._set_state(FeedingState.LOCATE_FOOD)
            elif elapsed > STATE_TIMEOUT:
                self._set_state(FeedingState.IDLE)

        # ---- LOCATE_FOOD (map plate + food, plan approach from above) ----
        elif self.state == FeedingState.LOCATE_FOOD:
            # Phase 1: Move to plate_above to get camera view (4s settle)
            if elapsed < 4.0:
                self._command_pose(POSES['plate_above'])
                self.get_logger().info('Scanning plate and food...',
                                       throttle_duration_sec=4.0)
                return

            if not self.food_visible or self.food_center[2] <= 0:
                if elapsed > STATE_TIMEOUT:
                    self.get_logger().warn('Food lost — proceeding anyway')
                    self._set_state(FeedingState.COLLECT_FOOD)
                return

            # Phase 2: Map plate and food positions
            food_cx = self.food_center[0]
            food_cy = self.food_center[1]
            plate_cx = self.plate_center[0]
            plate_cy = self.plate_center[1]
            plate_r = self.plate_radius

            # Check if food is near the plate edge
            if plate_r > 10:
                dist_from_center = math.sqrt(
                    (food_cx - plate_cx)**2 + (food_cy - plate_cy)**2)
                edge_margin = plate_r - dist_from_center
                near_edge = edge_margin < (plate_r * 0.3)  # within 30% of edge
            else:
                near_edge = False
                edge_margin = 999

            self.get_logger().info(
                f'Plate map: center=({plate_cx:.0f},{plate_cy:.0f}) r={plate_r:.0f}px | '
                f'Food at ({food_cx:.0f},{food_cy:.0f}) | '
                f'Edge margin={edge_margin:.0f}px {"(NEAR EDGE!)" if near_edge else ""}',
                throttle_duration_sec=2.0)

            # Phase 3: Align FORK TIP with food (not image center)
            # The camera sees both fork tip and food — align them horizontally
            fork_x = self.fork_tip[0]
            fork_y = self.fork_tip[1]

            # Horizontal error: food should be at same X as fork tip
            error_x = food_cx - fork_x
            # Vertical error: food should be below fork tip (fork will stab down)
            error_y = food_cy - fork_y

            if self.fork_tip_detected:
                align_target = f'fork tip ({fork_x:.0f},{fork_y:.0f})'
            else:
                align_target = 'image center (fork not detected)'
                fork_x = 320.0

            self.get_logger().info(
                f'Aligning: food({food_cx:.0f},{food_cy:.0f}) → {align_target} '
                f'error=({error_x:.0f},{error_y:.0f})px '
                f'plate edge_margin={edge_margin:.0f}px',
                throttle_duration_sec=1.0)

            if abs(error_x) < 50:
                # Fork tip is horizontally aligned with food — ready to stab!
                self.get_logger().info(
                    f'ALIGNED! Fork tip over food (error_x={error_x:.0f}px)')
                self._food_near_edge = near_edge
                self._set_state(FeedingState.COLLECT_FOOD)
            else:
                # Adjust base rotation to align fork tip with food
                correction = error_x * 0.001  # gentle proportional correction
                adjusted_pose = list(POSES['plate_above'])
                cur_base = self.current_positions['base_y_joint']
                adjusted_pose[0] = cur_base + correction
                adjusted_pose[0] = max(JOINT_LIMITS['base_y_joint'][0],
                                       min(adjusted_pose[0], JOINT_LIMITS['base_y_joint'][1]))
                self._command_pose(adjusted_pose, duration_sec=2)

            if elapsed > STATE_TIMEOUT:
                self._food_near_edge = False
                self._set_state(FeedingState.COLLECT_FOOD)

        # ---- COLLECT_FOOD (plate-aware, sensor-verified, with retry) ----
        elif self.state == FeedingState.COLLECT_FOOD:
            if not hasattr(self, '_collect_phase'):
                self._collect_phase = 'approach'
                self._phase_start = elapsed
                self._stab_force_baseline = self.current_force
                self._force_before_lift = 0.0
                self._stab_attempts = getattr(self, '_stab_attempts', 0)
                self._food_near_edge = getattr(self, '_food_near_edge', False)

            phase_elapsed = elapsed - self._phase_start

            # ---- APPROACH: settle fork directly above food ----
            if self._collect_phase == 'approach':
                self._command_pose(POSES['plate_above'], duration_sec=3)
                self.get_logger().info(
                    f'APPROACH (attempt {self._stab_attempts + 1}/3): '
                    f'sonar={self.plate_distance:.1f}cm, force={self.current_force:.2f}N',
                    throttle_duration_sec=2.0)

                if (self._at_target() and phase_elapsed > 2.0) or phase_elapsed > 4.0:
                    self._stab_force_baseline = self.current_force
                    self._collect_phase = 'stab'
                    self._phase_start = elapsed

            # ---- STAB: fork goes gently DOWN into food ----
            elif self._collect_phase == 'stab':
                stab_pose = list(POSES['plate_above'])
                # Gentle feeder tilt — just enough to pierce food, not slam plate
                stab_pose[3] = stab_pose[3] - 0.2  # gentle stab (was -0.35)
                self._command_pose(stab_pose, duration_sec=4)  # slow (4 seconds)

                force_change = abs(self.current_force - self._stab_force_baseline)
                self.get_logger().info(
                    f'STAB: force={self.current_force:.2f}N (change={force_change:.2f}N) '
                    f'sonar={self.plate_distance:.1f}cm',
                    throttle_duration_sec=1.0)

                # Stop as soon as force detected — don't keep pushing into plate
                if force_change > 0.2:
                    self.get_logger().info(
                        f'STAB: contact detected ({force_change:.2f}N) — stop pushing')
                    self._collect_phase = 'hold'
                    self._phase_start = elapsed
                elif phase_elapsed > 4.0:
                    self.get_logger().info('STAB: max time — holding')
                    self._collect_phase = 'hold'
                    self._phase_start = elapsed

            # ---- HOLD: keep fork pressed in food (3s) ----
            elif self._collect_phase == 'hold':
                self.get_logger().info(
                    f'HOLD: {phase_elapsed:.1f}s, force={self.current_force:.2f}N',
                    throttle_duration_sec=2.0)

                if phase_elapsed > 3.0:
                    self._force_before_lift = self.current_force
                    self._collect_phase = 'lift_fork'
                    self._phase_start = elapsed

            # ---- LIFT FORK: tilt feeder UP first (must clear 6cm plate rim) ----
            elif self._collect_phase == 'lift_fork':
                lift_pose = list(POSES['plate_above'])
                lift_pose[3] = lift_pose[3] + 0.8  # fork tilts up strongly to clear 6cm rim
                lift_pose[1] = lift_pose[1] - 0.15  # shoulder back slightly (helps clear rim)
                self._command_pose(lift_pose, duration_sec=3)
                self.get_logger().info(
                    f'LIFT FORK: tilting up to clear rim',
                    throttle_duration_sec=2.0)

                if phase_elapsed > 3.0:
                    self._collect_phase = 'lift_arm'
                    self._phase_start = elapsed

            # ---- LIFT ARM: raise entire arm up ----
            elif self._collect_phase == 'lift_arm':
                lift_pose = list(POSES['home'])
                lift_pose[3] = -0.8  # fork horizontal-ish to hold food
                self._command_pose(lift_pose, duration_sec=3)
                self.get_logger().info(
                    f'LIFT ARM: raising (force={self.current_force:.2f}N, '
                    f'sonar={self.plate_distance:.1f}cm)',
                    throttle_duration_sec=2.0)

                if phase_elapsed > 4.0:
                    self._collect_phase = 'verify'
                    self._phase_start = elapsed

            # ---- VERIFY: check if food is actually on the fork ----
            elif self._collect_phase == 'verify':
                # Two checks:
                # 1. Force: if force dropped significantly after lift, food fell off
                # 2. Camera: if food is still visible on plate = fork missed
                force_after = self.current_force
                force_diff = abs(force_after - self._stab_force_baseline)
                food_still_on_plate = self.food_visible

                self.get_logger().info(
                    f'VERIFY: force={force_after:.2f}N (baseline={self._stab_force_baseline:.2f}N, '
                    f'diff={force_diff:.2f}N), food_on_plate={food_still_on_plate}',
                    throttle_duration_sec=1.0)

                if phase_elapsed > 3.0:
                    # Decide: food on fork or retry?
                    forked = False

                    if force_diff > 0.2:
                        self.get_logger().info('VERIFY: force confirms food on fork!')
                        forked = True
                    elif not food_still_on_plate:
                        self.get_logger().info('VERIFY: food disappeared from plate — likely forked!')
                        forked = True

                    if forked:
                        # Success — clean up and move to patient
                        self._stab_attempts = 0
                        self._cleanup_collect()
                        self.get_logger().info('Food forked successfully — moving to patient')
                        self._set_state(FeedingState.DETECT_PATIENT)
                    else:
                        # Failed — retry or give up
                        self._stab_attempts += 1
                        if self._stab_attempts < 3:
                            self.get_logger().warn(
                                f'VERIFY FAILED: food still on plate — '
                                f'retry {self._stab_attempts}/3')
                            self._cleanup_collect()
                            # Go back to LOCATE_FOOD to re-center and try again
                            self._set_state(FeedingState.LOCATE_FOOD)
                        else:
                            self.get_logger().warn(
                                'VERIFY FAILED: 3 attempts exhausted — proceeding anyway')
                            self._stab_attempts = 0
                            self._cleanup_collect()
                            self._set_state(FeedingState.DETECT_PATIENT)

        # ---- DETECT_PATIENT ----
        elif self.state == FeedingState.DETECT_PATIENT:
            self._command_pose(POSES['pre_feed'])

            if self.face_detected and self.face_expression == 'ready':
                self.get_logger().info(
                    f'Patient found: bearing={math.degrees(self.face_bearing[0]):.1f}°, '
                    f'depth={self.face_bearing[2]:.2f}m')
                self.face_lost_count = 0
                self._set_state(FeedingState.PRE_FEED)
            elif self.override:
                self._set_state(FeedingState.PRE_FEED)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('No face found — retracting')
                self._set_state(FeedingState.RETRACT)

        # ---- PRE_FEED ----
        elif self.state == FeedingState.PRE_FEED:
            if self.current_force > FORCE_COLLISION_THRESHOLD:
                self.get_logger().warn(f'Force collision ({self.current_force:.1f}N) — retracting')
                self._set_state(FeedingState.RETRACT)
                return

            if self.face_detected:
                self.face_lost_count = 0
                approach_pose = self._compute_face_approach_ik(
                    self.face_bearing[0], self.face_bearing[1], self.face_bearing[2])
                if approach_pose:
                    self._command_pose(approach_pose, FACE_APPROACH_DURATION_SEC)
                else:
                    self._command_pose(POSES['pre_feed'])

                if self._at_target() or elapsed > 5.0:
                    if self.feeding_safe or self.override:
                        self.consecutive_mouth_open = 0
                        self._set_state(FeedingState.FEED)
            else:
                self.face_lost_count += 1
                if self.face_lost_count > FACE_LOST_RETRACT_FRAMES:
                    self.get_logger().warn('Face lost — retracting')
                    self._set_state(FeedingState.RETRACT)

            if elapsed > STATE_TIMEOUT:
                self._set_state(FeedingState.RETRACT)

        # ---- FEED ----
        elif self.state == FeedingState.FEED:
            if self.current_force > FORCE_COLLISION_THRESHOLD:
                self.get_logger().warn('Force collision during feed — retracting')
                self._set_state(FeedingState.RETRACT)
                return

            if not self.face_detected:
                self.face_lost_count += 1
                if self.face_lost_count > FACE_LOST_RETRACT_FRAMES // 2:
                    self.get_logger().warn('Face lost during feed — retreating')
                    self._set_state(FeedingState.PRE_FEED)
                return
            self.face_lost_count = 0

            if self.mouth_open:
                self.consecutive_mouth_open += 1
            else:
                self.consecutive_mouth_open = 0

            if self.consecutive_mouth_open >= MOUTH_OPEN_CONFIRM_FRAMES or self.override:
                self._command_pose(POSES['feed'])
                if self._at_target() or elapsed > 5.0:
                    # Hold at mouth for 3 seconds so patient can take food
                    if elapsed > 8.0:
                        self.feed_count += 1
                        self.get_logger().info(f'Feed #{self.feed_count} delivered!')
                        self.consecutive_mouth_open = 0
                        self._set_state(FeedingState.RETRACT)
                    else:
                        self.get_logger().info(
                            'Holding at mouth — patient taking food...',
                            throttle_duration_sec=2.0)
            elif elapsed % 2.0 < 0.15:
                self.get_logger().info(
                    f'Waiting for mouth ({self.consecutive_mouth_open}/'
                    f'{MOUTH_OPEN_CONFIRM_FRAMES})')

        # ---- EMERGENCY_STOP ----
        elif self.state == FeedingState.EMERGENCY_STOP:
            return

        # ---- RETRACT ----
        elif self.state == FeedingState.RETRACT:
            if elapsed < 4.0:
                # Phase 1: Move to retract position (4s)
                self._command_pose(POSES['retract'], duration_sec=4)
            elif elapsed < 7.0:
                # Phase 2: Move to home (3s)
                self._command_pose(POSES['home'], duration_sec=3)
            elif elapsed < 10.0:
                # Phase 3: Pause at home (3s) before restarting
                self.get_logger().info(
                    'Pausing at home before next cycle...',
                    throttle_duration_sec=3.0)
            else:
                self.get_logger().info(f'Cycle #{self.feed_count} complete — auto-restarting')
                self._set_state(FeedingState.WAITING)


def main(args=None):
    rclpy.init(args=args)
    node = FeedingFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
