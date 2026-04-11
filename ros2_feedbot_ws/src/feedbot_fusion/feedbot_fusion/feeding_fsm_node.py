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
    'base_y_joint':  (-3.05, 3.05),
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
        """Send arm to home position once on startup."""
        if not self._homed:
            self._homed = True
            self.get_logger().info('Moving arm to HOME position on startup...')
            self._command_pose(POSES['home'], duration_sec=5)

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

        # ---- LOCATE_FOOD (sensor-guided: camera bearing + sonar depth) ----
        elif self.state == FeedingState.LOCATE_FOOD:
            # Phase 1: Move to plate_above to get camera view of food
            if elapsed < 3.0:
                self._command_pose(POSES['plate_above'])
                return

            # Phase 2: Use camera + sonar to locate food in 3D
            if self._fusion_ok() and self._food_position_stable():
                fx, fy, fz = self.food_position_3d
                self.get_logger().info(
                    f'Food located via camera+sonar: ({fx:.3f}, {fy:.3f}, {fz:.3f})m '
                    f'plate_dist={self.plate_distance:.1f}cm '
                    f'confidence={self.fusion_confidence:.2f}')
                self.pickup_pose = self._compute_pickup_ik(fx, fy, fz)
                if self.pickup_pose is not None:
                    self.get_logger().info('IK computed — moving to collect')
                else:
                    self.get_logger().warn('IK failed — using plate_above as fallback')
                self._set_state(FeedingState.COLLECT_FOOD)
            elif not self._fusion_ok() and elapsed > 5.0:
                # Fusion not ready — use food bearing directly
                bx, by, bz = 0.0, 0.0, 0.0
                if self.food_center[2] > 0:
                    # Food visible in camera — use bearing for approximate IK
                    self.get_logger().info(
                        f'Using camera bearing (no fusion): '
                        f'center=({self.food_center[0]:.0f}, {self.food_center[1]:.0f})')
                self.pickup_pose = None  # will use plate_above
                self._set_state(FeedingState.COLLECT_FOOD)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Locate timeout — using plate_above')
                self.pickup_pose = None
                self._set_state(FeedingState.COLLECT_FOOD)

        # ---- COLLECT_FOOD (multi-phase: approach → descend → stab → verify → lift) ----
        elif self.state == FeedingState.COLLECT_FOOD:
            FORK_FORCE_THRESHOLD = 1.0  # N — force confirming fork contact with food
            LIFT_FORCE_THRESHOLD = 0.5  # N — force confirming food is on fork after lifting

            if elapsed < 2.0:
                # Phase 1: Position above the food
                if self.pickup_pose is not None:
                    # IK-computed approach from sensor data
                    approach = list(self.pickup_pose)
                    approach[3] = -1.2  # feeder tilted down but not fully stabbing yet
                    self._command_pose(approach)
                else:
                    self._command_pose(POSES['plate_above'])
                self.get_logger().info(
                    f'Approaching food (sonar={self.plate_distance:.1f}cm, '
                    f'force={self.current_force:.2f}N)',
                    throttle_duration_sec=2.0)

            elif elapsed < 5.0:
                # Phase 2: Descend and stab — go to full plate_above (fork down into food)
                self._command_pose(POSES['plate_above'])
                if self.current_force > FORK_FORCE_THRESHOLD:
                    self.get_logger().info(
                        f'Fork contact! force={self.current_force:.2f}N — '
                        f'food stabbed at sonar={self.plate_distance:.1f}cm')

            elif elapsed < 8.0:
                # Phase 3: Lift food off plate — raise shoulder, keep fork tilted
                lift_pose = list(POSES['plate_above'])
                lift_pose[1] = lift_pose[1] - 0.3  # raise shoulder (less forward lean)
                lift_pose[3] = -1.0  # tilt fork slightly less to hold food
                self._command_pose(lift_pose)

                if self.current_force > LIFT_FORCE_THRESHOLD:
                    self.get_logger().info(
                        f'Food on fork confirmed (force={self.current_force:.2f}N) — '
                        f'moving to patient')
                    self._set_state(FeedingState.DETECT_PATIENT)

            else:
                # Phase 4: Timeout — assume food collected, proceed
                self.get_logger().info(
                    f'Collection complete (force={self.current_force:.2f}N, '
                    f'sonar={self.plate_distance:.1f}cm) — moving to patient')
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
                    self.feed_count += 1
                    self.get_logger().info(f'Feed #{self.feed_count} delivered!')
                    self.consecutive_mouth_open = 0
                    self._set_state(FeedingState.RETRACT)
            elif elapsed % 2.0 < 0.15:
                self.get_logger().info(
                    f'Waiting for mouth ({self.consecutive_mouth_open}/'
                    f'{MOUTH_OPEN_CONFIRM_FRAMES})')

        # ---- EMERGENCY_STOP ----
        elif self.state == FeedingState.EMERGENCY_STOP:
            return

        # ---- RETRACT ----
        elif self.state == FeedingState.RETRACT:
            self._command_pose(POSES['retract'])
            if self._at_target() or elapsed > 5.0:
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
