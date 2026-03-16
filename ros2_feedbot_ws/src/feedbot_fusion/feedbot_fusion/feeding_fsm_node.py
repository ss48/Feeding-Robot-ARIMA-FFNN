"""
Feeding Finite State Machine (FSM) Node

Coordinates the full feeding sequence described in the paper (Section 2.2,
Figure 1 block diagram):

  IDLE -> DETECT_FOOD -> COLLECT_FOOD -> DETECT_PATIENT ->
  PRE_FEED -> FEED -> RETRACT -> IDLE

At each state transition the FSM checks safety conditions from the fuzzy
controller, uses predicted states from the ARIMA-FFNN node, and commands
joint trajectories via the joint controllers.

The FSM halts the feeding sequence whenever the user's mouth is detected as
closed (Section 3, page 20) and resumes when mouth-open is detected or an
override is received.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from control_msgs.msg import MultiDOFCommand

import math


class FeedingState:
    IDLE = 'IDLE'
    DETECT_FOOD = 'DETECT_FOOD'
    COLLECT_FOOD = 'COLLECT_FOOD'
    DETECT_PATIENT = 'DETECT_PATIENT'
    PRE_FEED = 'PRE_FEED'
    FEED = 'FEED'
    RETRACT = 'RETRACT'


# Predefined joint poses (radians) for each feeding phase
# These should be calibrated for the actual robot setup
POSES = {
    'home':         [0.0,   0.0,   0.0,   0.0],
    'plate_above':  [0.0,  -0.3,   0.4,   0.0],
    'plate_pickup': [0.0,  -0.5,   0.6,  -0.3],
    'pre_feed':     [1.2,   0.0,  -0.2,   0.5],
    'feed':         [1.2,   0.3,  -0.4,   0.8],
    'retract':      [0.6,   0.0,   0.0,   0.2],
}

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

# Position tolerance for considering a joint "arrived"
POSITION_TOLERANCE = 0.08  # rad
# Maximum time in any single state before timeout (seconds)
STATE_TIMEOUT = 15.0


class FeedingFSMNode(Node):
    """State machine that orchestrates the full feeding cycle.

    Topics subscribed:
        /joint_states               (JointState)
        /food_visible               (Bool)
        /food_center                (Point)
        /feeding_safe               (Bool)        - from fuzzy controller
        /mouth_ready_prediction     (Bool)        - from ARIMA-FFNN node
        /predicted_state            (Float64MultiArray)
        /spoon_force                (Float64)
        /feeding_override           (Bool)        - manual override to resume

    Topics published:
        /joint1_controller/reference (MultiDOFCommand)
        /joint2_controller/reference (MultiDOFCommand)
        /joint3_controller/reference (MultiDOFCommand)
        /joint4_controller/reference (MultiDOFCommand)
        /feeding_state               (String)     - current FSM state
        /food_type                   (String)     - detected food type
    """

    def __init__(self):
        super().__init__('feeding_fsm_node')

        self.state = FeedingState.IDLE
        self.state_start_time = self.get_clock().now()

        # Sensor data
        self.current_positions = [0.0] * 4
        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)
        self.feeding_safe = False
        self.mouth_ready = False
        self.predicted_state = [0.0] * 4
        self.current_force = 0.0
        self.override = False

        # Target pose for current state
        self.target_pose = POSES['home']

        # Feeding cycle counter
        self.feed_count = 0

        # ---- subscribers ----
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(
            Bool, '/food_visible', self.food_visible_cb, 10)
        self.create_subscription(
            Point, '/food_center', self.food_center_cb, 10)
        self.create_subscription(
            Bool, '/feeding_safe', self.safe_cb, 10)
        self.create_subscription(
            Bool, '/mouth_ready_prediction', self.mouth_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/predicted_state', self.pred_cb, 10)
        self.create_subscription(
            Float64, '/spoon_force', self.force_cb, 10)
        self.create_subscription(
            Bool, '/feeding_override', self.override_cb, 10)

        # ---- publishers ----
        self.joint_pubs = {}
        for jname in JOINT_NAMES:
            self.joint_pubs[jname] = self.create_publisher(
                MultiDOFCommand, f'/{jname}_controller/reference', 10)

        self.state_pub = self.create_publisher(String, '/feeding_state', 10)
        self.food_type_pub = self.create_publisher(String, '/food_type', 10)

        # 10 Hz FSM tick
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info('Feeding FSM node started')

    # ---- callbacks ----
    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            for i, jn in enumerate(JOINT_NAMES):
                if name == jn:
                    self.current_positions[i] = pos

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

    # ---- helpers ----
    def _set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'FSM: {self.state} -> {new_state}')
            self.state = new_state
            self.state_start_time = self.get_clock().now()

    def _state_elapsed(self):
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def _at_target(self):
        """Check if all joints have reached the target pose."""
        for cur, tgt in zip(self.current_positions, self.target_pose):
            if abs(cur - tgt) > POSITION_TOLERANCE:
                return False
        return True

    def _command_pose(self, pose):
        """Send joint position commands for the given pose."""
        self.target_pose = pose
        for i, jname in enumerate(JOINT_NAMES):
            msg = MultiDOFCommand()
            msg.dof_names = [jname]
            msg.values = [pose[i]]
            msg.values_dot = []
            self.joint_pubs[jname].publish(msg)

    def _classify_food(self):
        """Simple food classification based on detected area.
        Large area -> apple, small area -> strawberry."""
        area = self.food_center[2]
        food = 'apple' if area > 200 else 'strawberry'
        msg = String()
        msg.data = food
        self.food_type_pub.publish(msg)
        return food

    # ---- FSM tick ----
    def tick(self):
        """Main FSM update - called at 10 Hz."""

        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        elapsed = self._state_elapsed()

        # ---- IDLE ----
        if self.state == FeedingState.IDLE:
            self._command_pose(POSES['home'])
            # Transition: start when food is visible on the plate
            if self.food_visible:
                self._set_state(FeedingState.DETECT_FOOD)

        # ---- DETECT_FOOD ----
        elif self.state == FeedingState.DETECT_FOOD:
            if self.food_visible:
                food = self._classify_food()
                self.get_logger().info(f'Food detected: {food}')
                self._set_state(FeedingState.COLLECT_FOOD)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Food detection timed out')
                self._set_state(FeedingState.IDLE)

        # ---- COLLECT_FOOD ----
        elif self.state == FeedingState.COLLECT_FOOD:
            # Move above plate, then down to pick up
            if elapsed < 2.0:
                self._command_pose(POSES['plate_above'])
            else:
                self._command_pose(POSES['plate_pickup'])

            # Check if force indicates food has been picked up
            if self.current_force > 2.0 and elapsed > 3.0:
                self.get_logger().info(
                    f'Food collected (force={self.current_force:.1f}N)')
                self._set_state(FeedingState.DETECT_PATIENT)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Food collection timed out')
                self._set_state(FeedingState.RETRACT)

        # ---- DETECT_PATIENT ----
        elif self.state == FeedingState.DETECT_PATIENT:
            # Wait for mouth-readiness prediction from ARIMA-FFNN
            if self.mouth_ready or self.override:
                self._set_state(FeedingState.PRE_FEED)
            elif elapsed > STATE_TIMEOUT:
                self.get_logger().warn('Patient detection timed out')
                self._set_state(FeedingState.RETRACT)

        # ---- PRE_FEED ----
        elif self.state == FeedingState.PRE_FEED:
            self._command_pose(POSES['pre_feed'])
            if self._at_target() or elapsed > 5.0:
                # Safety check from fuzzy controller
                if self.feeding_safe or self.override:
                    self._set_state(FeedingState.FEED)
                elif elapsed > STATE_TIMEOUT:
                    self._set_state(FeedingState.RETRACT)

        # ---- FEED ----
        elif self.state == FeedingState.FEED:
            # Only proceed if mouth is predicted open
            if self.mouth_ready or self.override:
                self._command_pose(POSES['feed'])
                if self._at_target() or elapsed > 5.0:
                    self.feed_count += 1
                    self.get_logger().info(
                        f'Feed #{self.feed_count} delivered')
                    self._set_state(FeedingState.RETRACT)
            else:
                # Mouth closed - halt and wait (paper Section 3)
                self.get_logger().info(
                    'Mouth not ready - pausing feed sequence')

        # ---- RETRACT ----
        elif self.state == FeedingState.RETRACT:
            self._command_pose(POSES['retract'])
            if self._at_target() or elapsed > 5.0:
                self._set_state(FeedingState.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = FeedingFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
