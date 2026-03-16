"""
Fuzzy Logic Force and Angle Controller Node

Implements the fuzzy rules from Section 2.5.1 of the paper:

  Rule 1: plate=Near, mouth=Medium/Far, fruit=Apple -> High Force (20-30N), Steep Angle (60 deg)
  Rule 2: plate=Near, mouth=Near, fruit=Strawberry  -> Low Force (5-10N), Shallow Angle (45 deg)
  Rule 3: plate=Far                                  -> No action (safety)

Subscribes to distance, food type, and force data.
Publishes target force and target angle for the feeding end-effector.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Point

import numpy as np


# ---------------------------------------------------------------------------
# Fuzzy membership functions
# ---------------------------------------------------------------------------
def membership_near(distance):
    """Membership for 'Near' distance (0-15 cm)."""
    if distance <= 5.0:
        return 1.0
    elif distance <= 15.0:
        return (15.0 - distance) / 10.0
    return 0.0


def membership_medium(distance):
    """Membership for 'Medium' distance (10-30 cm)."""
    if distance <= 10.0:
        return 0.0
    elif distance <= 20.0:
        return (distance - 10.0) / 10.0
    elif distance <= 30.0:
        return (30.0 - distance) / 10.0
    return 0.0


def membership_far(distance):
    """Membership for 'Far' distance (>25 cm)."""
    if distance <= 25.0:
        return 0.0
    elif distance <= 40.0:
        return (distance - 25.0) / 15.0
    return 1.0


# ---------------------------------------------------------------------------
# Food-type force/angle parameters (Table 5 in the paper)
# ---------------------------------------------------------------------------
FOOD_PARAMS = {
    'apple': {
        'force_high': 25.0,   # N  (20-30 N range, nominal 25)
        'force_low': 15.0,    # N  (reduced for close mouth)
        'angle_steep': 60.0,  # degrees
        'angle_shallow': 45.0,
    },
    'strawberry': {
        'force_high': 7.0,    # N  (5-10 N range, nominal 7)
        'force_low': 5.0,     # N
        'angle_steep': 50.0,  # degrees
        'angle_shallow': 45.0,
    },
}

DEFAULT_FOOD = 'apple'


class FuzzyControllerNode(Node):
    """Fuzzy logic controller for force and angle regulation.

    Topics subscribed:
        /plate_distance     (Float64) – distance from end-effector to plate (cm)
        /mouth_distance     (Float64) – distance from end-effector to mouth (cm)
        /food_type          (String)  – detected food type ('apple'/'strawberry')
        /food_visible       (Bool)    – whether food is detected
        /spoon_force        (Float64) – current measured force

    Topics published:
        /target_force       (Float64) – desired force (N)
        /target_angle       (Float64) – desired fork/spoon angle (degrees)
        /feeding_safe       (Bool)    – whether it is safe to proceed
    """

    def __init__(self):
        super().__init__('fuzzy_controller_node')

        # State
        self.plate_distance = 10.0   # cm (default Near)
        self.mouth_distance = 20.0   # cm (default Medium)
        self.food_type = DEFAULT_FOOD
        self.food_visible = False
        self.current_force = 0.0

        # Subscribers
        self.create_subscription(
            Float64, '/plate_distance', self.plate_dist_cb, 10)
        self.create_subscription(
            Float64, '/mouth_distance', self.mouth_dist_cb, 10)
        self.create_subscription(
            String, '/food_type', self.food_type_cb, 10)
        self.create_subscription(
            Bool, '/food_visible', self.food_visible_cb, 10)
        self.create_subscription(
            Float64, '/spoon_force', self.force_cb, 10)

        # Publishers
        self.force_pub = self.create_publisher(Float64, '/target_force', 10)
        self.angle_pub = self.create_publisher(Float64, '/target_angle', 10)
        self.safe_pub = self.create_publisher(Bool, '/feeding_safe', 10)

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Fuzzy controller node started')

    # ---- callbacks ----
    def plate_dist_cb(self, msg):
        self.plate_distance = msg.data

    def mouth_dist_cb(self, msg):
        self.mouth_distance = msg.data

    def food_type_cb(self, msg):
        ft = msg.data.lower().strip()
        if ft in FOOD_PARAMS:
            self.food_type = ft

    def food_visible_cb(self, msg):
        self.food_visible = msg.data

    def force_cb(self, msg):
        self.current_force = msg.data

    # ---- fuzzy inference ----
    def control_loop(self):
        """Evaluate fuzzy rules and publish target force/angle."""

        pd = self.plate_distance
        md = self.mouth_distance

        # Compute memberships
        plate_near = membership_near(pd)
        plate_medium = membership_medium(pd)
        plate_far = membership_far(pd)

        mouth_near = membership_near(md)
        mouth_medium = membership_medium(md)
        mouth_far = membership_far(md)

        params = FOOD_PARAMS.get(self.food_type, FOOD_PARAMS[DEFAULT_FOOD])

        # ---- Rule 3 (safety): plate is Far -> do NOT proceed ----
        safe = True
        if plate_far > 0.5:
            safe = False

        # ---- Rule 1: plate=Near, mouth=Medium/Far -> normal force ----
        rule1_strength = plate_near * max(mouth_medium, mouth_far)
        force_r1 = params['force_high']
        angle_r1 = params['angle_steep']

        # ---- Rule 2: plate=Medium/Near, mouth=Near -> reduced force ----
        rule2_strength = max(plate_near, plate_medium) * mouth_near
        force_r2 = params['force_low']
        angle_r2 = params['angle_shallow']

        # Defuzzification (weighted average)
        total_strength = rule1_strength + rule2_strength + 1e-8
        target_force = (rule1_strength * force_r1 +
                        rule2_strength * force_r2) / total_strength
        target_angle = (rule1_strength * angle_r1 +
                        rule2_strength * angle_r2) / total_strength

        # Publish
        force_msg = Float64()
        force_msg.data = target_force
        self.force_pub.publish(force_msg)

        angle_msg = Float64()
        angle_msg.data = target_angle
        self.angle_pub.publish(angle_msg)

        safe_msg = Bool()
        safe_msg.data = safe and self.food_visible
        self.safe_pub.publish(safe_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
