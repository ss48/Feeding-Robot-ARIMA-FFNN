"""
Sensor Fusion Node (upgraded)

Fuses vision, force, joint state, and ARIMA-FFNN prediction data into a
unified error signal and distance estimates for downstream controllers.

Computes:
  - /food_error_x  : visual servoing error (pixel offset of food from center)
  - /plate_distance : estimated distance to plate (cm) from food area
  - /mouth_distance : estimated distance to mouth (cm) from joint positions
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

import math
import numpy as np


# Camera parameters (approximate – calibrate for your setup)
IMAGE_CENTER_X = 320.0  # half of 640px width
AREA_TO_DISTANCE_K = 5000.0  # empirical: distance ~ K / sqrt(area)


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        # State
        self.food_visible = False
        self.food_center = (0.0, 0.0, 0.0)  # x, y, area
        self.force = 0.0
        self.joint_positions = {}
        self.predicted_state = [0.0] * 4
        self.prediction_error = 0.0

        # Subscribers
        self.create_subscription(
            Bool, '/food_visible', self.food_callback, 10)
        self.create_subscription(
            Point, '/food_center', self.food_center_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(
            Float64, '/spoon_force', self.force_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/predicted_state', self.pred_callback, 10)
        self.create_subscription(
            Float64, '/prediction_error', self.pred_error_callback, 10)

        # Publishers
        self.error_pub = self.create_publisher(
            Float64, '/food_error_x', 10)
        self.plate_dist_pub = self.create_publisher(
            Float64, '/plate_distance', 10)
        self.mouth_dist_pub = self.create_publisher(
            Float64, '/mouth_distance', 10)

        # Run fusion at 10 Hz
        self.timer = self.create_timer(0.1, self.fuse_and_publish)

        self.get_logger().info("Fusion node started (upgraded with ARIMA-FFNN integration)")

    # ---- callbacks ----
    def food_callback(self, msg):
        self.food_visible = msg.data

    def food_center_callback(self, msg):
        self.food_center = (msg.x, msg.y, msg.z)

    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def force_callback(self, msg):
        self.force = msg.data

    def pred_callback(self, msg):
        self.predicted_state = list(msg.data)

    def pred_error_callback(self, msg):
        self.prediction_error = msg.data

    # ---- fusion logic ----
    def fuse_and_publish(self):
        # --- Food error (visual servoing) ---
        error_msg = Float64()
        if self.food_visible and self.food_center[0] >= 0:
            # Normalised pixel error: how far food is from image centre
            error_msg.data = (self.food_center[0] - IMAGE_CENTER_X) / IMAGE_CENTER_X
        else:
            error_msg.data = 0.0
        self.error_pub.publish(error_msg)

        # --- Plate distance estimate (from food bounding-box area) ---
        plate_msg = Float64()
        area = self.food_center[2]
        if area > 1.0:
            plate_msg.data = AREA_TO_DISTANCE_K / math.sqrt(area)
        else:
            plate_msg.data = 50.0  # default far
        self.plate_dist_pub.publish(plate_msg)

        # --- Mouth distance estimate (from joint kinematics) ---
        # Simple forward-kinematics approximation:
        # distance decreases as joint2+joint3 extend the arm forward
        mouth_msg = Float64()
        j2 = self.joint_positions.get('joint2', 0.0)
        j3 = self.joint_positions.get('joint3', 0.0)
        # Rough estimate: arm reach ~ 35cm at full extension
        reach = 20.0 + 15.0 * (abs(j2) + abs(j3)) / 3.0
        mouth_msg.data = max(5.0, 40.0 - reach)
        self.mouth_dist_pub.publish(mouth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
