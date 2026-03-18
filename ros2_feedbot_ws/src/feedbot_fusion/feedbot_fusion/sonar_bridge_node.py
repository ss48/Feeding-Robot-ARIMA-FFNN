"""
Sonar Bridge Node — converts ultrasonic LaserScan to distance Float64.

In Gazebo Fortress the HC-SR04 is simulated as a gpu_lidar sensor which
publishes sensor_msgs/LaserScan.  This node takes the minimum range from
that scan (representing the closest object in the sonar beam) and publishes:
  /sonar_plate_distance  (Float64) — distance to nearest object (cm)
  /sonar_mouth_distance  (Float64) — distance towards patient (cm)

Routing is based on the current feeding_state topic:
  plate states  -> /sonar_plate_distance
  patient states -> /sonar_mouth_distance
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float64, String


class SonarBridgeNode(Node):

    def __init__(self):
        super().__init__('sonar_bridge_node')

        self.feeding_state = 'IDLE'

        # Accept both LaserScan (Fortress gpu_lidar) and Range (classic)
        self.create_subscription(
            LaserScan, '/feeding_robot/ultrasonic/scan',
            self.scan_cb, 10)
        self.create_subscription(
            Range, '/feeding_robot/ultrasonic/range',
            self.range_cb, 10)

        # Feeding state for routing
        self.create_subscription(
            String, '/feeding_state',
            self.state_cb, 10)

        # Distance publishers (cm, matching fusion_node convention)
        self.plate_dist_pub = self.create_publisher(
            Float64, '/sonar_plate_distance', 10)
        self.mouth_dist_pub = self.create_publisher(
            Float64, '/sonar_mouth_distance', 10)

        self.get_logger().info('Sonar bridge node started')

    def state_cb(self, msg):
        self.feeding_state = msg.data

    def scan_cb(self, msg):
        """Handle LaserScan from Fortress gpu_lidar."""
        # Take the minimum valid range from the scan
        valid = [r for r in msg.ranges
                 if msg.range_min <= r <= msg.range_max
                 and math.isfinite(r)]
        if not valid:
            return
        dist_m = min(valid)
        self._publish_distance(dist_m)

    def range_cb(self, msg):
        """Handle Range from classic Gazebo ray sensor."""
        if msg.range < msg.min_range or msg.range > msg.max_range:
            return
        self._publish_distance(msg.range)

    def _publish_distance(self, dist_m):
        """Convert metres to cm and route to the appropriate topic."""
        dist_cm = dist_m * 100.0

        dist_msg = Float64()
        dist_msg.data = dist_cm

        if self.feeding_state in ('IDLE', 'DETECT_FOOD', 'COLLECT_FOOD'):
            self.plate_dist_pub.publish(dist_msg)
        elif self.feeding_state in ('DETECT_PATIENT', 'PRE_FEED', 'FEED'):
            self.mouth_dist_pub.publish(dist_msg)
        else:
            self.plate_dist_pub.publish(dist_msg)
            self.mouth_dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
