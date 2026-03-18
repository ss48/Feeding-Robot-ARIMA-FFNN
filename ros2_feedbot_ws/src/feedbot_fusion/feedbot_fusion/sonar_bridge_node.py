"""
Sonar Bridge Node — converts ultrasonic Range messages to distance Float64.

Subscribes to /feeding_robot/ultrasonic/range (sensor_msgs/Range) from
the Gazebo HC-SR04 simulation and publishes:
  /sonar_plate_distance  (Float64) — distance to nearest object (plate/food)
  /sonar_mouth_distance  (Float64) — distance estimate towards patient

The ultrasonic sensor is mounted on the feeder link and points forward,
so the range reading corresponds to whatever is in front of the spoon.
During COLLECT_FOOD the sensor measures plate distance; during FEED it
measures mouth distance.  The feeding_state topic is used to route the
reading to the appropriate output.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, String


class SonarBridgeNode(Node):

    def __init__(self):
        super().__init__('sonar_bridge_node')

        self.feeding_state = 'IDLE'

        # Subscribe to ultrasonic sensor
        self.create_subscription(
            Range, '/feeding_robot/ultrasonic/range',
            self.range_cb, 10)

        # Subscribe to feeding state to know where the arm is pointing
        self.create_subscription(
            String, '/feeding_state',
            self.state_cb, 10)

        # Distance publishers (in cm, matching fusion_node convention)
        self.plate_dist_pub = self.create_publisher(
            Float64, '/sonar_plate_distance', 10)
        self.mouth_dist_pub = self.create_publisher(
            Float64, '/sonar_mouth_distance', 10)

        self.get_logger().info('Sonar bridge node started')

    def state_cb(self, msg):
        self.feeding_state = msg.data

    def range_cb(self, msg):
        # Convert metres to cm
        dist_cm = msg.range * 100.0

        # Clamp to valid range
        min_cm = msg.min_range * 100.0
        max_cm = msg.max_range * 100.0
        dist_cm = max(min_cm, min(dist_cm, max_cm))

        dist_msg = Float64()
        dist_msg.data = dist_cm

        # Route based on feeding state
        if self.feeding_state in ('IDLE', 'DETECT_FOOD', 'COLLECT_FOOD'):
            # Arm is over the plate — sonar measures plate distance
            self.plate_dist_pub.publish(dist_msg)
        elif self.feeding_state in ('DETECT_PATIENT', 'PRE_FEED', 'FEED'):
            # Arm is near the patient — sonar measures mouth distance
            self.mouth_dist_pub.publish(dist_msg)
        else:
            # RETRACT or unknown — publish both
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
