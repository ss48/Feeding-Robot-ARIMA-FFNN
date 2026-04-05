"""
Teensy Serial Bridge Node — reads HX711 force + HC-SR04 sonar from Teensy
over serial JSON and publishes to ROS 2 topics.

Teensy sends lines like:  {"force":0.1234,"dist_cm":15.20}

Publishes:
  /spoon/wrench          (WrenchStamped) — for force_node compatibility
  /spoon_force           (Float64)       — filtered force (N), direct
  /feeding_robot/ultrasonic/range (Range) — for sonar_bridge compatibility
  /sonar_raw_cm          (Float64)       — raw distance in cm

Parameters:
  serial_port  (str)   — default /dev/ttyACM1
  baud_rate    (int)   — default 115200
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, Header
import serial


class TeensyBridgeNode(Node):

    def __init__(self):
        super().__init__('teensy_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # Publishers
        self.wrench_pub = self.create_publisher(
            WrenchStamped, '/spoon/wrench', 10)
        self.force_pub = self.create_publisher(
            Float64, '/spoon_force', 10)
        self.range_pub = self.create_publisher(
            Range, '/feeding_robot/ultrasonic/range', 10)
        self.sonar_raw_pub = self.create_publisher(
            Float64, '/sonar_raw_cm', 10)

        # Force filtering buffer
        self.force_buffer = []
        self.FORCE_BUFFER_SIZE = 10

        # Open serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Teensy connected on {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            self.ser = None

        # Timer to read serial at 50 Hz
        self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                self.parse_and_publish(line)
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def parse_and_publish(self, line):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return

        # Skip status messages
        if 'status' in data or 'tare' in data:
            self.get_logger().info(f'Teensy: {line}')
            return

        now = self.get_clock().now().to_msg()

        # ---- Force ----
        if 'force' in data:
            force_n = float(data['force'])

            # Publish WrenchStamped (force_node subscribes to this)
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = now
            wrench_msg.header.frame_id = 'load_cell_frame'
            wrench_msg.wrench.force.z = force_n  # Main axis
            self.wrench_pub.publish(wrench_msg)

            # Also publish filtered force directly
            self.force_buffer.append(abs(force_n))
            if len(self.force_buffer) > self.FORCE_BUFFER_SIZE:
                self.force_buffer.pop(0)
            filtered = sum(self.force_buffer) / len(self.force_buffer)

            force_msg = Float64()
            force_msg.data = filtered
            self.force_pub.publish(force_msg)

        # ---- Sonar ----
        if 'dist_cm' in data and data['dist_cm'] is not None:
            dist_cm = float(data['dist_cm'])
            dist_m = dist_cm / 100.0

            # Publish Range (sonar_bridge_node accepts this)
            range_msg = Range()
            range_msg.header.stamp = now
            range_msg.header.frame_id = 'ultrasonic_link'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26  # ~15 degrees
            range_msg.min_range = 0.02
            range_msg.max_range = 4.0
            range_msg.range = dist_m
            self.range_pub.publish(range_msg)

            # Also publish raw cm
            raw_msg = Float64()
            raw_msg.data = dist_cm
            self.sonar_raw_pub.publish(raw_msg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
