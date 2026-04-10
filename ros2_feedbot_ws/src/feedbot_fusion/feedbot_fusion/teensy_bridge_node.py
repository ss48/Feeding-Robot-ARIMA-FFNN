"""
Teensy Serial Bridge Node — reads HX711 force + HC-SR04 sonar from Teensy
over serial and publishes to ROS 2 topics.

Teensy sends lines:
  Distance (cm): 8.86
  Load Cell Reading: 9.2 g

Publishes:
  /spoon/wrench                    (WrenchStamped) — for force_node
  /spoon_force                     (Float64)       — filtered force (N)
  /feeding_robot/ultrasonic/range  (Range)         — for sonar_bridge
  /sonar_raw_cm                    (Float64)       — raw distance in cm

Parameters:
  serial_port  (str)  — default /dev/teensy
  baud_rate    (int)  — default 115200
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import serial


class TeensyBridgeNode(Node):

    FORCE_BUFFER_SIZE = 10
    GRAMS_TO_NEWTONS = 0.00981

    def __init__(self):
        super().__init__('teensy_bridge_node')

        self.declare_parameter('serial_port', '/dev/teensy')
        self.declare_parameter('baud_rate', 115200)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value

        self.wrench_pub = self.create_publisher(WrenchStamped, '/spoon/wrench', 10)
        self.force_pub = self.create_publisher(Float64, '/spoon_force', 10)
        self.range_pub = self.create_publisher(Range, '/feeding_robot/ultrasonic/range', 10)
        self.sonar_raw_pub = self.create_publisher(Float64, '/sonar_raw_cm', 10)

        self.force_buffer = []
        self.ser = None
        self._reconnect_interval = 5.0  # seconds between reconnect attempts
        self._last_reconnect = 0.0
        self._connect()

        self.create_timer(0.02, self.read_serial)

    def _connect(self):
        try:
            self.ser = serial.Serial(self._port, self._baud, timeout=0.1)
            self.get_logger().info(f'Teensy connected on {self._port} @ {self._baud}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Teensy not available ({self._port}) — will retry every {self._reconnect_interval:.0f}s')
            self.ser = None

    def _reconnect(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._connect()

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            now = time.monotonic()
            if now - self._last_reconnect < self._reconnect_interval:
                return
            self._last_reconnect = now
            self._reconnect()
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_and_publish(line)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Serial error: {e}, reconnecting...')
            self._reconnect()

    def parse_and_publish(self, line):
        now = self.get_clock().now().to_msg()

        if line.startswith('Distance (cm):'):
            try:
                dist_cm = float(line.split(':')[1].strip())
            except (ValueError, IndexError):
                return

            range_msg = Range()
            range_msg.header.stamp = now
            range_msg.header.frame_id = 'ultrasonic_link'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26
            range_msg.min_range = 0.02
            range_msg.max_range = 4.0
            range_msg.range = dist_cm / 100.0
            self.range_pub.publish(range_msg)

            raw_msg = Float64()
            raw_msg.data = dist_cm
            self.sonar_raw_pub.publish(raw_msg)

        elif line.startswith('Load Cell Reading:'):
            try:
                grams = float(line.split(':')[1].strip().replace('g', '').strip())
            except (ValueError, IndexError):
                return
            force_n = grams * self.GRAMS_TO_NEWTONS

            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = now
            wrench_msg.header.frame_id = 'load_cell_frame'
            wrench_msg.wrench.force.z = force_n
            self.wrench_pub.publish(wrench_msg)

            self.force_buffer.append(abs(force_n))
            if len(self.force_buffer) > self.FORCE_BUFFER_SIZE:
                self.force_buffer.pop(0)

            force_msg = Float64()
            force_msg.data = sum(self.force_buffer) / len(self.force_buffer)
            self.force_pub.publish(force_msg)

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
