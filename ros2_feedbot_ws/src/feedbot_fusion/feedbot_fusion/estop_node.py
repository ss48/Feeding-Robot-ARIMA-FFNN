"""
Emergency Stop Node — Physical GPIO button for immediate arm safety.

Monitors a push button on a Raspberry Pi GPIO pin. On press:
  1. Immediately publishes /emergency_stop (Bool=True)
  2. FSM freezes arm at current position (hold)
  3. After hold_before_disable seconds, disables Dynamixel torque via SDK
  4. Arm goes limp (safe state)

On release:
  1. Re-enables Dynamixel torque
  2. Publishes /emergency_stop (Bool=False)

Wiring:
  GPIO 17 (pin 11) ──── Button ──── GND (pin 9)
  Internal pull-up resistor used (no external resistor needed)

Parameters:
  gpio_pin            (int)   — default 17
  hold_before_disable (float) — seconds to hold before torque off (default 2.0)
  dynamixel_port      (str)   — default /dev/dynamixel
  baud_rate           (int)   — default 1000000
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Dynamixel SDK for direct torque control (most reliable for E-Stop)
from dynamixel_sdk import PortHandler, PacketHandler

SERVO_IDS = [11, 12, 13, 14]
TORQUE_ENABLE_ADDR = 64  # XM430 torque enable register
PROTOCOL_VERSION = 2.0


class EStopNode(Node):

    def __init__(self):
        super().__init__('estop_node')

        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('hold_before_disable', 2.0)
        self.declare_parameter('dynamixel_port', '/dev/dynamixel')
        self.declare_parameter('baud_rate', 1000000)

        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.hold_time = self.get_parameter('hold_before_disable').value
        self.dxl_port = self.get_parameter('dynamixel_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        self.estop_active = False
        self._torque_disabled = False
        self._press_time = 0.0

        # Try to import GPIO library
        self._gpio = None
        self._setup_gpio()

        # Poll GPIO at 100 Hz
        self.create_timer(0.01, self.poll_gpio)

        self.get_logger().info(
            f'E-Stop node started (GPIO {self.gpio_pin}, '
            f'hold {self.hold_time}s before torque off)')

    def _setup_gpio(self):
        """Set up GPIO input with pull-up resistor."""
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self._gpio = GPIO
            self.get_logger().info(f'GPIO {self.gpio_pin} configured (RPi.GPIO)')
        except ImportError:
            try:
                import gpiod
                chip = gpiod.Chip('gpiochip0')
                self._gpiod_line = chip.get_line(self.gpio_pin)
                self._gpiod_line.request(
                    consumer='estop',
                    type=gpiod.LINE_REQ_DIR_IN,
                    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
                self._gpio = 'gpiod'
                self.get_logger().info(f'GPIO {self.gpio_pin} configured (gpiod)')
            except (ImportError, Exception) as e:
                self.get_logger().error(
                    f'No GPIO library available ({e}). '
                    f'E-Stop will only work via /emergency_stop topic.')
                self._gpio = None

        # Also subscribe to software E-Stop topic for fallback
        self.create_subscription(
            Bool, '/emergency_stop_sw', self._sw_estop_cb, 10)

    def _sw_estop_cb(self, msg):
        """Software E-Stop fallback (from any terminal)."""
        if msg.data and not self.estop_active:
            self._activate_estop()
        elif not msg.data and self.estop_active:
            self._deactivate_estop()

    def _read_pin(self):
        """Read the GPIO pin state. Returns True if button pressed (active low)."""
        if self._gpio is None:
            return False
        try:
            if self._gpio == 'gpiod':
                return self._gpiod_line.get_value() == 0
            else:
                return self._gpio.input(self.gpio_pin) == 0
        except Exception:
            return False

    def poll_gpio(self):
        """Poll GPIO pin and manage E-Stop state."""
        button_pressed = self._read_pin()

        if button_pressed and not self.estop_active:
            self._activate_estop()
        elif not button_pressed and self.estop_active:
            self._deactivate_estop()

        # Check if we should disable torque (after hold time)
        if self.estop_active and not self._torque_disabled:
            elapsed = time.monotonic() - self._press_time
            if elapsed >= self.hold_time:
                self._disable_torque()

        # Keep publishing E-Stop state
        if self.estop_active:
            self.estop_pub.publish(Bool(data=True))

    def _activate_estop(self):
        """Button pressed — activate emergency stop."""
        self.estop_active = True
        self._torque_disabled = False
        self._press_time = time.monotonic()

        self.estop_pub.publish(Bool(data=True))
        self.get_logger().warn(
            f'EMERGENCY STOP ACTIVATED — holding position, '
            f'torque off in {self.hold_time}s')

    def _deactivate_estop(self):
        """Button released — deactivate emergency stop."""
        self.estop_active = False

        if self._torque_disabled:
            self._enable_torque()
            self._torque_disabled = False

        self.estop_pub.publish(Bool(data=False))
        self.get_logger().info('E-Stop released — torque re-enabled')

    def _disable_torque(self):
        """Disable torque on all Dynamixel servos via SDK."""
        self.get_logger().warn('Disabling servo torque (arm going limp)')
        self._set_torque(0)
        self._torque_disabled = True

    def _enable_torque(self):
        """Re-enable torque on all Dynamixel servos."""
        self.get_logger().info('Re-enabling servo torque')
        self._set_torque(1)

    def _set_torque(self, enable):
        """Write torque enable register on all servos."""
        try:
            port = PortHandler(self.dxl_port)
            packet = PacketHandler(PROTOCOL_VERSION)
            if not port.openPort():
                self.get_logger().error(f'Cannot open {self.dxl_port} for torque control')
                return
            port.setBaudRate(self.baud_rate)

            for sid in SERVO_IDS:
                result, error = packet.write1ByteTxRx(
                    port, sid, TORQUE_ENABLE_ADDR, enable)
                if result != 0:
                    self.get_logger().error(
                        f'Torque {"ON" if enable else "OFF"} failed for ID {sid}')
                else:
                    self.get_logger().info(
                        f'Servo {sid}: torque {"ON" if enable else "OFF"}')

            port.closePort()
        except Exception as e:
            self.get_logger().error(f'Torque control error: {e}')

    def destroy_node(self):
        if self._gpio and self._gpio != 'gpiod':
            try:
                self._gpio.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
