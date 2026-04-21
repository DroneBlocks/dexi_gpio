"""
DMA-PWM servo control service for DEXI GPIO via pigpio.

Exposes the standard dexi_interfaces/srv/ServoControl service on
/dexi/servo_control, driving a hobby servo through the pigpio daemon.
pigpio generates servo pulses via DMA in the Pi's hardware timer, so
timing is not affected by CPU load or kernel scheduling -- a drop-in
replacement for the RPi.GPIO software-PWM approach that eliminates the
jitter ordinary software PWM exhibits under load.

Intended for ARK CM4 (and other non-PCA9685 platforms) where the
existing dexi_cpp/servo_controller is unavailable.

Prerequisites on the host:
  sudo apt install -y pigpio python3-pigpio
  sudo systemctl enable --now pigpiod

Default configuration: servos on BCM GPIO pins [21, 22] with a
500..2500 us pulse range. Pins are configurable via the 'servo_pins'
ROS parameter (int array).
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import pigpio

from dexi_interfaces.srv import ServoControl

DEFAULT_MIN_PW_US = 500
DEFAULT_MAX_PW_US = 2500
MIN_ANGLE = 0
MAX_ANGLE = 180


class ServoPwmService(Node):

    def __init__(self):
        super().__init__('servo_pwm_service')

        self.declare_parameter('servo_pins', [21, 22])
        self.declare_parameter('pigpiod_host', 'localhost')
        self.servo_pins = set(int(p) for p in self.get_parameter('servo_pins').value)
        pigpiod_host = str(self.get_parameter('pigpiod_host').value)

        self.pi = pigpio.pi(pigpiod_host)
        if not self.pi.connected:
            self.get_logger().error(
                f'Could not connect to pigpiod on {pigpiod_host}. '
                f'Is the daemon running? Try: sudo systemctl enable --now pigpiod'
            )
            raise RuntimeError('pigpiod not reachable')

        # Start with each line idle (no pulses) so the servos are limp
        # until the first command.
        for p in self.servo_pins:
            self.pi.set_servo_pulsewidth(p, 0)

        # Absolute service name so it resolves to /dexi/servo_control
        # regardless of any namespace the node is launched under.
        self.srv = self.create_service(
            ServoControl,
            '/dexi/servo_control',
            self.servo_callback,
        )

        self.get_logger().info(
            f'servo_pwm_service ready on BCM pins {sorted(self.servo_pins)} '
            f'via pigpiod@{pigpiod_host}, service: /dexi/servo_control'
        )

    def servo_callback(self, request, response):
        if request.pin not in self.servo_pins:
            response.success = False
            response.message = (
                f'Pin {request.pin} not configured. '
                f'This node only drives BCM pins {sorted(self.servo_pins)}.'
            )
            self.get_logger().warn(response.message)
            return response

        angle = max(MIN_ANGLE, min(MAX_ANGLE, int(request.angle)))
        min_pw = request.min_pw if request.min_pw > 0 else DEFAULT_MIN_PW_US
        max_pw = request.max_pw if request.max_pw > 0 else DEFAULT_MAX_PW_US

        if max_pw <= min_pw:
            response.success = False
            response.message = f'Invalid pulse range: min_pw={min_pw}, max_pw={max_pw}'
            self.get_logger().warn(response.message)
            return response

        pulse_us = int(min_pw + (max_pw - min_pw) * (angle / float(MAX_ANGLE)))

        try:
            self.pi.set_servo_pulsewidth(request.pin, pulse_us)
        except Exception as exc:
            response.success = False
            response.message = f'pigpio set_servo_pulsewidth failed: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f'pin={request.pin} angle={angle} pulse={pulse_us}us'
        self.get_logger().info(response.message)
        return response

    def shutdown(self):
        try:
            # Pulsewidth 0 releases each line so the servos stop drawing current.
            for p in self.servo_pins:
                self.pi.set_servo_pulsewidth(p, 0)
        finally:
            self.pi.stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoPwmService()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
