"""
Software-PWM servo control service for DEXI GPIO.

Exposes the standard dexi_interfaces/srv/ServoControl service on
/dexi/servo_control, driving a hobby servo via RPi.GPIO software PWM.

Intended for ARK CM4 (and other non-PCA9685 platforms) where the existing
dexi_cpp/servo_controller is unavailable. Software PWM is bench-grade
timing only -- not recommended for flight use.

Default configuration: single servo on BCM GPIO 21, 50 Hz frame,
0.5 ms to 2.5 ms pulse range. Pin is configurable via the 'servo_pin'
ROS parameter.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import RPi.GPIO as GPIO

from dexi_interfaces.srv import ServoControl

SERVO_FREQ_HZ = 50
FRAME_US = 1_000_000.0 / SERVO_FREQ_HZ  # 20000 us at 50 Hz

DEFAULT_MIN_PW_US = 500
DEFAULT_MAX_PW_US = 2500
MIN_ANGLE = 0
MAX_ANGLE = 180


class ServoPwmService(Node):

    def __init__(self):
        super().__init__('servo_pwm_service')

        self.declare_parameter('servo_pin', 21)
        self.servo_pin = int(self.get_parameter('servo_pin').value)

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.servo_pin, SERVO_FREQ_HZ)
        self.pwm.start(0)  # idle: no pulse until first command

        # Absolute service name so it resolves to /dexi/servo_control
        # regardless of any namespace the node is launched under.
        self.srv = self.create_service(
            ServoControl,
            '/dexi/servo_control',
            self.servo_callback,
        )

        self.get_logger().info(
            f'servo_pwm_service ready on BCM pin {self.servo_pin} '
            f'@ {SERVO_FREQ_HZ} Hz, service: /dexi/servo_control'
        )

    def servo_callback(self, request, response):
        if request.pin != self.servo_pin:
            response.success = False
            response.message = (
                f'Pin {request.pin} not configured. '
                f'This node only drives BCM pin {self.servo_pin}.'
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

        pulse_us = min_pw + (max_pw - min_pw) * (angle / float(MAX_ANGLE))
        duty_pct = (pulse_us / FRAME_US) * 100.0

        try:
            self.pwm.ChangeDutyCycle(duty_pct)
        except Exception as exc:
            response.success = False
            response.message = f'PWM update failed: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'pin={self.servo_pin} angle={angle} '
            f'pulse={pulse_us:.0f}us duty={duty_pct:.2f}%'
        )
        self.get_logger().info(response.message)
        return response

    def shutdown(self):
        try:
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
        finally:
            GPIO.cleanup(self.servo_pin)


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
