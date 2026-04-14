"""
Minimal Python example: open then close a servo via /dexi/servo_control.

Mirrors the Node-RED flow in examples/node_red_servo_flow.json -- two calls
to the ServoControl service, one for open (angle 150) and one for close
(angle 30), with a pause in between. Intended for competition payload drops
and similar one-shot mechanisms.

Prerequisites:
  - servo_pwm_service running on the companion computer
    (ros2 launch dexi_gpio servo_pwm.launch.py)
  - Servo wired per the README.

Usage:
  ros2 run dexi_gpio servo_open_close
  python3 servo_open_close.py
"""
import time

import rclpy
from rclpy.node import Node

from dexi_interfaces.srv import ServoControl

PIN = 21
OPEN_ANGLE = 150
CLOSE_ANGLE = 30
HOLD_SECONDS = 2.0


class ServoOpenClose(Node):

    def __init__(self):
        super().__init__('servo_open_close')
        self.client = self.create_client(ServoControl, '/dexi/servo_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/dexi/servo_control not available, waiting...')

    def set_angle(self, angle: int):
        req = ServoControl.Request()
        req.pin = PIN
        req.angle = int(angle)
        req.min_pw = 0  # 0 -> node default (500 us)
        req.max_pw = 0  # 0 -> node default (2500 us)
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'angle={angle} -> success={result.success} msg={result.message}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ServoOpenClose()
    try:
        node.get_logger().info(f'Opening servo (angle {OPEN_ANGLE})')
        node.set_angle(OPEN_ANGLE)
        time.sleep(HOLD_SECONDS)

        node.get_logger().info(f'Closing servo (angle {CLOSE_ANGLE})')
        node.set_angle(CLOSE_ANGLE)
        time.sleep(HOLD_SECONDS)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
