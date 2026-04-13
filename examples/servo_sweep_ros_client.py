"""
ROS2 client that sweeps a servo by calling /dexi/servo_control.

Assumes servo_pwm_service (or any node advertising
dexi_interfaces/srv/ServoControl on /dexi/servo_control) is running.
Sweeps BCM pin 21 from 0 -> 180 -> 0 a few times, then parks at 90.

Usage:
  ros2 run dexi_gpio servo_sweep_ros_client
  python3 servo_sweep_ros_client.py
"""
import time

import rclpy
from rclpy.node import Node

from dexi_interfaces.srv import ServoControl

PIN = 21
CYCLES = 3
STEP = 5
DWELL_S = 0.05


class ServoSweepClient(Node):

    def __init__(self):
        super().__init__('servo_sweep_client')
        self.client = self.create_client(ServoControl, '/dexi/servo_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/dexi/servo_control not available, waiting...')
        self.get_logger().info('Connected to /dexi/servo_control')

    def set_angle(self, angle: int):
        req = ServoControl.Request()
        req.pin = PIN
        req.angle = int(angle)
        req.min_pw = 0  # 0 -> node uses its default (500 us)
        req.max_pw = 0  # 0 -> node uses its default (2500 us)
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = ServoSweepClient()

    try:
        client.set_angle(0)
        time.sleep(0.5)

        for c in range(CYCLES):
            client.get_logger().info(f'Cycle {c + 1}/{CYCLES}: 0 -> 180')
            for a in range(0, 181, STEP):
                client.set_angle(a)
                time.sleep(DWELL_S)
            client.get_logger().info(f'Cycle {c + 1}/{CYCLES}: 180 -> 0')
            for a in range(180, -1, -STEP):
                client.set_angle(a)
                time.sleep(DWELL_S)

        client.get_logger().info('Parking at 90')
        client.set_angle(90)
        time.sleep(0.5)
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
