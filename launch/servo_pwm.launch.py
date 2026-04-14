from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_gpio',
            executable='servo_pwm_service',
            name='servo_pwm_service',
            parameters=[{'servo_pin': 21}],
            output='screen',
        ),
    ])
