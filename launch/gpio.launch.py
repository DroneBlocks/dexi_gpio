import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('dexi_gpio'),
      'config',
      'gpio.yaml'
      )

   return LaunchDescription([
      Node(
         package='dexi_gpio',
         namespace='dexi',
         executable='gpio_writer_service',
         name='gpio_writer_service',
         parameters=[config]
      ),
      Node(
         package='dexi_gpio',
         namespace='dexi',
         executable='gpio_reader',
         name='gpio_input_22',
         parameters = [
            {'pin': 22}
         ]
      ),
      Node(
         package='dexi_gpio',
         namespace='dexi',
         executable='gpio_reader',
         name='gpio_input_23',
         parameters = [
            {'pin': 23}
         ]
      ),
      Node(
         package='dexi_gpio',
         namespace='dexi',
         executable='gpio_reader',
         name='gpio_input_24',
         parameters = [
            {'pin': 24}
         ]
      ),
   ])