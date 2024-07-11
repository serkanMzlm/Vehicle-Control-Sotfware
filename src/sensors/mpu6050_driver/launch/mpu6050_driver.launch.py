from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

config = os.path.join(
        get_package_share_directory('mpu6050_driver'),
        'config',
        'params.yml'
    )

mpu6050driver_node = Node(
        package='mpu6050_driver',
        executable='mpu6050_driver_node',
        output="screen",
        emulate_tty=True,
        # respawn=True,
        remappings=[
                ('/imu', '/mpu6050')
        ],
        parameters=[config]
    )

def generate_launch_description():
   return LaunchDescription([
        mpu6050driver_node
   ])