import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

gz_sim_resource_path = get_package_share_directory('vehicle_control_software')
urdf_file = os.path.join(gz_sim_resource_path, "urdf", "vehicle.urdf")


robot_state_pub = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output='screen',
    arguments=[urdf_file],
)

joint_state_pub = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    arguments=[urdf_file],
)

def generate_launch_description():
    return LaunchDescription([
        robot_state_pub,
        joint_state_pub       
    ]) 