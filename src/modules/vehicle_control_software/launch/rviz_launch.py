import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path


gz_sim_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")

if gz_sim_resource_path:
    paths = gz_sim_resource_path.split(":")
    desired_path = None

    for path in paths:
        if "Vehicle-Control-Sotfware" in path:
            desired_path = path
            break
    
    if desired_path:
        directory = desired_path.split("/")
        sim_world_path = "/".join(directory[:-1])
        sim_launch_file = simulation_world_path = Path(sim_world_path, "models", "urdf", "vehicle.urdf")


robot_state_pub = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output='screen',
    arguments=[sim_launch_file],
)

joint_state_pub = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    arguments=[sim_launch_file],
)

def generate_launch_description():
    return LaunchDescription([
        robot_state_pub,
        joint_state_pub       
    ]) 