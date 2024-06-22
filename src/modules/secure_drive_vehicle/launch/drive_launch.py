import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

# The program is ensured to terminate when the simulation is closed
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

gz_sim_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")

if gz_sim_resource_path:
    paths = gz_sim_resource_path.split(":")
    desired_path = None

    for path in paths:
        if "Secure-Drive-Vehicle" in path:
            desired_path = path
            break
    
    if desired_path:
        directory = desired_path.split("/")
        sim_world_path = "/".join(directory[:-1])
        sim_world_file = simulation_world_path = Path(sim_world_path, "worlds", "land_vehicle.sdf")
        simulation = ExecuteProcess(cmd=["gz", "sim", "-r", sim_world_file])
    else:
        print("Secure-Drive-Vehicle directory not found in GZ_SIM_RESOURCE_PATH.")
        simulation = ExecuteProcess(cmd=["gz", "sim"])

else:
    print("GZ_SIM_RESOURCE_PATH environment variable is not set.")
    simulation = ExecuteProcess(cmd=["gz", "sim"])

bridge_directory = get_package_share_directory('secure_drive_vehicle')
bridge_file = PythonLaunchDescriptionSource(
                    os.path.join(bridge_directory,'launch','gazebo_bridge_launch.py')
                )

bridge_launch = IncludeLaunchDescription(bridge_file)

config_file = os.path.join(
        get_package_share_directory('secure_drive_vehicle'),
        'config',
        'params.yaml'
    )

rviz = ExecuteProcess( cmd=["rviz2"] )

control_unit = Node(
    package="control_unit",                                              
    executable="control_unit_node",
    parameters=[config_file],
    output="screen"
)

commander = Node(
    package="commander",                                              
    executable="commander_node",
    # ros_arguments=[ "--log-level", "controller_node:=debug",
    #                 "--remap", "Command_node:=my_command_node"],
    parameters=[config_file],
    output="screen"
)

camera_recorder = Node(
    package="camera_streamer",
    executable="recorder_node",
    parameters=[config_file],
)

camera_player = Node(
    package="camera_streamer",
    executable="player_node",
    parameters=[config_file],
)

shutdown = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=simulation,
      on_exit=[EmitEvent(event=Shutdown)]
    )
)

def generate_launch_description():
    return LaunchDescription([
        control_unit,
        # commander,
        
        # camera_recorder,
        # camera_player,

        simulation,
        bridge_launch,
        shutdown        
    ]) 
