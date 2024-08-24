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
        if "Vehicle-Control-Sotfware" in path:
            desired_path = path
            break
    
    if desired_path:
        directory = desired_path.split("/")
        sim_world_path = "/".join(directory[:-1])
        sim_world_file = simulation_world_path = Path(sim_world_path, "worlds", "land_vehicle.sdf")
        simulation = ExecuteProcess(cmd=["gz", "sim", "-r", sim_world_file])
    else:
        print("Vehicle-Control-Sotfware directory not found in GZ_SIM_RESOURCE_PATH.")
        simulation = ExecuteProcess(cmd=["gz", "sim"])

else:
    print("GZ_SIM_RESOURCE_PATH environment variable is not set.")
    simulation = ExecuteProcess(cmd=["gz", "sim"])

bridge_directory = get_package_share_directory('vehicle_control_software')
bridge_file = PythonLaunchDescriptionSource(
                    os.path.join(bridge_directory,'launch','gz_bridge_launch.py')
                )

bridge_launch = IncludeLaunchDescription(bridge_file)

config_file = os.path.join(
        get_package_share_directory('vehicle_control_software'),
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

create_model = Node(
    package="create_model",                                              
    executable="create_model_node",
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

player_node = Node(
    package="camera",
    executable="player_node",
    parameters=[config_file],
)
recorder_node = Node(
    package="camera",
    executable="recorder_node",
    parameters=[config_file],
)
camera_player = Node(
    package="camera",
    executable="camera_player",
    parameters=[config_file],
)
screenshot_node = Node(
    package="camera",
    executable="screenshot_node",
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
        commander,
        create_model,
        
        # player_node,
        # recorder_node,
        # camera_player,
        # screenshot_node,

        simulation,
        bridge_launch,
        shutdown        
    ]) 
