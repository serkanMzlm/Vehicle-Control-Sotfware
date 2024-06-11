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


config_file = os.path.join(
        get_package_share_directory('secure_drive_vehicle'),
        'config',
        'params.yaml'
    )


rviz = ExecuteProcess( cmd=["rviz2"] )

control_unit = Node(
    package="control_unit",                                              
    executable="control_unit_node",
    # ros_arguments=[ "--remap", "Command_node:=my_command_node"
        # "--log-level", "Command_node:=debug"],
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

##################### BRIDGE #####################
bridge_control = Node(
    package="ros_gz_bridge",                                               # ros_ign_bridge is used in an older version
    executable="parameter_bridge",
    arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
    output="screen"
)

bridge_keyboard = Node(
    package="ros_gz_bridge",                                               # ros_ign_bridge is used in an older version
    executable="parameter_bridge",
    arguments=["/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32"],
    remappings=[("/keyboard/keypress","/keypress")],
    output="screen"
)

bridge_lidar = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
    remappings=[("/lidar/points","/lidar")],
)

bridge_laser_scan = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],  # Only one axis is being provided
    remappings=[("/lidar","/scan")],
)

bridge_camera = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/camera@sensor_msgs/msg/Image[gz.msgs.Image"]
)

bridge_imu = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"]
)

def generate_launch_description():
    return LaunchDescription([
        control_unit,
        # commander,
        # camera_recorder,
        simulation,
        # camera_player,

        bridge_control,
        bridge_keyboard,
        # bridge_laser_scan,
        bridge_lidar,
        bridge_camera,
        # bridge_imu,
        shutdown        
    ]) 
