from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

# Similasyon kapatılınca programda sonlanması sağlanır.
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


ign_ros_pkg_path      = get_package_share_directory("ros_ign_gazebo")
land_vehicle_path     = get_package_share_directory("land_vehicle")
simulation_world_path = Path(land_vehicle_path, "worlds", "land_vehicle.sdf")
simulation_model_path = Path(land_vehicle_path, "models")

simulation = ExecuteProcess(
    cmd=["gz", "sim", "-r", simulation_world_path],
    # output="screen"
)

control = Node(
            package="ros_gz_bridge",                                               # ros_ign_bridge eski versiyonda kullanılır.
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"
            ],
            # ros_arguments=["/cmd_vel:=/cmd_vel_ros"]  remapping ile aynı
            # remappings=[("/cmd_vel","/cmd_vel_ros")],
            output="screen"
          )

lidar = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                # "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"  # Sadece bir ekseni veriyor
                "/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            remappings=[("/lidar/points","/lidar")],
          )

shutdown = RegisterEventHandler(
            event_handler=OnProcessExit(
              target_action=simulation,
              on_exit=[EmitEvent(event=Shutdown)]
            )
          )

def generate_launch_description():
    return LaunchDescription([
          simulation,
          control,
          lidar,
          shutdown        
    ]) 
