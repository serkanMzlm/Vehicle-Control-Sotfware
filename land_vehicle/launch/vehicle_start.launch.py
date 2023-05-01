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
simulation_world_path = Path(land_vehicle_path, "worlds", "land_vehicle_world.sdf")
simulation_model_path = Path(land_vehicle_path, "models")

# Çalışmadı
# simulation = IncludeLaunchDescription(
#               PythonLaunchDescriptionSource(
#                 launch_file_path = Path(ign_ros_pkg_path, "launch", "ign_gazebo.launch.py")
#               ),
#               launch_arguments = {"gz_args": "-r " + simulation_world_path}.items()  # ign_args eski versiyonda kullanılır
#             ),

simulation = ExecuteProcess(
    cmd=["ign", "gazebo", "-r", simulation_world_path],
    output="screen"
)



def generate_launch_description():
    return LaunchDescription([
        # SetEnvironmentVariable(
        # name="IGN_GAZEBO_RESOURCE_PATH",
        # value=simulation_model_path
        # ),
        # IncludeLaunchDescription(
        #       PythonLaunchDescriptionSource(
        #         launch_file_path = Path(ign_ros_pkg_path, "launch", "ign_gazebo.launch.py")
        #       ),
        #       launch_arguments = {"gz_args": "-r " + simulation_world_path}.items()  # ign_args eski versiyonda kullanılır
        #     ),
        simulation,
        Node(
          package="ros_gz_bridge",                                               # ros_ign_bridge eski versiyonda kullanılır.
          executable="parameter_bridge",
          arguments=[
              "/model/land_vehicle/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
          ],
          # ros_arguments=["model/land_vehicle/cmd_vel:=/cmd_vel"]  remapping ile aynı
          remappings=[("model/land_vehicle/cmd_vel","/cmd_vel")],
          output="screen"
        ),
        RegisterEventHandler(
          event_handler=OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown)]
          )
        )
    ])
