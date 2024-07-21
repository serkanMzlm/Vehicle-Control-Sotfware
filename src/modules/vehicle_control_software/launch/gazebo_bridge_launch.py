from launch import LaunchDescription
from launch_ros.actions import Node


bridge_control = Node(
    package="ros_gz_bridge",                                               # ros_ign_bridge is used in an older version
    executable="parameter_bridge",
    arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
    output="log"
)

bridge_keyboard = Node(
    package="ros_gz_bridge",                                               # ros_ign_bridge is used in an older version
    executable="parameter_bridge",
    arguments=["/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32"],
    remappings=[("/keyboard/keypress","/keypress")],
    output="log"
)

bridge_lidar = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
    remappings=[("/lidar/points","/lidar")],
    output="log"
)

bridge_laser_scan = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],  # Only one axis is being provided
    remappings=[("/lidar","/laser_scan")],
    output="log"
)

bridge_camera = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/camera@sensor_msgs/msg/Image[gz.msgs.Image"],
    output="log"
)

bridge_imu = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
    output="log"
)

def generate_launch_description():
    return LaunchDescription([
        bridge_control,
        bridge_keyboard,
        bridge_laser_scan,
        bridge_lidar,
        bridge_camera,
        # bridge_imu,
    ]) 
