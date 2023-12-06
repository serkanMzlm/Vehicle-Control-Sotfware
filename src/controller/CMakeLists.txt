cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.3 REQUIRED)

FILE(GLOB_RECURSE SRC_FILE src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(controller_node ${SRC_FILE})

ament_target_dependencies(
    controller_node 
    rclcpp 
    tf2_ros
    pcl_ros
    pcl_conversions
    visualization_msgs                
    sensor_msgs geometry_msgs
)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
