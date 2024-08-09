cmake_minimum_required(VERSION 3.8)
project(commander)

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

# find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(num_tools REQUIRED)
find_package(geometric_operations REQUIRED)

FILE(GLOB_RECURSE SRC_FILE src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(commander_node ${SRC_FILE})

ament_target_dependencies(
    commander_node 
    rclcpp 
    tf2_ros
    PCL
    pcl_conversions
    visualization_msgs                
    sensor_msgs
    geometry_msgs
    num_tools
    geometric_operations
)

target_link_libraries(commander_node Eigen3::Eigen)

install(TARGETS
  commander_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
