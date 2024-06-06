cmake_minimum_required(VERSION 3.8)
project(sensor_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(pcl_ros REQUIRED )
find_package(pcl_conversions REQUIRED )

include_directories(include ${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(SOURCE_CODE src/sensor_listener_node.cpp)

add_executable(sensor_listener_node ${SOURCE_CODE})

target_link_libraries(sensor_listener_node ${PCL_LIBRARIES} )

ament_target_dependencies(sensor_listener_node 
  rclcpp
  sensor_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  OpenCV
)

install(TARGETS
  sensor_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
