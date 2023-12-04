cmake_minimum_required(VERSION 3.8)
project(remote_selector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB_RECURSE SRC_FILE src/*.cpp)

include_directories(include)
add_executable(remote_selector_node ${SRC_FILE})
ament_target_dependencies(remote_selector_node rclcpp sensor_msgs)

install(TARGETS 
  remote_selector_node  
  DESTINATION 
  lib/${PROJECT_NAME}
)

ament_package()
