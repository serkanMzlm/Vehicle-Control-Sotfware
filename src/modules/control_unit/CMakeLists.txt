cmake_minimum_required(VERSION 3.8)
project(control_unit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(num_tools REQUIRED)
find_package(time_utils REQUIRED)

file(GLOB_RECURSE SRC_FILE src/*.cpp)
include_directories(include)

add_executable(control_unit_node ${SRC_FILE})
ament_target_dependencies(
  control_unit_node
  rclcpp
  std_msgs
  sensor_msgs
  time_utils
  num_tools
)

install(TARGETS
        control_unit_node
        DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
