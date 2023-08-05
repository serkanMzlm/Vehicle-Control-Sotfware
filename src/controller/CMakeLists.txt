cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SOURCE_CODE src/controller.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(controller_node src/controller.cpp)
add_executable(command_node src/command.cpp)

ament_target_dependencies(controller_node 
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(command_node 
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  controller_node
  command_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
