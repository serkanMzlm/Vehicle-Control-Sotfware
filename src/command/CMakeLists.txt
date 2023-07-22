cmake_minimum_required(VERSION 3.8)
project(command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCE_CODE src/command.cpp)

add_executable(command_node ${SOURCE_CODE})

ament_target_dependencies(command_node 
  rclcpp
  geometry_msgs
)

install(TARGETS
  command_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
