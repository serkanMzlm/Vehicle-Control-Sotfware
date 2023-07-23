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
add_executable(controller_node ${SOURCE_CODE})

ament_target_dependencies(controller_node 
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
