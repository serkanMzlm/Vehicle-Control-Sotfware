cmake_minimum_required(VERSION 3.8)
project(control_unit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_PREFIX_PATH)
  set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}; $ENV{VEHICLE_CONTRAL_SOFTWARE}/src/include/lib/cmake")
else()
  set(CMAKE_PREFIX_PATH "$ENV{VEHICLE_CONTRAL_SOFTWARE}/src/include/lib/cmake")
  message("Path: ${CMAKE_PREFIX_PATH}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(mathToolsLib REQUIRED)

file(GLOB_RECURSE SRC_FILE src/*.cpp)
include_directories(include)

add_executable(control_unit_node ${SRC_FILE})
ament_target_dependencies(
  control_unit_node
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(control_unit_node mathToolsLib)

install(TARGETS
        control_unit_node
        DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
