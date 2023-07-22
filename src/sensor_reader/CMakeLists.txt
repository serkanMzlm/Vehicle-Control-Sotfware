cmake_minimum_required(VERSION 3.8)
project(sensor_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCE_CODE src/sensors_reader.cpp)

add_executable(sensor_reader_node ${SOURCE_CODE})

ament_target_dependencies(sensor_reader_node 
  rclcpp
  sensor_msgs
)

install(TARGETS
  sensor_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
