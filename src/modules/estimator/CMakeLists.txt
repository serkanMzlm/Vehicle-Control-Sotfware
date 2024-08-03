cmake_minimum_required(VERSION 3.8)
project(estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(num_tools REQUIRED)
find_package(time_utils REQUIRED)
find_package(geometric_operations REQUIRED)
find_package (filters REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

include_directories(include)

file(GLOB_RECURSE SRC_FILE src/*.cpp)
add_executable(estimator_node ${SRC_FILE})

ament_target_dependencies(
  estimator_node
  filters
  rclcpp
  num_tools
  geometry_msgs
  sensor_msgs
  time_utils
  geometric_operations
)

target_link_libraries(estimator_node Eigen3::Eigen)

install(TARGETS
        estimator_node
        DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
