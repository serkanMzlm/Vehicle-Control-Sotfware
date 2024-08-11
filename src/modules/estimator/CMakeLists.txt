cmake_minimum_required(VERSION 3.8)
project(estimator)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package (Eigen3 3.3 REQUIRED)
find_package (filterToolsLib REQUIRED)
find_package (mathToolsLib REQUIRED)

include_directories(include)

file(GLOB_RECURSE SRC_FILE src/*.cpp)
add_executable(estimator_node ${SRC_FILE})

ament_target_dependencies(
  estimator_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

target_link_libraries(estimator_node Eigen3::Eigen filterToolsLib mathToolsLib)

install(TARGETS
        estimator_node
        DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
