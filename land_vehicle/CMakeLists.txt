cmake_minimum_required(VERSION 3.8)
project(land_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_ign_gazebo REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   DEPENDENCIES
#   sensor_msgs geometry_msgs action_msgs
# )




include_directories(${PROJECT_SOURCE_DIR}/include)

# install(
#   TARGETS

#   DESTINATION lib/${PROJECT_NAME}
# )


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)

ament_package()
