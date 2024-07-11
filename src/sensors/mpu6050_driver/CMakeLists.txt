cmake_minimum_required(VERSION 3.8)
project(mpu6050_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB_RECURSE MPU6050_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(mpu6050_driver_node ${MPU6050_FILES})

ament_target_dependencies(mpu6050_driver_node rclcpp sensor_msgs)
target_link_libraries(mpu6050_driver_node i2c)
target_include_directories( mpu6050_driver_node 
					PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
					$<INSTALL_INTERFACE:include>
)

install(TARGETS mpu6050_driver_node 
		DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()
