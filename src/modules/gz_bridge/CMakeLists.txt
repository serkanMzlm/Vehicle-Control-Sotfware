cmake_minimum_required(VERSION 3.8)
project(gz_bridge)

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
find_package(gz-cmake3 REQUIRED)
find_package(geometryToolsLib REQUIRED)

set(GZ_DISTRIBUTION "Garden")

gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-msgs10 REQUIRED)
gz_find_package(sdformat14 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
gz_find_package(gz-transport13 VERSION 13.1 REQUIRED COMPONENTS log parameters)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

file(GLOB_RECURSE SRC_FILE src/*.cpp)
include_directories(include)

add_executable(gz_bridge_node ${SRC_FILE})

target_link_libraries(gz_bridge_node
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

ament_target_dependencies(
  gz_bridge_node
  rclcpp
)

target_link_libraries(gz_bridge_node geometryToolsLib)

install(TARGETS
        gz_bridge_node
        DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
