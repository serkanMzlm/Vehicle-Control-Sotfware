cmake_minimum_required(VERSION 3.8)
project(geometry_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

include_directories(include)
add_library(geometry_utils ${PROJECT_SOURCE_DIR}/src/geometry_utils.cpp)

target_include_directories(geometry_utils PUBLIC 
                            "$<INSTALL_INTERFACE:include/>"
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

ament_target_dependencies(geometry_utils)
ament_export_libraries(geometry_utils)
target_link_libraries(geometry_utils Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS geometry_utils
  EXPORT geometry_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)

ament_package()
