cmake_minimum_required(VERSION 3.8)
project(geometric_operations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

include_directories(include)
file(GLOB LIB_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(geometric_operations ${LIB_FILE})

target_include_directories(geometric_operations PUBLIC 
                            "$<INSTALL_INTERFACE:include/>"
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

ament_target_dependencies(geometric_operations)
ament_export_libraries(geometric_operations)
target_link_libraries(geometric_operations Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS geometric_operations
  EXPORT geometric_operations
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)

ament_package()
