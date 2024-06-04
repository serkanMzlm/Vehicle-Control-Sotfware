cmake_minimum_required(VERSION 3.8)
project(num_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include)
add_library(num_tools ${PROJECT_SOURCE_DIR}/src/num_tools.cpp)

target_include_directories(num_tools PUBLIC 
                            "$<INSTALL_INTERFACE:include/>"
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

ament_target_dependencies(num_tools)
ament_export_libraries(num_tools)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS num_tools
  EXPORT num_tools
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)

ament_package()
