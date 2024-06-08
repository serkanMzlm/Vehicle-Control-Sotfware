cmake_minimum_required(VERSION 3.8)
project(time_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(time_utils ${PROJECT_SOURCE_DIR}/src/tic_toc.cpp)

target_include_directories(time_utils PUBLIC 
                            "$<INSTALL_INTERFACE:include>"
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

# ament_export_targets(time_utils HAS_LIBRARY_TARGET)

ament_target_dependencies(time_utils)
ament_export_libraries(time_utils)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS time_utils
  EXPORT time_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_package()
