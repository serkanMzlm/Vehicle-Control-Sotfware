cmake_minimum_required(VERSION 3.8)
project(time_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(time_utility ${PROJECT_SOURCE_DIR}/src/tic_toc.cpp)
ament_export_targets(time_utility HAS_LIBRARY_TARGET)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS time_utility
  EXPORT time_utility
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
