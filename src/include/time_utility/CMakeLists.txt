cmake_minimum_required(VERSION 3.8)
project(time_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(time_utility ${PROJECT_SOURCE_DIR}/src/tic_toc.cpp)

target_include_directories(time_utility PUBLIC 
                            "$<INSTALL_INTERFACE:include>"
                            ${PROJECT_SOURCE_DIR}/include/
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

# ament_export_targets(time_utility HAS_LIBRARY_TARGET)

ament_target_dependencies(time_utility)
ament_export_libraries(time_utility)

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

ament_export_include_directories(include)
ament_package()
