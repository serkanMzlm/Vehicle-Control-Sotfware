cmake_minimum_required(VERSION 3.8)
project(filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB LIB_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(filters ${LIB_FILE})

target_include_directories(filters PUBLIC 
                            "$<INSTALL_INTERFACE:include>"
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

ament_target_dependencies(filters)
ament_export_libraries(filters)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS filters
  EXPORT filters
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_package()
